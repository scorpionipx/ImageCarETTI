import cv2
import serial
import sys
import time
import math

import numpy as np
import RPi.GPIO as GPIO

from picamera.array import PiRGBArray
from picamera import PiCamera

from luma.core.interface.serial import i2c
from luma.core.render import canvas
from luma.oled.device import ssd1306, ssd1325, ssd1331, sh1106

import time
import os

__version__ = '1.0.0'

serial_interface = i2c(port=1, address=0x3C)
device = ssd1306(serial_interface, rotate=0, height=32)

# Box and text rendered in portrait mode
with canvas(device) as draw:
    draw.text((0, 0), "ScorpionIPX", fill="white")
    draw.text((0, 8), "ETTI v{}".format(__version__), fill="white")

OUTPUT_BASE_DIR = r'/home/pi/Documents/ETTI_CAR_OUTPUTS'


MILISECOND = 0.001
SECOND = 1


CRUISE_SPEED = 35

CENTER = 13
LEFT = 69
RIGHT = 10

print('EETI car v{}'.format(__version__))


WIDTH = 800
HEIGHT = 600

BINARY_THRESHOLD = 95
MINIMUM_AREA = 800

SWA = CENTER

CENTER_STEERING = 130
LEFT_STEERING_MAX = 180
RIGHT_STEERING_MAX = 78


CENTER_STEERING_TOLERANCE = 2
LEFT_STEERING_TOLERANCE = 3
RIGHT_STEERING_TOLERANCE = 3

stepper_pins = [31, 33, 35, 37]
        
STEPPER_FULL_STEP_SEQUENCE = [
  [1, 0, 0, 0],
  [0, 1, 0, 0],
  [0, 0, 1, 0],
  [0, 0, 0, 1],
]

CRUISE_TURNS = 55
STEP_DELAY = 0.0015

COMMAND_DELAY = 0.005
NONE_COMMAND_COUNTER_THRESHOLD = 150

ser = None
camera = None
raw_capture = None


def init_steering():
    try:
        print('Initiating Stepper...')
        GPIO.setmode(GPIO.BOARD)
        for pin in stepper_pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, 0)
        print('Stepper initiated!')
        return True            
    except Exception as err:
        print(err)
        return False


def init_camera():
    global camera
    global raw_capture
    try:
        print('Initiating camera...')
        camera = PiCamera()
        camera.close()
        camera = PiCamera()
        camera.resolution = (WIDTH, HEIGHT)
        camera.framerate = 20
        camera.color_effects = (128, 128)  # grayscale
        raw_capture = PiRGBArray(camera, size=(WIDTH, HEIGHT))
        print('Warming up PICamera...')
        time.sleep(0.5)
        print('Camera initiated!')
        return True
    except Exception as err:
        print('Failed to init camera! {}'.format(err))
        return False
        

def init_serial():
    global ser
    try:
        print('Initiating UART...')
        ser = serial.Serial(            
            port='/dev/serial0',
            baudrate = 115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1
        )
        print('UART initiated!')
        return True
    except Exception as err:
        print('Failed to init UART! {}'.format(err))
        return False


def get_steering_angle():
    global ser
    ser.read_all()  # clear buffer
    time.sleep(MILISECOND)
    
    ser.write([0x5A])
    time.sleep(MILISECOND)
    ser.write([1])
    time.sleep(2 * MILISECOND)

##    steering_angle = ord(ser.read(1))
##    print('SWA: {}'.format(steering_angle))
    if ser.in_waiting >= 1:
        steering_angle = ord(ser.read(1))
        print('SWA: {}'.format(steering_angle))
    else:
        steering_angle = -1
        print('Timeout [get_steering_angle]')

    return steering_angle


def steer_right(turns):
    sequence = STEPPER_FULL_STEP_SEQUENCE.copy()
    for i in range(turns):
        for step in range(len(sequence)):
            for pin_index, pin in enumerate(stepper_pins):
                GPIO.output(pin, sequence[step][pin_index])
                time.sleep(STEP_DELAY)
                
    for pin in stepper_pins:
        GPIO.output(pin, 0)


def steer_left(turns):
    sequence = STEPPER_FULL_STEP_SEQUENCE.copy()
    sequence.reverse()
    for i in range(turns):
        for step in range(len(sequence)):
            for pin_index, pin in enumerate(stepper_pins):
                GPIO.output(pin, sequence[step][pin_index])
                time.sleep(STEP_DELAY)
                
    for pin in stepper_pins:
        GPIO.output(pin, 0)


def center_steering():
    global CENTER_STEERING
    same_angle_cnt = 0
    angle = get_steering_angle()
    while angle == -1:
        angle = get_steering_angle()
    while abs(angle - CENTER_STEERING) > CENTER_STEERING_TOLERANCE:
        if angle < CENTER_STEERING:
            steer_left(3)
        else:
            steer_right(3)
        previous_angle = angle
        angle = get_steering_angle()
        while angle == -1:
            angle = get_steering_angle()

        if angle == previous_angle:
            same_angle_cnt += 1
            if same_angle_cnt >= 5:
                print('Stepper mechanically blocked or ADC not working properly!')
                return
        else:
            same_angle_cnt = 0
            


def steer_max_left():
    same_angle_cnt = 0
    angle = get_steering_angle()
    while angle == -1:
        angle = get_steering_angle()
    if abs(angle - LEFT_STEERING_MAX) <= LEFT_STEERING_TOLERANCE:
        return
    while abs(angle - LEFT_STEERING_MAX) > LEFT_STEERING_TOLERANCE:
        steer_left(5)
        previous_angle = angle
        angle = get_steering_angle()
        while angle == -1:
            angle = get_steering_angle()

        if angle == previous_angle:
            same_angle_cnt += 1
            if same_angle_cnt >= 5:
                print('Stepper mechanically blocked or ADC not working properly!')
                return
        else:
            same_angle_cnt = 0


def steer_max_right():
    same_angle_cnt = 0
    angle = get_steering_angle()
    while angle == -1:
        angle = get_steering_angle()
    if abs(angle - RIGHT_STEERING_MAX) <= RIGHT_STEERING_TOLERANCE:
        return
    while abs(angle - RIGHT_STEERING_MAX) > RIGHT_STEERING_TOLERANCE:
        steer_right(5)
        previous_angle = angle
        angle = get_steering_angle()
        while angle == -1:
            angle = get_steering_angle()

        if angle == previous_angle:
            same_angle_cnt += 1
            if same_angle_cnt >= 5:
                print('Stepper mechanically blocked or ADC not working properly!')
                return
        else:
            same_angle_cnt = 0


def steer(swa):
    global SWA
    if swa == CENTER:
        if SWA == CENTER:
            pass
        elif SWA == LEFT:
            steer_right(CRUISE_TURNS)
            SWA = CENTER
        elif SWA == RIGHT:
            steer_left(CRUISE_TURNS)
            SWA = CENTER
            
    elif swa == LEFT:
        if SWA == CENTER:
            steer_left(CRUISE_TURNS)
            SWA = LEFT
        elif SWA == LEFT:
            pass
        elif SWA == RIGHT:
            steer_left(2 * CRUISE_TURNS)
            SWA = LEFT
            
    elif swa == RIGHT:
        if SWA == CENTER:
            steer_right(CRUISE_TURNS)
            SWA = RIGHT
        elif SWA == LEFT:
            steer_right(2 * CRUISE_TURNS)
            SWA = RIGHT
        elif SWA == RIGHT:
            pass


def test_steering():
    if not init_steering():
        return

    steer_right(CRUISE_TURNS)
    time.sleep(SECOND)
    steer_left(CRUISE_TURNS)
    time.sleep(SECOND)
    steer_left(CRUISE_TURNS)
    time.sleep(SECOND)
    steer_right(CRUISE_TURNS)
    GPIO.cleanup()


def stop():
    global ser
    print('Stop')
    
    ser.write([0x39])
    time.sleep(MILISECOND)
    ser.write([0])
    time.sleep(MILISECOND)


def fwd(speed=100):
    global ser
    if speed < 0:
        speed = 0
    if speed > 255:
        speed = 255
    print('FWD: {}'.format(speed))
    ser.write([0x35])
    time.sleep(MILISECOND)
    ser.write([speed])
    time.sleep(5 * MILISECOND)


def bkwd(speed=100):
    global ser
    if speed < 0:
        speed = 0
    if speed > 255:
        speed = 255
    print('FWD: {}'.format(speed))
    ser.write([0x39])
    time.sleep(MILISECOND)
    ser.write([speed])
    time.sleep(5 * MILISECOND)


def test_swa():
    
    if not init_serial():
        return
    
    while True:
        get_steering_angle()
        time.sleep(.25)
        
        key = cv2.waitKey(1) & 0xFF

        # if the `q` key was pressed, break from the loop
        if key == ord("q"):
            stop()
            stop()
            cv2.destroyAllWindows()
            print('Stopped recording!')
            break


def test_serial():
    
    if not init_serial():
        return
    
    fwd(CRUISE_SPEED)
    time.sleep(2)
    stop()
    time.sleep(1)
    bkwd(CRUISE_SPEED)
    time.sleep(2)
    stop()


def auto():
    session_name = 'ETTI_CAR_{}'.format(int(time.time()))
    session_out_dir = OUTPUT_BASE_DIR + r'/' + session_name
    os.makedirs(session_out_dir)
    
    stop_auto = False
    auto_cnt = 0
    try:
        global camera
        global raw_capture
        global ser
        
        if not init_serial():
            with canvas(device) as draw:
                draw.text((0, 0), "serial init failed", fill="white")
            return

        with canvas(device) as draw:
            draw.text((0, 0), "serial interface ok", fill="white")
        time.sleep(1)

        if not init_camera():
            with canvas(device) as draw:
                draw.text((0, 0), "camera init failed", fill="white")
            return

        with canvas(device) as draw:
            draw.text((0, 0), "camera ok", fill="white")
        time.sleep(1)

            
        if not init_steering():
            with canvas(device) as draw:
                draw.text((0, 0), "GPIO init failed", fill="white")
            return

        with canvas(device) as draw:
            draw.text((0, 0), "GPIO ok", fill="white")
        time.sleep(1)

        with canvas(device) as draw:
            draw.text((0, 0), "System ok", fill="white")
        time.sleep(1)

        with canvas(device) as draw:
            draw.text((0, 0), "Centering wheels...", fill="white")
        time.sleep(1)
        center_steering()
        time.sleep(1)
        with canvas(device) as draw:
            draw.text((0, 0), "Wheels centered!", fill="white")
        time.sleep(1)
            
        with canvas(device) as draw:
            draw.text((0, 0), "System running...", fill="white")
        print('Starting recording...')
        none_counter = 0

        for frame in camera.capture_continuous(raw_capture, format="bgr", use_video_port=True):

            auto_cnt += 1
            
            square_detected = False
            triangle_detected = False
            stop_detected = False
            direction = 'NONE'
            
            image = frame.array
            grey_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(grey_image, (5, 5), 0)
    ##        ret, binary_image = cv2.threshold(grey_image, BINARY_THRESHOLD, 255, cv2.THRESH_BINARY)
            binary_image = cv2.adaptiveThreshold(blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)

            _, contours, _ = cv2.findContours(binary_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                approx = cv2.approxPolyDP(contour, 0.01 * cv2.arcLength(contour, True), True)

                if len(approx) == 3 and not triangle_detected:
                    if not cv2.isContourConvex(approx):
                        continue
                    try:
                        M = cv2.moments(approx)
                        tr_cx = int(M['m10']/M['m00'])
                        tr_cy = int(M['m01']/M['m00'])
                        tr_area = int(cv2.contourArea(approx))
                        if tr_area > MINIMUM_AREA:
                            (x, y, w, h) = cv2.boundingRect(approx)
                            ar = w / float(h)
                            if 0.3 < ar < 2.2:
                                triangle_detected = True
                                cv2.drawContours(image, [approx], 0, (0, 255, 0), )
                                cv2.putText(image, str(ar), (tr_cx, tr_cy), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), lineType=cv2.LINE_AA)
            ##                    cv2.putText(image, str(tr_area), (tr_cx, tr_cy), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), lineType=cv2.LINE_AA)
                                break
                    except Exception as err:
                        pass
        ##                print(err)
                    
            if triangle_detected:
                for contour in contours:
                    approx = cv2.approxPolyDP(contour, 0.01 * cv2.arcLength(contour, True), True)
                    if len(approx) == 4 and not square_detected:
                        if not cv2.isContourConvex(approx):
                            continue
                        try:
                            M = cv2.moments(approx)
                            sq_cx = int(M['m10']/M['m00'])
                            sq_cy = int(M['m01']/M['m00'])
                            sq_area = int(cv2.contourArea(approx))
                            if sq_area > MINIMUM_AREA and sq_area < tr_area + tr_area / 5:  # avoid full image contour detected as square
                                (x, y, w, h) = cv2.boundingRect(approx)
                                ar = w / float(h)
                                if 0.9 < ar < 1.1:
                                    square_detected = True
                                cv2.drawContours(image, [approx], 0, (0, 0, 255), )
                                cv2.putText(image, str(ar), (sq_cx, sq_cy), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), lineType=cv2.LINE_AA)
        ##                        cv2.putText(image, str(sq_area), (sq_cx, sq_cy), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), lineType=cv2.LINE_AA)
                                break
                        except Exception as err:
                            pass
        ##                    print(err)

            if triangle_detected and square_detected:
                steering = 0  # center
                cv2.line(image, (tr_cx, tr_cy), (sq_cx, sq_cy), (255, 0, 0), 5)
                steering = tr_cx - sq_cx
        ##        cv2.putText(image, str(steering), (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), lineType=cv2.LINE_AA)
                try:
                    angle = int(math.atan((tr_cy - sq_cy) / (tr_cx - sq_cx)) * 100)
                except Exception as err:
        ##            print(err)
                    if tr_cy < sq_cy:
                        angle = -155
                    else:
                        angle = 155
                
                raw_steering = abs(angle // 3)
                if tr_cy > sq_cy:  # triangle down
                    if raw_steering >= 40:
                        direction = 'BKWD'
                    else:
                        if tr_cx > sq_cx:  # triangle right
                            if raw_steering <=10:
                                direction = 'FWD RIGHT'
                            else:
                                direction = 'BKWD RIGHT'
                        else:  # triangle left
                            if raw_steering <=10:
                                direction = 'FWD LEFT'
                            else:
                                direction = 'BKWD LEFT'
                else:  # triangle up
                    if raw_steering >= 40:
                        direction = 'FWD'
                    else:
                        if tr_cx > sq_cx:  # triangle right
                            direction = 'FWD RIGHT'
                        else:  # triangle left
                            direction = 'FWD LEFT'
    ##            cv2.putText(image, '{} {} {}'.format(angle, raw_steering, direction), (0, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), lineType=cv2.LINE_AA)

            else:
                for contour in contours:
                    approx = cv2.approxPolyDP(contour, 0.01 * cv2.arcLength(contour, True), True)

                    if len(approx) == 8 and not stop_detected:
                        if not cv2.isContourConvex(approx):
                            continue
                        try:
                                                    
                            M = cv2.moments(approx)
                            stop_cx = int(M['m10']/M['m00'])
                            stop_cy = int(M['m01']/M['m00'])
                            stop_area = int(cv2.contourArea(approx))
                            if stop_area > 2 * MINIMUM_AREA:
                                (x, y, w, h) = cv2.boundingRect(approx)
                                ar = w / float(h)
                                if 0.9 < ar < 1.1:
                                    stop_detected = True
                                    cv2.drawContours(image, [approx], 0, (0, 255, 255), )
                ##                    cv2.putText(image, str(stop_area), (stop_cx, stop_cy), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), lineType=cv2.LINE_AA)
                                    direction = 'STOP'
                                    break
                        except Exception as err:
                            pass
            ##                print(err)
            
                    
    ##            cv2.putText(image, '{}', (50, 150), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), lineType=cv2.LINE_AA)
            
    ##        cv2.putText(image, '{}'.format(direction), (0, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), lineType=cv2.LINE_AA)

            if direction == 'NONE':
                none_counter += 1
                if none_counter > NONE_COMMAND_COUNTER_THRESHOLD:
                    print('NONE counter max reached!')
                    direction = 'STOP'
            else:
                none_counter = 0

            if direction == 'FWD':
                fwd(CRUISE_SPEED)
                center_steering()
                time.sleep(COMMAND_DELAY)
                cv2.imwrite(session_out_dir  + r'/FWD' + str(auto_cnt) + ".jpg", image)
            elif direction == 'FWD LEFT':
                fwd(CRUISE_SPEED)
                steer_max_left()
                time.sleep(COMMAND_DELAY)
                cv2.imwrite(session_out_dir  + r'/FWDL' + str(auto_cnt) + ".jpg", image)
            elif direction == 'FWD RIGHT':
                fwd(CRUISE_SPEED)
                steer_max_right()
                time.sleep(COMMAND_DELAY)
                cv2.imwrite(session_out_dir  + r'/FWDR' + str(auto_cnt) + ".jpg", image)
            elif direction == 'BKWD':
                bkwd(CRUISE_SPEED)
                center_steering()
                time.sleep(COMMAND_DELAY)
                cv2.imwrite(session_out_dir  + r'/BKWD' + str(auto_cnt) + ".jpg", image)
            elif direction == 'BKWD LEFT':
                bkwd(CRUISE_SPEED)
                steer_max_left()
                time.sleep(COMMAND_DELAY)
                cv2.imwrite(session_out_dir  + r'/BKWDL' + str(auto_cnt) + ".jpg", image)
            elif direction == 'BKWD RIGHT':
                bkwd(CRUISE_SPEED)
                steer_max_right()
                time.sleep(COMMAND_DELAY)
                cv2.imwrite(session_out_dir  + r'/BKWDR' + str(auto_cnt) + ".jpg", image)
            elif direction == 'STOP':
                print('DIRECTION STOP')
                stop()
                stop()
                stop()
                if none_counter < NONE_COMMAND_COUNTER_THRESHOLD:
                    cv2.imwrite(session_out_dir  + r'/STOP' + str(auto_cnt) + ".jpg", image)
                none_counter = 0
            elif direction == 'NONE':
                pass
            else:
                stop()
                stop()
                stop()
                print('WARNING!!! Unknown direction: {}'.format(direction))
            
            # show the frame
            cv2.imshow("Original frame", image)
        ##    cv2.imshow("Grey frame", grey_image)
            cv2.imshow("Binary frame", binary_image)
            
            key = cv2.waitKey(1) & 0xFF

            # clear the stream in preparation for the next frame
            raw_capture.truncate(0)

            # if the `q` key was pressed, break from the loop
            if key == ord("q") or stop_auto:
                stop()
                stop()
                stop()
                GPIO.cleanup()
                cv2.destroyAllWindows()
                camera.close()
                print('Stopped recording!')        
                with canvas(device) as draw:
                    draw.text((0, 0), "System stopped!", fill="white")
                break
    except KeyboardInterrupt:
        print('Stopped by user!')
        stop_auto = True
        pass
    
    except Exception as err:
        print(err)
        stop()
        stop()
        stop()
        GPIO.cleanup()
        time.sleep(.5)
        cv2.destroyAllWindows()
        with canvas(device) as draw:
            draw.text((0, 0), "System stopped!", fill="white")

