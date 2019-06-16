import cv2
import serial
import sys
import time
import math

import numpy as np

from picamera.array import PiRGBArray
from picamera import PiCamera


MILISECOND = 0.001
SECOND = 1


CRUISE_SPEED = 35

__version__ = '0.0.1'
print('EETI car v{}'.format(__version__))


WIDTH = 640
HEIGHT = 480

BINARY_THRESHOLD = 95
MINIMUM_AREA = 800

ser = None
camera = None
raw_capture = None


def init_camera():
    global camera
    global raw_capture
    try:
        print('Initiating camera...')
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


def auto():
    global camera
    global raw_capture
    global ser
    if not init_serial():
        return

    if not init_camera():
        return

    print('Starting recording...')
    none_counter = 0

    for frame in camera.capture_continuous(raw_capture, format="bgr", use_video_port=True):

        square_detected = False
        triangle_detected = False
        stop_detected = False
        direction = 'NONE'
        
        image = frame.array
        grey_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        ret, binary_image = cv2.threshold(grey_image, BINARY_THRESHOLD, 255, cv2.THRESH_BINARY)

        _, contours, _ = cv2.findContours(binary_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            approx = cv2.approxPolyDP(contour, 0.01 * cv2.arcLength(contour, True), True)

            if len(approx) == 3 and not triangle_detected:
                try:
                    M = cv2.moments(approx)
                    tr_cx = int(M['m10']/M['m00'])
                    tr_cy = int(M['m01']/M['m00'])
                    tr_area = int(cv2.contourArea(approx))
                    if tr_area > MINIMUM_AREA:
                        triangle_detected = True
                        cv2.drawContours(image, [approx], 0, (0, 255, 0), )
    ##                    cv2.putText(image, str(tr_area), (tr_cx, tr_cy), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), lineType=cv2.LINE_AA)
                        break
                except Exception as err:
                    pass
    ##                print(err)
                
        if triangle_detected:
            for contour in contours:
                approx = cv2.approxPolyDP(contour, 0.01 * cv2.arcLength(contour, True), True)
                if len(approx) == 4 and not square_detected:
                    try:
                        M = cv2.moments(approx)
                        sq_cx = int(M['m10']/M['m00'])
                        sq_cy = int(M['m01']/M['m00'])
                        sq_area = int(cv2.contourArea(approx))
                        if sq_area > MINIMUM_AREA and sq_area < tr_area + tr_area / 5:  # avoid full image contour detected as square
                            square_detected = True
                            cv2.drawContours(image, [approx], 0, (0, 0, 255), )
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

                if len(approx) == 6 and not stop_detected:
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
        
        cv2.putText(image, '{}'.format(direction), (0, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), lineType=cv2.LINE_AA)

        if direction == 'NONE':
            none_counter += 1
            if none_counter > 1000:
                print('NONE counter max reached!')
                direction = 'STOP'
        else:
            none_counter = 0

        if direction == 'FWD':
            fwd(CRUISE_SPEED)
            time.sleep(1)
        elif direction == 'FWD_LEFT':
            fwd(CRUISE_SPEED)
            time.sleep(1)
        elif direction == 'FWD_RIGHT':
            fwd(CRUISE_SPEED)
            time.sleep(1)
        elif direction == 'BKWD':
            bkwd(CRUISE_SPEED)
            time.sleep(1)
        elif direction == 'BKWD_LEFT':
            bkwd(CRUISE_SPEED)
            time.sleep(1)
        elif direction == 'BKWD_RIGHT':
            bkwd(CRUISE_SPEED)
            time.sleep(1)
        elif direction == 'STOP':
            print('DIRECTION STOP')
            stop()
            stop()
            stop()
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
        if key == ord("q"):
            stop()
            stop()
            cv2.destroyAllWindows()
            print('Stopped recording!')
            break

