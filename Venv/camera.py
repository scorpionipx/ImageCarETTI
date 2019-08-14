import time
import cv2
import math

import numpy as np

from picamera.array import PiRGBArray
from picamera import PiCamera

WIDTH = 640
HEIGHT = 480

BINARY_THRESHOLD = 95
MINIMUM_AREA = 800

DECISION_NONE = 0
DECISION_FWD = 1
DECISION_FWD_LEFT = 2
DECISION_FWD_RIGHT = 3
DECISION_BKWD = 4
DECISION_BKWD_LEFT = 5
DECISION_BKWD_RIGHT = 6
DECISION_STOP = 7

DECISION_FILTER_RANK = 10
RAW_DECISIONS = [DECISION_NONE] * DECISION_FILTER_RANK
DECISION_RANKS = [0, 0, 0, 0, 0, 0, 0, 0, ]

camera = PiCamera()
camera.resolution = (WIDTH, HEIGHT)
camera.framerate = 20
camera.color_effects = (128, 128)  # grayscale
raw_capture = PiRGBArray(camera, size=(WIDTH, HEIGHT))


def start_camera():
    print('Warming up PICamera...')
    time.sleep(0.1)

    print('Starting recording...')


    for frame in camera.capture_continuous(raw_capture, format="bgr", use_video_port=True):
        
        RAW_DECISIONS.pop(0)  # remove first element of list
        
        square_detected = False
        triangle_detected = False
        stop_detected = False
        msg = 'None'
        direction = 'NONE'
        
        image = frame.array
        grey_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
##        ret, binary_image = cv2.threshold(grey_image, BINARY_THRESHOLD, 255, cv2.THRESH_BINARY)
##        blur = cv2.GaussianBlur(image, (5, 5), 0)
##        ret3, binary_image = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        binary_image = cv2.adaptiveThreshold(image, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)


        _, contours, _ = cv2.findContours(binary_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
##        circles = cv2.HoughCircles(binary_image, cv2.HOUGH_GRADIENT, 1.2, 100)
##
##        if circles is not None:
##	# convert the (x, y) coordinates and radius of the circles to integers
##            circles = np.round(circles[0, :]).astype("int")
##
##            # loop over the (x, y) coordinates and radius of the circles
##            for (x, y, r) in circles:
##            # draw the circle in the output image, then draw a rectangle
##            # corresponding to the center of the circle
##                cv2.circle(image, (x, y), r, (0, 255, 0), 4)
##                cv2.rectangle(image, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
		
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
                    RAW_DECISIONS.append(DECISION_BKWD)
                else:
                    if tr_cx > sq_cx:  # triangle right
                        if raw_steering <=10:
                            direction = 'FWD RIGHT'
                            RAW_DECISIONS.append(DECISION_FWD_RIGHT)
                        else:
                            direction = 'BKWD RIGHT'
                            RAW_DECISIONS.append(DECISION_BKWD_RIGHT)
                    else:  # triangle left
                        if raw_steering <=10:
                            direction = 'FWD LEFT'
                            RAW_DECISIONS.append(DECISION_FWD_LEFT)
                        else:
                            direction = 'BKWD LEFT'
                            RAW_DECISIONS.append(DECISION_BKWD_LEFT)
            else:  # triangle up
                if raw_steering >= 40:
                    direction = 'FWD'
                    RAW_DECISIONS.append(DECISION_FWD)
                else:
                    if tr_cx > sq_cx:  # triangle right
                        direction = 'FWD RIGHT'
                        RAW_DECISIONS.append(DECISION_FWD_RIGHT)
                    else:  # triangle left
                        direction = 'FWD LEFT'
                        RAW_DECISIONS.append(DECISION_FWD_LEFT)
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
                        if stop_area > MINIMUM_AREA:
                            (x, y, w, h) = cv2.boundingRect(approx)
                            ar = w / float(h)
                            if 0.9 < ar < 1.1:
                                stop_detected = True
                                cv2.drawContours(image, [approx], 0, (0, 255, 255), )
    ##                            cv2.putText(image, 'AR: {}'.format(ar), (50, 250), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), lineType=cv2.LINE_AA)
                                
            ##                    cv2.putText(image, str(stop_area), (stop_cx, stop_cy), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), lineType=cv2.LINE_AA)
                                msg = 'STOP'
                                RAW_DECISIONS.append(DECISION_STOP)
                                break
                    except Exception as err:
                        pass
        ##                print(err)
        
                
##            cv2.putText(image, '{}', (50, 150), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), lineType=cv2.LINE_AA)

        if direction == 'NONE':
            RAW_DECISIONS.append(DECISION_NONE)

        DECISION_RANKS[DECISION_NONE] = RAW_DECISIONS.count(RAW_DECISIONS)
        DECISION_RANKS[DECISION_FWD] = RAW_DECISIONS.count(DECISION_FWD)
        DECISION_RANKS[DECISION_FWD_LEFT] = RAW_DECISIONS.count(DECISION_FWD_LEFT)
        DECISION_RANKS[DECISION_FWD_RIGHT] = RAW_DECISIONS.count(DECISION_FWD_RIGHT)
        DECISION_RANKS[DECISION_BKWD] = RAW_DECISIONS.count(DECISION_BKWD)
        DECISION_RANKS[DECISION_BKWD_LEFT] = RAW_DECISIONS.count(DECISION_BKWD_LEFT)
        DECISION_RANKS[DECISION_BKWD_RIGHT] = RAW_DECISIONS.count(DECISION_BKWD_RIGHT)
        DECISION_RANKS[DECISION_STOP] = RAW_DECISIONS.count(DECISION_STOP)

        FILTERED_DECISION = DECISION_RANKS.index(max(DECISION_RANKS))
        
        cv2.putText(image, 'DIR: {} RD: {}'.format(direction, RAW_DECISIONS[-1]), (0, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), lineType=cv2.LINE_AA)
        cv2.putText(image, 'FD: {}'.format(FILTERED_DECISION), (0, 150), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), lineType=cv2.LINE_AA)
        # show the frame
        cv2.imshow("Original frame", image)
    ##    cv2.imshow("Grey frame", grey_image)
        cv2.imshow("Binary frame", binary_image)
        
        key = cv2.waitKey(1) & 0xFF
##        if stop_detected:
##            time.sleep(5)

        # clear the stream in preparation for the next frame
        raw_capture.truncate(0)

        # if the `q` key was pressed, break from the loop
        if key == ord("q"):
            cv2.destroyAllWindows()
            print('Stopped recording!')
            break


if __name__ == '__main__':
    start_camera()
    cv2.destroyAllWindows()
