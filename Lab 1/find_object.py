import numpy as np
import cv2
from collections import deque
import argparse
import time
import imutils

ballLower = (0, 150, 0)
ballUpper = (10, 255, 255)
def get_circles(input_frame):
    
    frame = input_frame.copy()

    # frame[0:int(len(frame)//3)] = 0

    blurred = cv2.medianBlur(frame, 5)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    
    mask = cv2.inRange(hsv, ballLower, ballUpper)

    open_struct = cv2.getStructuringElement(cv2.MORPH_RECT,(19,19))
    close_struct = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(15,15))
    
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_struct)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, open_struct)
    
    mask = cv2.GaussianBlur(mask, (15, 15), 2, 2)

    circles = cv2.HoughCircles(mask, 
        cv2.HOUGH_GRADIENT, 
        1, 
        mask.shape[0] / 4, 
        param1=40, 
        param2=20, 
        minRadius=10, 
        maxRadius=0)
    
    return circles, mask

def get_circles_countours(input_frame):
    
    frame = input_frame.copy()

    # frame[0:int(len(frame)//3)] = 0

    blurred = cv2.medianBlur(frame, 5)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    
    mask = cv2.inRange(hsv, ballLower, ballUpper)

    open_struct = cv2.getStructuringElement(cv2.MORPH_RECT,(19,19))
    close_struct = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(15,15))
    
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_struct)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, open_struct)
    
    mask = cv2.GaussianBlur(mask, (15, 15), 2, 2)

    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
	
    center = None
    radius = 0
	# only proceed if at least one contour was found
    if len(cnts) > 0:
		# find the largest contour in the mask, then use
		# it to compute the minimum enclosing circle and
		# centroid
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
		# only proceed if the radius meets a minimum size
        if radius > 10:
			# draw the circle and centroid on the frame,
			# then update the list of tracked points
            cv2.circle(frame, (int(x), int(y)), int(radius),
				(0, 255, 255), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)
    
    return [center, int(radius)], mask

vs = cv2.VideoCapture(0)

frame_width = int(vs.get(3))
frame_height = int(vs.get(4))

out = cv2.VideoWriter('outpy2.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 10, (frame_width,frame_height))

# Start a while loop
while(1):

    # Reading the video from the
    # webcam in image frames
    ret, input_frame = vs.read()
    
    circles, mask = get_circles(input_frame)
    center, mask = get_circles_countours(input_frame)
    
    if circles is not None and center[0] is not None:
        
        circles = circles[0]
        center_circles = [circles[0, 0], circles[0, 1], circles[0, 2]]
        center_contours = [center[0][0], center[0][1], center[1]]
        centers = np.array([center_circles, center_contours])
        center = np.average(centers, axis=0)
        print(center)
        cv2.circle(input_frame, 
            center=(int(center[0]), int(center[1])), 
            radius=int(center[2]), 
            color=(0, 255, 0), 
            thickness=2)
        
    # if circles[0] is not None:
    #     cv2.circle(input_frame, 
    #         center=(circles[0][0], circles[0][1]), 
    #         radius=circles[1], 
    #         color=(0, 255, 0), 
    #         thickness=2)

    # Display the resulting frame, quit with q
    cv2.imshow('frame', input_frame)
    # cv2.imshow('mask', mask)
    # cv2.imwrite("color_mask.jpg", mask)
    out.write(input_frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


out.release()
vs.release()
