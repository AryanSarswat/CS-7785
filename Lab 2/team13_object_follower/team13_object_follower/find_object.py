# Team - Aaron Zhao, Aryan Sarswat


import numpy as np
import cv2
from collections import deque
import argparse
import time
import imutils

class DetectObject:
    def __init__(self):
        self.ballLower = (0, 150, 0)
        self.ballUpper = (15, 255, 255)
        self.video_cap = cv2.VideoCapture(0)
        
        self.frame_height = int(self.video_cap.get(4))
        self.frame_width = int(self.video_cap.get(3))
        self.video_writer = cv2.VideoWriter('outpy2.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 10, (self.frame_width,self.frame_height))
    
    def pre_process(self, input_frame):
        frame = input_frame.copy()

        # Median blur to remove noise
        blurred = cv2.medianBlur(frame, 3)
        
        # Convert to HSV
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        
        # Apply Color mask mask
        mask = cv2.inRange(hsv, self.ballLower, self.ballUpper)

        # Open and close morphological operations
        open_struct = cv2.getStructuringElement(cv2.MORPH_RECT,(5,5))
        close_struct = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))
        
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_struct)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, open_struct)
        
        # Gaussian blur to smooth edges
        mask = cv2.GaussianBlur(mask, (3, 3), 10, 2)

        #cv2.imshow('mask', mask)
        
        return mask
    
    def get_contours(self, pre_processed_frame):
        cnts = cv2.findContours(pre_processed_frame.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        
        center = None
        radius = 0
        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosisng circle and
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
        
        return [center, int(radius)]
    
    def detect_object(self, input_frame):
        pre_processed_frame = self.pre_process(input_frame)
        center, radius = self.get_contours(pre_processed_frame)
        return center, radius, pre_processed_frame
    
    def start(self):
        
        while True:
            ret, input_frame = self.video_cap.read()
            
            center, radius, pre_processed_frame = self.detect_object(input_frame)
            
            if center is not None:
                cv2.circle(input_frame, 
                    center=(int(center[0]), int(center[1])), 
                    radius=int(radius), 
                    color=(0, 255, 0), 
                    thickness=2)
                
            cv2.imshow('frame', input_frame)
            self.video_writer.write(input_frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        self.video_writer.release()
        self.video_cap.release()

if __name__ == '__main__':
    object_detector = DetectObject()
    object_detector.start()
    cv2.destroyAllWindows()
