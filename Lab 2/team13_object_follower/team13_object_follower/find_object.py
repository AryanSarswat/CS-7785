# Team - Aaron Zhao, Aryan Sarswat

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String

import sys

import numpy as np
import cv2
from cv_bridge import CvBridge
import imutils

class DetectObject(Node):
    def __init__(self):
        # Creates the node.
        super().__init__('detect_object_node')
        
        self.ballLower = (0, 150, 0)
        self.ballUpper = (15, 255, 255)
        
        self.declare_parameter('show_image_bool', True)
        self.declare_parameter('window_name', "Raw Image")
        
        #Determine Window Showing Based on Input
        self._display_image = bool(self.get_parameter('show_image_bool').value)
        
        # Declare some variables
        self._titleOriginal = self.get_parameter('window_name').value # Image Window Title
        if(self._display_image):
            # Set Up Image Viewing
            cv2.namedWindow(self._titleOriginal, cv2.WINDOW_AUTOSIZE )
            cv2.moveWindow(self._titleOriginal, 50, 50)
        
        #Set up QoS Profiles for passing images over WiFi
        image_qos_profile = QoSProfile(depth=5)
        image_qos_profile.history = QoSHistoryPolicy.KEEP_LAST
        image_qos_profile.durability = QoSDurabilityPolicy.VOLATILE
        image_qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        
        #Declare that the minimal_video_subscriber node is subcribing to the /camera/image/compressed topic.
        self._video_subscriber = self.create_subscription(
                CompressedImage,
                '/image_raw/compressed',
                self._image_callback,
                image_qos_profile)
        self._video_subscriber # Prevents unused variable warning.
        
        # Create Publisher
        self.publisher = self.create_publisher(String, 'object_location', 10)
    
    def _image_callback(self, CompressedImage):
        # The "CompressedImage" is transformed to a color image in BGR space and is store in "_imgBGR"
        self._imgBGR = CvBridge().compressed_imgmsg_to_cv2(CompressedImage, "bgr8")
        if(self._display_image):
            # Display the image in a window
            input_frame = self._imgBGR.copy()
            
            center, radius, pre_processed_frame = self.detect_object(input_frame)
            
            if center is not None:
                cv2.circle(input_frame, 
                    center=(int(center[0]), int(center[1])), 
                    radius=int(radius), 
                    color=(0, 255, 0), 
                    thickness=2)
                
                msg = String()
                msg.data = str(center[0]) + "," + str(center[1])
                self.publisher.publish(msg)
                self.get_logger().info('Publishing: "%s"' % msg.data)
        
            self.show_image(input_frame)
            
    
    def get_image(self):
        return self._imgBGR
    
    def get_user_input(self):
        return self._user_input
    
    def show_image(self, img):
        cv2.imshow(self._titleOriginal, img)
        
        self._user_input = cv2.waitKey(1)
        if self._user_input == ord('q'):
            cv2.destroyAllWindows()
            raise SystemExit
        
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


def main():
    rclpy.init()
    object_detector = DetectObject()
    
    try:
        rclpy.spin(object_detector)
    except SystemExit:
        rclpy.logging.get_logger("Object Detector Node Info...").info("Shutting Down")
    
    detect_object.destroy_node()
    rclpy.shutdown()
    
    
if __name__ == '__main__':
    main()