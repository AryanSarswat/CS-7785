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

class Rotate_Robot(Node):
    def __init__(self):
        # Creates the node.
        super().__init__('rotate_robot_node')
        
        # Setup subscriber
        self._message_subscriber = self.create_subscription(
                String,
                'object_location',
                self._rotate_object_callback,
                10)
        self._message_subscriber
        
        # Setup publishers
        self._vel_publisher = self.create_publisher(Twist, '/cmd_vel', 5)
        self.cmd_publisher = self.create_publisher(String, 'rotation_command', 10)
        
        self.resolution = (340, 240)
        self.center = (self.resolution[0]//2, self.resolution[1]//2)
        
    def _rotate_object_callback(self, msg):
        coord = msg.data
        
        x, y = coord.split(',')
        x, y = int(x), int(y)
        
        magnitude = abs(x - self.center[0])
        if x > self.center[0]:
            self.rotate_right(magnitude)
        elif x < self.center[0]:
            self.rotate_left(magnitude)
        else:
            pass
        
        self._user_input =  cv2.waitKey(1)
        if self._user_input == ord('q'):
            raise SystemExit
        
    def rotate_left(self, magnitude):
        if magnitude > 100:
            angle = 0.5
        elif magnitude > 50:
            angle = 0.25
        else:
            angle = 0.1
        self._vel_publisher.publish(Twist(linear=Vector3(x=0.0), angular=Vector3(z=angle)))
        self.cmd_publisher.publish(String(data='rotate_left'))
    
    def rotate_right(self, magnitude):
        if magnitude > 100:
            angle = 0.5
        elif magnitude > 50:
            angle = 0.25
        else:
            angle = 0.1
        self._vel_publisher.publish(Twist(linear=Vector3(x=0.0), angular=Vector3(z=-angle)))
        self.cmd_publisher.publish(String(data='rotate_right'))
        
def main():
    rclpy.init()
    rotate_robot = Rotate_Robot()
    
    try:
        rclpy.spin(object_detector)
    except SystemExit:
        rclpy.logging.get_logger("Rotate Object node...").info("Shutting Down")
    
    rotate_robot.destroy_node()
    rclpy.shutdown()