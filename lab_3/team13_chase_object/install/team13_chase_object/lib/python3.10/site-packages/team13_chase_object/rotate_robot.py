# Team - Aaron Zhao, Aryan Sarswat

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3

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
        
        threshold = 20
        
        magnitude = abs(x - self.center[0])
        scaled_magnitude = magnitude / self.resolution[0]
        if x > self.center[0] and magnitude > threshold:
            self.rotate_right(scaled_magnitude)
        elif x < self.center[0] and magnitude > threshold:
            self.rotate_left(scaled_magnitude)
        else:
            self._vel_publisher.publish(Twist(linear=Vector3(x=0.0), angular=Vector3(z=0.0)))
        
        self._user_input =  cv2.waitKey(1)
        if self._user_input == ord('q'):
            self._vel_publisher.publish(Twist(linear=Vector3(x=0.0), angular=Vector3(z=0.0)))
            raise SystemExit
        
    def rotate_left(self, magnitude):
        magnitude = float(magnitude)
        self._vel_publisher.publish(Twist(linear=Vector3(x=0.0), angular=Vector3(z=magnitude)))
        self.cmd_publisher.publish(String(data='rotate_left'))
    
    def rotate_right(self, magnitude):
        magnitude = float(magnitude)
        self._vel_publisher.publish(Twist(linear=Vector3(x=0.0), angular=Vector3(z=-magnitude)))
        self.cmd_publisher.publish(String(data='rotate_right'))
        
def main():
    rclpy.init()
    rotate_robot = Rotate_Robot()
    
    try:
        rclpy.spin(rotate_robot)
    except SystemExit:
        rclpy.logging.get_logger("Rotate Object node...").info("Shutting Down")
    
    rotate_robot.destroy_node()
    rclpy.shutdown()