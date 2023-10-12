#!/usr/bin/env python
# Team Aaron, Aryan

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String

import sys

import numpy as np
import cv2
from cv_bridge import CvBridge

class ChaseObject(Node):
    def __init__(self):
        # Creates the node.
        super().__init__('chase_object_node')
        
        # Setup PD Control
        self.P_K = 0.5
        self.D_K = 0.5
        
        #Set up QoS Profiles for LIDAR Data
        lidar_qos_profile = QoSProfile(depth=5)
        lidar_qos_profile.history = QoSHistoryPolicy.KEEP_LAST
        lidar_qos_profile.durability = QoSDurabilityPolicy.VOLATILE
        lidar_qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        
        
        
        self.range_angle_subscriber = self.create_subscription(
                String,
                'distance_angle',
                self.actuate,
                10)
        
        # Setup publishers
        self._vel_publisher = self.create_publisher(Twist, '/cmd_vel', 5)
        
        self.prev_vel = 0
        self.prev_ang = 0
                
    def _actuate(self, String):
        distance, angle = String.data.split(',')
        
        # Proportional Control
        angle = float(angle) * self.P_K
        speed = float(distance) * self.P_K
        
        # Derivative Control
        angle = (angle - self.prev_ang) * self.D_K
        speed = (speed - self.prev_vel) * self.D_K
        
        # Update Previous Values
        self.prev_vel = speed
        self.prev_ang = angle
        
        # Publish Twist Message
        self._vel_publisher.publish(Twist(linear=Vector3(x=speed), angular=Vector3(z=angle)))
    