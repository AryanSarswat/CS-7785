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
        
        #Set up QoS Profiles for LIDAR Data
        lidar_qos_profile = QoSProfile(depth=5)
        lidar_qos_profile.history = QoSHistoryPolicy.KEEP_LAST
        lidar_qos_profile.durability = QoSDurabilityPolicy.VOLATILE
        lidar_qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        
        self.lidar_subscriber = self.create_subscription(
                LaserScan,
                '/scan',
                self._lidar_callback,
                lidar_qos_profile)
        self.lidar_subscriber # Prevents unused variable warning
        
        self.angle_subscriber = self.create_subscription(
                String,
                'object_angle_offset',
                self._angle_callback,
                10)
        
        self.publisher = self.create_publisher(String, 'distance_angle', 10)
                
    def _lidar_callback(self, LaserScan):
        # Preprocess LIDAR Data
        pass
    
    def _angle_callback(self, String):
        # Preprocess Angle Data
        pass
    
    
    def get_distance(self, LaserScan):
        pass
    
    