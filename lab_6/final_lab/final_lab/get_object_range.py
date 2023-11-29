#!/usr/bin/env python
# Team Aaron, Aryan

import sys

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class RangeDetector(Node):
    def __init__(self):
        # Creates the node.
        super().__init__('range_detector_node')
        
        self.last_object_angle = 0
        self.last_object_dist = 0
        
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
        
        self.publisher = self.create_publisher(String, 'distance', 10)
                
    def _lidar_callback(self, laserScan):
        dst_values =  laserScan.ranges
        angle_increment_per_index = laserScan.angle_increment
        
        idx = 0
        
        min_idx = None
        
        if 0 <= idx < len(dst_values):
            total = []
            for idx in range(idx - 10, idx + 10, 1):
                if idx >= len(dst_values):
                    idx = idx % len(dst_values)
                
                if not np.isnan(dst_values[idx]):
                    total.append(dst_values[idx])
            
            total = np.array(total)
            
            if len(total) != 0:
                dst = min(total)
             
                
                msg = String()
                msg.data = f"{dst}" 
                
                self.publisher.publish(msg)
                self.get_logger().info(f"Distance to Object and angle {dst}")
            
                
        
    
def main():
    rclpy.init()
    range_detector = RangeDetector()
    
    try:
        rclpy.spin(range_detector)
    except SystemExit:
        rclpy.logging.get_logger("Range detector node...").info("Shutting Down")
    
    range_detector.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
    
    
