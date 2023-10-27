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
        angle_increment_per_index = laserScan.angle_increment
        dst_values =  laserScan.ranges
        
        idx = 0
        
        min_idx = None
        
        if 0 <= idx < len(dst_values):
            total = []
            for idx in range(idx - 5, idx + 5, 1):
                if idx >= len(dst_values):
                    idx = idx % len(dst_values)
                
                if not np.isnan(dst_values[idx]):
                    total.append(dst_values[idx])
                else:
                    dst_values[idx] = dst_values[idx - 1]
            
            total = np.array(total)
            
            if len(total) != 0:
                dst = total.min()
                min_idx = total.argmin()
                corner_angle = None
                angle = 0
             
                if dst <= 0.5:
                    left_idx = min_idx
                    right_idx = min_idx  
                    corner_idx = None
                    displace = ((5/180) * np.pi) / angle_increment_per_index
                    while corner_idx == None:
                        n_left_idx = (left_idx - 1) % len(dst_values)
                        n_right_idx = (right_idx + 1) % len(dst_values)
                        
                        if abs(dst_values[left_idx] - dst_values[n_left_idx]) > 0.5:
                            # Displace the corner by 5 degrees
                            corner_idx = n_left_idx - displace
                            break
                        if abs(dst_values[right_idx] - dst_values[n_right_idx]) > 0.5:
                            corner_idx = n_right_idx + displace
                            break
                        
                        left_idx = n_left_idx
                        right_idx = n_right_idx

                    angle = corner_idx * angle_increment_per_index
                    
                msg = String()
                msg.data = f"{dst},{angle}" 
                
                self.publisher.publish(msg)
                self.get_logger().info(f"Distance to Object and angle {dst},{angle}")
            
                
        
    
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
    
    
