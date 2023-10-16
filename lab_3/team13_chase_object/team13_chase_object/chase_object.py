#!/usr/bin/env python
# Team Aaron, Aryan

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3


import sys

import numpy as np
import cv2
from cv_bridge import CvBridge

class ChaseObject(Node):
    def __init__(self):
        # Creates the node.
        super().__init__('chase_object_node')
        
        # Setup PD Control
        self.P_K_V = 0.4
        self.P_K_A = 1.5
        self.D_K_V = 0.4
        self.D_K_A = 1.5
                
        #Set up QoS Profiles for LIDAR Data
        lidar_qos_profile = QoSProfile(depth=5)
        lidar_qos_profile.history = QoSHistoryPolicy.KEEP_LAST
        lidar_qos_profile.durability = QoSDurabilityPolicy.VOLATILE
        lidar_qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        
        self.range_angle_subscriber = self.create_subscription(
                String,
                'distance_angle',
                self._actuate,
                10)
        
        # Setup publishers
        self._vel_publisher = self.create_publisher(Twist, '/cmd_vel', 5)
        
        # self.prev_vel = 0
        self.prev_ang = 0
        self.prev_dist = 0
                
    def _actuate(self, String):
        distance, angle = String.data.split(',')
        distance = float(distance) - 0.55
        angle = float(angle) / np.pi
        dt = 1
        
        angular_vel, linear_vel = 0, 0
        if self.prev_ang:
            angular_vel = (angle - self.prev_ang)/dt
        if self.prev_dist:
            linear_vel = (distance - self.prev_dist)/dt
                
        # Proportional Control
        u_angle = float(angle) * self.P_K_A
        u_dist = distance* self.P_K_V
        
        # Derivative Control
        u_angular_vel = angular_vel * self.D_K_A
        u_linear_vel = linear_vel * self.D_K_V
        
        # Control terms
        u_angular = u_angle + u_angular_vel
        u_linear = u_dist + u_linear_vel
        
        # Clamping
        u_angular = np.clip(u_angular, -1.84, 1.84)
        u_linear = np.clip(u_linear, -0.12,0.12)
        
        # Stopping thershold
        epsilon_v = 0.1
        if -epsilon_v <= distance < epsilon_v:
            u_linear = 0.0
        
        epsilon_a = 0.005
        if -epsilon_a <= angle <= epsilon_a:
            u_angular = 0.0
        
        # Update Previous Values
        self.prev_ang = angle
        self.prev_dist = distance
        
        # Publish Twist Message
        self._vel_publisher.publish(Twist(linear=Vector3(x=u_linear), angular=Vector3(z=u_angular)))
        self.get_logger().info(f"Publishing l_v : {u_linear} and a_v : {u_angular}")
    
    
def main():
    rclpy.init()
    chaser = ChaseObject()
    
    try:
        rclpy.spin(chaser)
    except SystemExit:
        rclpy.logging.get_logger("Chaser node...").info("Shutting Down")
    
    chaser.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
