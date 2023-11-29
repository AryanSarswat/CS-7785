#!/usr/bin/env python
import sys

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Point, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from std_msgs.msg import String


class Controller(Node):
    def __init__(self):
        super().__init__('controller_node')
        
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Init Odom to (0,0,0) and initializep position variables
        self.Init = True
        self.Init_pos = Point()
        self.Init_pos.x = 0.0
        self.Init_pos.y = 0.0
        self.Init_ang = 0.0
        self.globalPos = Point()
        self.globalAng = 0.0
        self.targetGlobalAng = 0.0
        
        self.pos_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            1)
        self.pos_sub # prevent unsued variable warning
        
        self.distance_to_object = float('inf')
        
        self.range_sub = self.create_subscription(
            String,
            '/distance',
            self.dist_callback,
            1)
        
        self.range_sub # prevent unsued variable warning
        
        self.sign_sub = self.create_subscription(
            String,
            '/sign',
            self.sign_callback,
            1)
        
        self.sign_sub # prevent unsued variable warning
        self.accumalate = False
        self.most_recent_signs = []
        
        self.before_action_pose = None
        self.TURNING_AROUND = False
        self.TURNING_LEFT = False
        self.TURNING_RIGHT = False
        self.TURNING = False
        
        self.ALLOWANCE = 0.1
        
    
    def odom_callback(self, data):
        self.update_Odometry(data)
        
    def update_Odometry(self, Odom):
        position = Odom.pose.pose.position
            
        #Orientation uses the quaternion aprametrization.
        #To get the angular position along the z-axis, the following equation is required.
        q = Odom.pose.pose.orientation
        orientation = np.arctan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

        if self.Init:
            #The initial data is stored to by subtracted to all the other values as we want to start at position (0,0) and orientation 0
            self.Init = False
            self.Init_ang = orientation
            self.globalAng = self.Init_ang
            Mrot = np.matrix([[np.cos(self.Init_ang), np.sin(self.Init_ang)],[-np.sin(self.Init_ang), np.cos(self.Init_ang)]])        
            self.Init_pos.x = Mrot.item((0,0))*position.x + Mrot.item((0,1))*position.y
            self.Init_pos.y = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y
            self.Init_pos.z = position.z
            
        Mrot = np.matrix([[np.cos(self.Init_ang), np.sin(self.Init_ang)],[-np.sin(self.Init_ang), np.cos(self.Init_ang)]])        

        #We subtract the initial values
        self.globalPos.x = Mrot.item((0,0))*position.x + Mrot.item((0,1))*position.y - self.Init_pos.x
        self.globalPos.y = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y - self.Init_pos.y
        self.globalAng = orientation - self.Init_ang


        self.get_logger().info('Transformed global pose is x:{}, y:{}, a:{}'.format(self.globalPos.x,self.globalPos.y,self.globalAng))
        self.goToGoal()
        
    def sign_callback(self, msg):
        if self. accumalate:
            self.most_recent_signs.append(int(msg.data))
    
    def goToGoal(self):
        # if self.TURNING_LEFT:
        #     self.turning_left_goal_check()
        # elif self.TURNING_RIGHT:
        #     self.turning_right_goal_check()
        # elif self.TURNING_AROUND:
        #     self.turning_around_goal_check()
        if self.TURNING:
            self.turn_goal_check()
        else:
            # Adjust this to be distance from wall to center of tile
            if self.distance_to_object <= 0.6:
                # Check sign and turn accordingly
                self.stop()
                self.accumalate = True
                if len(self.most_recent_signs) < 100:
                    self.get_logger().info("Waiting for classifier")
                    return
                
                sign_count_dict = {}
                for sign in self.most_recent_signs: 
                    if sign in sign_count_dict:
                        sign_count_dict[sign] += 1
                    else:
                        sign_count_dict[sign] = 1
                most_common_sign = max(sign_count_dict, key=sign_count_dict.get)
                
                # Reset
                self.most_recent_signs = []
                self.accumalate = False
                
                if most_common_sign == 0:
                    # Empty Wall
                    self.turn_around()
                elif most_common_sign == 1:
                    self.get_logger().info("Turning Left")
                    
                    # if self.globalAng > np.pi / 2:
                    #     self.targetGlobalAng = 0.0
                    #     self.Init = True
                    #     self.Init_pos = Point()
                    #     self.Init_pos.x = 0.0
                    #     self.Init_pos.y = 0.0
                    #     self.Init_ang = 0.0
                    #     self.globalPos = Point()
                    #     self.globalAng = 0.0
                    #     self.targetGlobalAng = 0.0
                    
                    # Turn Left
                    self.targetGlobalAng += np.pi/2
                    # while self.targetGlobalAng <= -np.pi:
                    #     self.targetGlobalAng += 2*np.pi
                    # while self.targetGlobalAng > np.pi:
                    #     self.targetGlobalAng -= 2*np.pi
                    self.TURNING = True
                    self.turn()
                    # self.turn_left()
                elif most_common_sign == 2:
                    self.get_logger().info("Turning Right")
                    
                    # if self.globalAng < -np.pi / 2:
                    #     self.targetGlobalAng = 0.0
                    #     self.Init = True
                    #     self.Init_pos = Point()
                    #     self.Init_pos.x = 0.0
                    #     self.Init_pos.y = 0.0
                    #     self.Init_ang = 0.0
                    #     self.globalPos = Point()
                    #     self.globalAng = 0.0
                    #     self.targetGlobalAng = 0.0
                    
                    # Turn Right
                    self.targetGlobalAng -= np.pi/2
                    # while self.targetGlobalAng <= -np.pi:
                    #     self.targetGlobalAng += 2*np.pi
                    # while self.targetGlobalAng > np.pi:
                    #     self.targetGlobalAng -= 2*np.pi
                    self.TURNING = True
                    self.turn()
                    # self.turn_right()
                elif most_common_sign == 3 or most_common_sign == 4:
                    # Do not enter (Turn Around)
                    self.targetGlobalAng -= np.pi
                    # while self.targetGlobalAng <= -np.pi:
                    #     self.targetGlobalAng += 2*np.pi
                    # while self.targetGlobalAng > np.pi:
                    #     self.targetGlobalAng -= 2*np.pi
                    self.TURNING = True
                    self.turn()
                    # self.turn_around()
                elif most_common_sign == 5:
                    # Goal State
                    self.stop()
                    raise Exception("Goal State Reached")
                else:
                    raise Exception("Invalid sign")
            else:
                # Go forward
                self.go_forward()
        
    def dist_callback(self, msg):
        dst =  msg.data
        dst = float(dst)
        self.distance_to_object = dst
    
    def turn(self):
        Kp = 0.1
        msg = Twist()
        msg.linear.x = 0.0
        target_angle_robot_frame = self.targetGlobalAng
        
        if self.targetGlobalAng > np.pi:
            target_angle_robot_frame = -(2*np.pi - self.targetGlobalAng)
        angle_robot_frame = self.globalAng
        if self.globalAng > np.pi:
            angle_robot_frame = -(2*np.pi - self.globalAng)
        elif self.globalAng < -np.pi:
            angle_robot_frame = (2*np.pi + self.globalAng)
        msg.angular.z = Kp * (target_angle_robot_frame - angle_robot_frame)
        self.vel_publisher.publish(msg)
    
    def turn_goal_check(self):
        target_angle_robot_frame = self.targetGlobalAng
        if self.targetGlobalAng > np.pi:
            target_angle_robot_frame = -(2*np.pi - self.targetGlobalAng)
        angle_robot_frame = self.globalAng
        if self.globalAng > np.pi:
            angle_robot_frame = -(2*np.pi - self.globalAng)
        elif self.globalAng < -np.pi:
            angle_robot_frame = (2*np.pi + self.globalAng)
        diff = np.abs(target_angle_robot_frame - angle_robot_frame)
        self.get_logger().info(f"Turning : diff = {diff} = {target_angle_robot_frame} - {angle_robot_frame}")
        if diff < self.ALLOWANCE:   
            self.TURNING = False
            self.vel_publisher.publish(Twist(linear=Vector3(x=0.0), angular=Vector3(z=0.0)))
            
            self.targetGlobalAng = 0.0
            self.Init = True
            self.Init_pos = Point()
            self.Init_pos.x = 0.0
            self.Init_pos.y = 0.0
            self.Init_ang = 0.0
            self.globalPos = Point()
            self.globalAng = 0.0
            self.targetGlobalAng = 0.0
    
    def turn_left(self):
        self.before_action_pose = self.globalAng
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.05
        self.vel_publisher.publish(msg)
        self.TURNING_LEFT = True
        
    def turning_left_goal_check(self):
        diff_init = abs(self.before_action_pose - self.globalAng)
        diff = min(diff_init, abs(2*np.pi - diff_init))
        
        self.get_logger().info(f"Turning left : diff = {diff}")
        
        if abs(diff) >= np.pi/2 - self.ALLOWANCE:
            self.TURNING_LEFT = False
            self.before_action_pose = None
            self.vel_publisher.publish(Twist(linear=Vector3(x=0.0), angular=Vector3(z=0.0)))
    
    def turn_right(self):
        self.before_action_pose = self.globalAng
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = -0.05
        self.vel_publisher.publish(msg)
        self.TURNING_RIGHT = True
        
    def turning_right_goal_check(self):
        diff_init = abs(self.before_action_pose - self.globalAng)
        diff = min(diff_init, abs(2*np.pi - diff_init))
        
        self.get_logger().info(f"Turning right : diff = {diff}")
         
        if abs(diff) >= np.pi/2 - self.ALLOWANCE:
            self.TURNING_RIGHT = False
            self.before_action_pose = None
            self.vel_publisher.publish(Twist(linear=Vector3(x=0.0), angular=Vector3(z=0.0)))
    
    def go_forward(self):
        msg = Twist()
        u_linear = 0.1 * self.distance_to_object
        u_linear = np.clip(u_linear, -0.1,0.1)
        
        msg.linear.x = u_linear
        
        Kp = 0.1
        target_angle_robot_frame = self.targetGlobalAng
        if self.targetGlobalAng > np.pi:
            target_angle_robot_frame = -(2*np.pi - self.targetGlobalAng)
        angle_robot_frame = self.globalAng
        if self.globalAng > np.pi:
            angle_robot_frame = -(2*np.pi - self.globalAng)
            
        msg.angular.z = Kp * (target_angle_robot_frame - angle_robot_frame)
        
        self.get_logger().info(f"Going straight: angle diff = {msg.angular.z}")
            
        self.vel_publisher.publish(msg)
    
    def turn_around(self):
        self.before_action_pose = self.globalAng
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.5
        self.vel_publisher.publish(msg)
        self.TURNING_AROUND = True
        
    def turning_around_goal_check(self):
        diff = abs(self.before_action_pose - self.globalAng)
        diff = min(diff, abs(2*np.pi - diff))
        
        self.get_logger().info(f"Turning around : diff = {diff}")
        
        if abs(diff) >= np.pi - self.ALLOWANCE:
            self.TURNING_AROUND = False
            self.before_action_pose = None
            self.vel_publisher.publish(Twist(linear=Vector3(x=0.0), angular=Vector3(z=0.0)))
        
    def stop(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.vel_publisher.publish(msg)
        
def main():
    rclpy.init()
    controller = Controller()
    
    try:
        rclpy.spin(controller)
    except SystemExit:
        rclpy.logging.get_logger("Controller node...").info("Shutting Down")
    
    controller.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
    
        