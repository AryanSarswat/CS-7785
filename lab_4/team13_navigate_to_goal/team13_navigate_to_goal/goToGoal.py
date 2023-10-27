#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist, Vector3


import sys

import numpy as np
import cv2
from cv_bridge import CvBridge

def euclidean_distance(p1, p2):
    return np.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)

class State():
    """
    State machine used to determine what command to publish based on sensor data
    
    State 0 : No obstacles ahead drive towards goal
    State 1 : Obstacle Avoidance
    State 2 : Reached Goal
    
    """
    def __init__(self, logger, goals):
        self.logger = logger
        self.cur_state = 0
        self.goals = goals
    
        self.min_safe_dist = 0.3
        self.EPISILON = 0.1
        self.goal_err = 0.15
        
        self.time_elapsed = 0
        self.min_time_elasped = 60
        
        self.goal_state_counter = 0
        
        
    def update_state(self, distance_to_object, curr_pos):
        dst_pos = self.goals[0]
        if self.cur_state == 2:
            # Stay at goal for 10 seconds
            self.goal_state_counter += 1
            if self.goal_state_counter >= 50:
                self.cur_state = 0
        elif (distance_to_object <= self.min_safe_dist + self.EPISILON) and self.cur_state != 1:
            self.cur_state = 1
            self.time_elapsed = 0
            self.logger.info(f"[State] Switching to state 1")
        elif self.cur_state == 1 and self.time_elapsed <= self.min_time_elasped:
            # Stay in state 1 and update distance travelled
            self.time_elapsed += 1
            if self.time_elapsed > self.min_time_elasped:
                self.cur_state = 0
            self.logger.info(f"[State] Staying in state 1, time elapsed {self.time_elapsed}")
        elif euclidean_distance(curr_pos, dst_pos) <= self.goal_err and self.cur_state != 2:
            # Reached goal
            self.cur_state = 2
            self.goal_state_counter = 0
            self.goals.pop(0)
            self.logger.info(f"\n[State] Switching to state 2\n")
        elif self.cur_state != 0:
            self.cur_state = 0
            self.logger.info(f"[State] Switching to state 0")
        else:
            self.logger.info(f"No change in state {self.cur_state}")
            
    def get_state(self):
        return self.cur_state

class GoToGoal(Node):
    def __init__(self):
        super().__init__('GoToGoal')
        
        
        # Init Odom to (0,0,0) and initializep position variables
        self.Init = True
        self.Init_pos = Point()
        self.Init_pos.x = 0.0
        self.Init_pos.y = 0.0
        self.Init_ang = 0.0
        self.globalPos = Point()
        self.globalAng = 0.0
        
        self.prev_pos = Point()
        
        self.pos_sub = self.create_subscription(
                Odometry,
                '/odom',
                self.odom_callback,
                1)
        self.pos_sub # prevent unsued variable warning
        
        self.distance_to_object = float('inf')
        self.object_angle = 0.0
        
        self.range_sub = self.create_subscription(
                String,
                '/distance',
                self.dist_callback,
                1)
        
        self.range_sub # prevent unsued variable warning
        
        self.goals = []
        GOALS_PATH = './wayPoints.txt'
        f = open(GOALS_PATH, "r")
        txt = f.read()
        temp = txt.split("\n")[:-1]
        for goals in temp:
            xy = goals.split(" ")
            x = float(xy[0])
            y = float(xy[1])
            goal = Point()
            goal.x = x
            goal.y = y
            self.goals.append(goal)
            
        print(f"Goals : {self.goals}")   
        
        self.state_machine = State(self.get_logger(), self.goals)
        
        self.Kp_linear = 0.5
        self.Kp_angular = 1.0
        self.Kd_linear = 0.7
        self.Kd_angular = 0.5
        
        
        self._vel_publisher = self.create_publisher(Twist, '/cmd_vel', 5)
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        
    
    def timer_callback(self):
        self.state_machine.update_state(self.distance_to_object, self.globalPos)
    
    def dist_callback(self, msg):
        dst, angle = msg.data.split(",")
        dst = float(dst)
        angle = float(angle)
        self.distance_to_object = dst
        self.object_angle = angle
    
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

        #self.get_logger().info('Transformed global pose is x:{}, y:{}, a:{}'.format(self.globalPos.x,self.globalPos.y,self.globalAng)) 
        
        self.update_vel()  

    def update_vel(self):
        if self.state_machine.get_state() == 0:
            # Drive to goal
            x_g = self.state_machine.goals[0].x
            y_g = self.state_machine.goals[0].y
            
            mag = euclidean_distance(self.globalPos, self.goals[0])
            self.get_logger().info(f"y_diff = {y_g - self.globalPos.y}, x_diff = {x_g - self.globalPos.x}")
            
            ang = np.arctan2(y_g - self.globalPos.y, x_g - self.globalPos.x)
            
            ang -= self.globalAng
            
            while ang <= -np.pi:
                ang += 2*np.pi
            while ang > np.pi:
                ang -= 2*np.pi
            
            dt = 1
            linear_vel = euclidean_distance(self.prev_pos, self.globalPos)/dt
            
            self.get_logger().info(f"mag = {mag}, ang = {ang}, turtlebot_angle = {self.globalAng}")
            
            u_linear = self.Kp_linear * mag + self.Kd_linear * linear_vel
            u_angular = self.Kp_angular * ang

            u_angular = np.clip(u_angular, -1.5, 1.5)
            u_linear = np.clip(u_linear, -0.1,0.1)
            
            
            self._vel_publisher.publish(Twist(linear=Vector3(x=u_linear), angular=Vector3(z=u_angular)))
            self.get_logger().info(f"Publishing l_v : {u_linear} and a_v : {u_angular}")
            
            self.prev_pos = self.globalPos
            self.prev_ang = self.globalAng
            
        elif self.state_machine.get_state() == 1:
            # Obstacle Avoidance
            dst = self.distance_to_object
            ang = self.object_angle
            
            
            u_linear = self.Kp_linear * dst
            u_angular = self.Kp_angular * ang
            
            u_angular = np.clip(u_angular, -1.5, 1.5)
            u_linear = np.clip(u_linear, -0.1,0.1)
            
            if dst < 0.2:
                u_angular = 0.0
                u_linear = -u_linear
            
            self._vel_publisher.publish(Twist(linear=Vector3(x=u_linear), angular=Vector3(z=u_angular)))
        elif self.state_machine.get_state() == 2:
            # Near Goal state
            self._vel_publisher.publish(Twist(linear=Vector3(x=0.0), angular=Vector3(z=0.0)))
            self.prev_ang = self.globalAng
            
            if len(self.goals) == 0:
                self.get_logger().info(f"Reached all goals")
                self._vel_publisher.publish(Twist(  linear=Vector3(x=0), angular=Vector3(z=0)))
                raise KeyboardInterrupt
        else:
            self.get_logger.info(f"Invalid state of {self.state.get_state()} encountered")          

def main(args=None):
    rclpy.init(args=args)
    goToGoalNode = GoToGoal()
    rclpy.spin(goToGoalNode)
    goToGoalNode.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    
