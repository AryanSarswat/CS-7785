#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point

import sys

import numpy as np
import cv2
from cv_bridge import CvBridge

class State():
    """
    State machine used to determine what command to publish based on sensor data
    
    State 0 : No obstacles ahead drive towards goal
    State 1 : Obstacle Avoidance
    State 2 : Reached Goal
    
    """
    def __init__(self, logger):
        self.logger = logger
        self.cur_state = 0
    
        self.min_safe_dist = 0.4
        self.EPISILON = 0.1
        self.goal_err = 0.1
        
        self.distance_travelled = 0
        self.min_distance_travelled = 0.1
        
        self.last_pos = Point()
        self.last_pos.x = 0.0
        self.last_pos.y = 0.0
        
        self.goal_state_counter = 0
        
        
    def update_state(self, distance_to_object, curr_pos, dst_pos):
        if self.cur_state == 2:
            # Stay at goal for 10 seconds
            self.goal_state_counter += 1
            if goal_state_counter == 100:
                self.cur_state = 0
        elif (distance_to_object <= self.min_safe_dist + self.EPISILON) and self.cur_state != 1:
            self.cur_state = 1
            self.distance_travelled = 0
            self.last_pos.x = curr_pos.x
            self.last_pos.y = curr_pos.y
            self.logger.info(f"Switching to state 1")
        elif self.cur_state == 1 and self.distance_travelled <= self.min_distance_travelled:
            # Stay in state 1 and update distance travelled
            self.logger.info(f"{curr_pos}, {self.last_pos}")
            self.distance_travelled = abs(self.last_pos.x - curr_pos.x) + abs(self.last_pos.y - curr_pos.y)
            self.logger.info(f"Staying in state 1, distance travelled {self.distance_travelled}")
        elif abs(curr_pos.x - dst_pos.x) + abs(curr_pos.x- dst_pos.y) <= self.goal_err and self.cur_state != 2:
            # Reached goal
            self.cur_state = 2
            self.goal_state_counter = 0
            self.logger.info(f"Switching to state 2")
        elif self.cur_state != 0:
            self.cur_state = 0
            self.logger.info(f"Switching to state 0")
        else:
            pass
            
    def get_state(self):
        return self.cur_state

class GoToGoal(Node):
    def __init__(self):
        super().__init__('GoToGoal')
        
        self.state_machine = State(self.get_logger())
        
        # Init Odom to (0,0,0) and initializep position variables
        self.Init = True
        self.Init_pos = Point()
        self.Init_pos.x = 0.0
        self.Init_pos.y = 0.0
        self.Init_ang = 0.0
        self.globalPos = Point()
        
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
        
        self.proportional_velocity_gain = 0.3
        self.proportional_angular_gain = 1.5
        
        self._vel_publisher = self.create_publisher(Twist, '/cmd_vel', 5)
        
    
    def dist_callback(self, msg):
        dst = float(msg.data)
        self.distance_to_object = dst
        #self.get_logger().info(f"Distance to Object {dst}")
    
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
        
        self.state_machine.update_state(self.distance_to_object, self.globalPos, self.goals[0])
        
        self.update_vel()  

    def update_vel(self):
        if self.state_machine.get_state() == 0:
            # Drive to goal
            x_g = self.goal[0].x
            y_g = self.goal[0].y
            
            mag = abs(self.globalPos.x - x_g) + abs(self.globalPos.y - y_g)
            ang = np.arctan2(self.globalPos.y - y_g, self.globalPos.x - x_g)
            
            u_linear = self.proportional_velocity_gain * mag
            u_angular = self.proportional_angular_gain * ang
            
            self._vel_publisher.publish(Twist(linear=Vector3(x=u_linear), angular=Vector3(z=u_angular)))
            self.get_logger().info(f"Publishing l_v : {u_linear} and a_v : {u_angular}")
            
        elif self.state_machine.get_state() == 1:
            # Obstacle Avoidance
            pass
        elif self.state_machine.get_state() == 2:
            # Reached Goals state
            if sabs(self.globalPos.x - self.goals[0].x) + abs(self.globalPos.y - self.goals[0].y) <= 0.1:
                goal_reached = self.goals.pop(0)
                self.get_logger().info(f"Reached goal {goal_reached}")
            
            if len(self.goals) == 0:
                self.get_logger().info(f"Reached all goals")
                self._vel_publisher.publish(Twist(linear=Vector3(x=0), angular=Vector3(z=0)))
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
    