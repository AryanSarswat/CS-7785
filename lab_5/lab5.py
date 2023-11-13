#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped, Twist, Vector3
from nav2_msgs.action._navigate_to_pose import NavigateToPose_FeedbackMessage

import sys

# Points to navigate to
GOALS = [((1.9639177322387695,0.15323543548583984,-0.001434326171875),(0.0,0.0,0.0,1.0)),
         ((1.0109432935714722,-1.731042146682739,0.0025329589843),(0.0,0.0,0.0,1.0)),
         ((0.115954212844371, -0.854123294353,-0.001434326171),(0.0,0.0,0.0,1.0))
         ]

class Planner(Node):
    def __init__(self, goals):  
        super().__init__('planner') 
        
        self.goals = goals
        
        qos_profile = QoSProfile(depth=5)
        qos_profile.history = QoSHistoryPolicy.KEEP_LAST
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        
        self.publisher = self.create_publisher(PoseStamped, '/goal_pose', qos_profile)
        
        self._vel_publisher = self.create_publisher(Twist, '/cmd_vel', 5)
        
        self.subscription = self.create_subscription(
            NavigateToPose_FeedbackMessage,
            'navigate_to_pose/_action/feedback',
            self.listener_callback,
            qos_profile)
        
        self.subscription  # prevent unused variable warning
        
        self.min_dist = 0.2
        
        self.timer = self.create_timer(0.5, self.timer_callback)
        
    def timer_callback(self):
        self.publish_goal()

    def listener_callback(self, msg):
        position = msg.feedback.current_pose.pose.position
        
        x = position.x
        y = position.y
        z = position.z
        
        goal = self.goals[0][0]
        
        g_x = goal[0]
        g_y = goal[1]
        g_z = goal[2]
        
        dst_remaining = ((g_x - x)**2 + (g_y - y)**2 + (g_z - z)**2)**0.5
        #dst_remaining = msg.feedback.distance_remaining
                
        if dst_remaining < self.min_dist:
            self.goals.pop(0)
            if len(self.goals) == 0:
                self.goals.append(((x,y,z),(0.0,0.0,0.0,1.0)))
            self.publish_goal()
    
    def publish_goal(self):
        if len(self.goals) == 0:
            self.get_logger().info('No more goals to publish')
            raise SystemExit
        
        goal = self.goals[0]
        
        goal_to_publish = PoseStamped()
        
        goal_to_publish.pose.position = Point()
        goal_to_publish.pose.position.x = goal[0][0]
        goal_to_publish.pose.position.y = goal[0][1]
        goal_to_publish.pose.position.z = goal[0][2]
        goal_to_publish.pose.orientation = Quaternion()
        goal_to_publish.pose.orientation.x = goal[1][0]
        goal_to_publish.pose.orientation.y = goal[1][0]
        goal_to_publish.pose.orientation.z = goal[1][0]
        goal_to_publish.pose.orientation.w = goal[1][0]
        
        goal_to_publish.header.frame_id = "map"
        
        self.publisher.publish(goal_to_publish)
        
        self.get_logger().info('Publishing goal: %s' % goal_to_publish)
        
def main():
    rclpy.init()
    planner = Planner(GOALS)
    try:
        rclpy.spin(planner)
    except SystemExit:
        rclpy.logging.get_logger("Planner").info("Shutting Down")
    
    planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()