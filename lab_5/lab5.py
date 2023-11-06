#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action._navigate_to_pose import NavigateToPose_FeedbackMessage

import sys

# Points to navigate to
GOALS = [((1,1,0),(0,0,0,0)), ((1,1,0),(0,0,0,0)), ((1,1,0),(0,0,0,0))]

class Planner(Node):
    def __init__(self, goals):
        super().__init__('planner')
        
        self.goals = goals
        
        qos_profile = QoSProfile(depth=5)
        qos_profile.history = QoSHistoryPolicy.KEEP_LAST
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        
        self.publisher = self.create_publisher(PoseStamped, '/goal_pose', qos_profile)
        
        self.subscription = self.create_subscription(
            NavigateToPose_FeedbackMessage,
            'navigate_to_pose/_action/feedback',
            self.listener_callback,
            qos_profile)
        
        self.subscription  # prevent unused variable warning
        
        self.min_dist = 0.1
        
        # Publish first goal
        self.publish_goal()

    def listener_callback(self, msg):
        dst_remaining = msg.distance_remaining
        
        if dst_remaining < self.min_dist:
            self.publish_goal()
    
    def publish_goal(self):
        if len(self.goals) == 0:
            self.get_logger().info('No more goals to publish')
            raise SystemExit
        
        goal = self.goals.pop(0)
        
        goal_to_publish = PoseStamped()
        
        goal_to_publish.pose.position = Point(*goal[0])
        goal_to_publish.pose.orientation = Quaternion(*goal[1])
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