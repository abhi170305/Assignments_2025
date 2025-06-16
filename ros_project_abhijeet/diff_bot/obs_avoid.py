#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoidance:
    def __init__(self):
        rospy.init_node('obstacle_avoidance_node', anonymous=True)

        # Publisher to control robot's movement
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Subscriber to get LIDAR data
        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)

        # Speed settings
        self.linear_speed = 0.2
        self.angular_speed = 0.2

    def lidar_callback(self, msg):
        # Get the range data (array of distances)
        ranges = msg.ranges

        # Check if there's an obstacle within 0.5 meters
        if min(ranges) < 0.5:
            self.avoid_obstacle()
        else:
            self.move_forward()

    def move_forward(self):
        # Move the robot forward with a certain linear speed
        twist = Twist()
        twist.linear.x = self.linear_speed
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

    def avoid_obstacle(self):
        # Stop the robot and then turn it to avoid the obstacle
        twist = Twist()
        twist.linear.x = 0.0  # Stop the robot
        twist.angular.z = self.angular_speed  # Turn the robot to avoid the obstacle
        self.cmd_vel_pub.publish(twist)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        # Initialize and run the obstacle avoidance node
        avoidance_system = ObstacleAvoidance()
        avoidance_system.run()
    except rospy.ROSInterruptException:
        pass
