import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
import numpy as np
import time

class WallFollower(Node):
    def __init__(self):
        super.__init__('wall follower')

        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.move = Twist()
        self.linear_speed = 0.3
        self.angular_speed = 2.0

        self.laser = self.create_subscription(LaserScan, 'scan', self.LasScan)
        self.scan_msg = LaserScan()
        self.run_loop()


    def LasScan(self, angle):
        # values of distances from robot to wall 
        return self.scan_msg.ranges(angle)


    def run_robot(self, angle):
        turn_time = (angle * (math.pi/180))/self.angular_speed
        self.move.linear.x = 0.0
        self.move.angular.z = self.angular_speed
        self.vel_pub.publish(self.move)
        time.sleep(turn_time)
        self.move.angular.z = 0.0
        self.move.linear.x = self.linear_speed
        self.vel_pub.publish(self.move)
        


    def run_loop(self):
        # calculating l1 distance from wall
        l1 = self.LasScan(45)
        angle = math.pi/4
        l1_x = math.cos(angle)*l1
        l1_y = math.sin(angle)*l1

        # calculating l2 distance from wall
        l2 = self.LasScan(135)
        l2_x = math.cos(angle)*l2
        l2_y = math.sin(angle)*l2

        # shift the wall coordinates over robot frame
        l2_x = l2_x - l1_x
        l2_y = l2_y - l1_y

        robot_frame = [0,1]
        l2_vec = [l2_x, l2_y]
        move_angle = math.acos(np.dot(robot_frame, l2_vec) / np.multiply(np.absolute(robot_frame),np.absolute(l2_vec)))

        self.run_robot(move_angle)




# first scan the walls from 45 deg, find the distances, relate the distances to 
# our robot's coordinate frame, find the angle to turn the robot to be parallel 
