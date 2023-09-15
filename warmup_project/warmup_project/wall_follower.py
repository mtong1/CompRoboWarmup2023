import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
import numpy as np
import time

class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.move = Twist()
        self.linear_speed = 0.3
        self.angular_speed = 2.0
        self.l1 = None
        self.l2 = None
        self.scan_msg = None
        self.laser = self.create_subscription(LaserScan, 'scan', self.parse_scan, 10)
        timer_period = 2
        self.timer = self.create_timer(timer_period, self.run_loop)



    def parse_scan(self, msg):
        # values of distances from robot to wall 
        self.scan_msg = msg
        self.l1 = self.scan_msg.ranges[45]
        self.l2 = self.scan_msg.ranges[135]
        print(f"l1 = {self.l1} l2 = {self.l2}")

    def magnitude(self, vector):
        return math.sqrt(sum(pow(element, 2) for element in vector))

    def run_robot(self, angle):
        print("run robot")
        turn_time = (angle)/self.angular_speed
        self.move.linear.x = 0.0
        self.move.angular.z = -1 * self.angular_speed
        self.vel_pub.publish(self.move)
        time.sleep(turn_time)
        print("turned")
        self.move.angular.z = 0.0
        self.move.linear.x = self.linear_speed
        self.vel_pub.publish(self.move)
        print("straight")

    def robot_stop(self):
        self.move.linear.x = 0.0
        self.move.angular.z = 0.0
        self.vel_pub.publish(self.move)

    def run_loop(self):

        #calculating l1 distance from wall
        self.robot_stop()
        if self.l1 == math.inf or self.l2 == math.inf:
            return
        angle1 = math.pi/4
        l1_x = math.cos(angle1)*self.l1
        l1_y = math.sin(angle1)*self.l1
        angle2 = math.pi*3/4
        # calculating l2 distance from wall
        l2_x = math.cos(angle2)*self.l2
        l2_y = math.sin(angle2)*self.l2
        # shift the wall coordinates over robot frame
        wall_x = l1_x - l2_x
        wall_y = l1_y - l2_y

        robot_frame = [1,0]
        wall_vec = [wall_x, wall_y]
        parta = np.dot(robot_frame, wall_vec)
        partb = np.multiply(self.magnitude(robot_frame),self.magnitude(wall_vec))
        if partb == 0:
            print("is 0")
            return
        partc = parta/partb
        move_angle = math.acos(partc)
        print(f"move angle: {move_angle}")
        self.run_robot(move_angle)


def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# first scan the walls from 45 deg, find the distances, relate the distances to 
# our robot's coordinate frame, find the angle to turn the robot to be parallel 
