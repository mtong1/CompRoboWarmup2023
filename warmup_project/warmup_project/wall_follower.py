import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
import numpy as np
import time
from visualization_msgs.msg import Marker


class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.vis_marker = self.create_publisher(Marker,'visualization_marker', 10)
        self.laser = self.create_subscription(LaserScan, 'scan', self.parse_scan, 10)
        timer_period = 0.2
        self.timer = self.create_timer(timer_period, self.run_loop)
        self.move = Twist()
        self.linear_speed = 0.3
        self.angular_speed = 0.1
        self.l1 = None
        self.l2 = None
        self.scan_msg = None


    def parse_scan(self, msg):
        '''
        Checks the distance of the wall at specifically angle 45
        and 135 of the laser, and saves the distance values.
        '''
        # values of distances from robot to wall 
        self.scan_msg = msg
        self.l1 = self.scan_msg.ranges[45]
        self.l2 = self.scan_msg.ranges[135]
        print(f"l1 = {self.l1} l2 = {self.l2}")

    def magnitude(self, vector):
        '''
        Function that returns the magnitude of a vector.
        '''
        return math.sqrt(sum(pow(element, 2) for element in vector))

    def run_robot(self, angle):
        '''
        Function that determines the turn speed of the robot based on how
        much you want to rotate. After determining speed, the robot will
        move accordingly then drive straight.
        '''
        print(f"angle: {angle}")
        if angle > 0 and angle < 0.04:
            self.move.angular.z = 0.0
            self.move.linear.x = self.linear_speed
            print("forward")
        else:
            self.move.linear.x = 0.0
            self.move.angular.z = -1*self.angular_speed
            print("turning")
        
        self.vel_pub.publish(self.move)

    def robot_stop(self):
        '''
        Function that stops all movement from the robot.
        '''
        self.move.linear.x = 0.0
        self.move.angular.z = 0.0
        self.vel_pub.publish(self.move)

    def publish_markers(self):
        '''
        Publishes two markers at where the wall is being sensed.
        '''
        # calculating l1 distance from wall
        angle1 = math.pi/4
        l1_x = math.cos(angle1)*self.l1
        l1_y = math.sin(angle1)*self.l1

        # calculating l2 distance from wall
        angle2 = math.pi*3/4
        l2_x = math.cos(angle2)*self.l2
        l2_y = math.sin(angle2)*self.l2

        marker_1 = Marker() # initialize marker
        marker_1.header.frame_id = "base_link" 
        marker_1.type = Marker.SPHERE
        marker_1.action = Marker.ADD
        marker_1.pose.position.x = l1_x
        marker_1.pose.position.y = l1_y
        marker_1.pose.position.z = 0.0
        marker_1.scale.x = 0.1
        marker_1.scale.y = 0.1
        marker_1.scale.z = 0.1
        marker_1.color.g = 1.0
        marker_1.color.a = 1.0
        self.vis_marker.publish(marker_1)

        marker_2 = Marker()
        marker_2.header.frame_id = "base_link"
        marker_2.type = Marker.SPHERE
        marker_2.action = Marker.ADD
        marker_2.pose.position.x = l2_x
        marker_2.pose.position.y = l2_y
        marker_2.pose.position.z = 0.0
        marker_2.scale.x = 0.1
        marker_2.scale.y = 0.1
        marker_2.scale.z = 0.1
        marker_2.color.b = 1.0
        marker_2.color.a = 1.0

        self.vis_marker.publish(marker_2)

    def run_loop(self):
        '''
        Using distances calculated at 45 and 135 degrees from the robot,
        we use trigonometric math to determine the coordinates of the wall
        at these points in terms of the robot's coordinate frame. Then, we
        shift the wall vector over the robot coordinate plane/y-vector, and
        use an equation to determine the angle between the two vectors. Then,
        the robot will turn and drive parallel.
        '''
        self.robot_stop()
        if self.l1 == math.inf or self.l2 == math.inf:
            return
        
        self.publish_markers()

        # derived equation of distance between two vectors        
        move_angle = math.acos((self.l1+self.l2)/(math.sqrt(2*(self.l1**2+self.l2**2))))

        self.run_robot(move_angle)




def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
