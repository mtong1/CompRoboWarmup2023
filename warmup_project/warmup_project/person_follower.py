import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

class PersonFollower(Node):
    def __init__(self):
        super().__init__('person_follower')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan = self.create_subscription(LaserScan, 'scan', self.parse_scans, 10)
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.run_loop)
        self.move = Twist()
        self.scan_msg = None
        self.closest_dist = math.inf
        self.closest_angle = 0

    def parse_scans(self, msg):
        '''
        Checks laserscan messages from /scan topic, figures out where the closest object
        is by parsing each angle and finding the closest distance, as well as which angle.
        '''
        self.scan_msg = msg
        self.closest_angle = 0 
        self.closest_dist = math.inf
        for i in range(0,360):
            if self.scan_msg.ranges[i] < self.closest_dist and self.scan_msg.ranges[i] < 3: # units: meters
                self.closest_dist = self.scan_msg.ranges[i]
                self.closest_angle = i
        print(self.closest_dist)

    def orient_robot(self):
        '''
        Tells the robot to turn to a certain angle relative to its coordinate frame and
        drive forwards.
        '''
        # object is to the left 
        if self.closest_angle > 20 and self.closest_angle <= 180:
            self.move.angular.z = 0.3
            self.move.linear.x = 0.0

        # object is to the right
        elif self.closest_angle > 180 and self.closest_angle <= 340:
            self.move.angular.z = -0.3
            self.move.linear.x = 0.0

        # object is in front
        else:
            self.move.linear.x = 0.3
            self.move.angular.z = 0.0

        self.publisher.publish(self.move)
        

    def run_loop(self):
        self.orient_robot()

def main(args=None):
    rclpy.init(args=args)
    node = PersonFollower()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()
