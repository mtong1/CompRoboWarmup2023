import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class PersonFollower(Node):
    def __init__(self):
        super().__init__('person_follower')
        self.scan = self.create_subscription(LaserScan, 'scan', self.parse_scans, 10)
        self.timer = self.create_timer(timer_period, self.run_loop)
        self.move = Twist()
        self.scan_msg = None
        self.closest_dist= 0
        self.closest_angle = 0
        timer_period = 1

    def parse_scans(self, msg):
        '''
        Checks laserscan messages from /scan topic, figures out where the closest object
        is by parsing each angle and finding the closest distance, as well as which angle.
        '''
        self.scan_msg = msg
        for i in range(0,360):
            if self.scan_msg.ranges[i] < self.closest_dist and self.scan_msg.ranges[i] < 3: # units: meters
                self.closest_dist = self.scan_msg.ranges[i]
                self.closest_angle = i

    def orient_robot(self):
        '''
        Tells the robot to turn to a certain angle relative to its coordinate frame and
        drive forwards.
        '''
        # object is to the left 
        if self.closest_angle > 20 and self.closest_angle <= 180:
            self.move.angular.z = 1.0

        # object is to the right
        elif self.closest_angle > 180 and self.closest_angle <= 340:
            self.move.angular.z = -1.0

        # object is in front
        elif 0 <= self.closest_angle <= 20 | 340 < self.closest_angle <= 359:
            self.move.linear.x = 1.0
        

    def run_loop(self):
        self.orient_robot()

def main(args=None):
    rclpy.init(args=args)
    node = PersonFollower()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()
