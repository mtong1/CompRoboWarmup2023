import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')
        self.scan = self.create_subscription(LaserScan, 'scan', self.parse_scans, 10)
        self.move = Twist()
        self.scan_msg = None
        self.closest_dist = 0
        self.closest_angle = 0

        timer_period = 1
        self.timer = self.create_timer(timer_period, self.run_loop)

    def parse_scans(self, msg):
        self.scan_msg = msg

# if the object is in front, keep note of which side its on
        for i in range (0, 20):
            if self.scan_msg.ranges[i] < 5:
                self.move.linear.x = 0.0
                self.move.angular.z = 0.3
            else:
                self.move.linear.x = 0.3

        for i in range(340,361):        # to the right
            if self.scan_msg.ranges[i] < 5:
                self.move.linear.x = 0.0
                self.move.angular.z = -0.3
            else:
                self.move.linear.x = 0.3



    def run_loop(self):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()
