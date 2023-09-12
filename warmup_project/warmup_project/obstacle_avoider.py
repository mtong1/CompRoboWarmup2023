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
        self.closest_angle = 0

    def parse_scans(self, msg):
        self.scan_msg = msg

        for i in range(0,360):
            if self.scan_msg.ranges[i] < 1.5: # if there are objects less than 3 meters away
                self.closest_angle = i

    def run_loop(self):
        if self.closest_angle != 0 and (0 <= self.closest_angle <= 20 | 359 >= self.closest_angle >= 340):
            pass


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()
