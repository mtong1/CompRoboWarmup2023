import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')
        self.scan = self.create_subscription(LaserScan, 'scan', self.parse_scans, 10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.move = Twist()
        self.scan_msg = None
        self.closest_dist = math.inf
        self.closest_angle = 0

        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.run_loop)

    def parse_scans(self, msg):
        self.scan_msg = msg
        self.closest_angle = 0
        for i in range (0, 20):
            if self.scan_msg.ranges[i] < 5 :#and self.scan_msg.ranges[i] < self.closest_dist: 
                # self.closest_dist = self.scan_msg.ranges[i]
                self.closest_angle = i

        for i in range(340,361): 
            if self.scan_msg.ranges[i] < 5: # and self.scan_msg.ranges[i] < self.closest_dist :
                # self.closest_dist = self.scan_msg.ranges[i]
                self.closest_angle = i
        print(f"closest angle : {self.closest_angle}")
        # print(f"closest distance; {self.closest_dist}")

    def move_robot(self):
        if self.closest_angle <= 20 and self.closest_angle > 0: # if the object is on left side 
            self.move.linear.x = 0.0
            self.move.angular.z = -0.3
            print("turn right")
            
        if self.closest_angle > 340 and self.closest_angle <= 361: # if object is on right side 
            self.move.linear.x = 0.0
            self.move.angular.z = 0.3
            print("turn left")

        else: #if no object in front 
            self.move.linear.x = 0.3
            self.move.angular.z = 0.0
            print("move straight")

        self.publisher.publish(self.move)


    def run_loop(self):
        self.move_robot()


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()
