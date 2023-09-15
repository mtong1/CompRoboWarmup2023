import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')
        self.scan = self.create_subscription(LaserScan, 'scan', self.parse_scans, 10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.move = Twist()
        self.scan_msg = None

        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.run_loop)

    def parse_scans(self, msg):
        '''
        Checks messages from /scan topic, determines if there is an object within 
        40 degrees and 5m of its front side. If there is, the angle of the object is saved.
        '''
        self.scan_msg = msg
        self.closest_angle = 0

        for i in range (0, 20):
            if self.scan_msg.ranges[i] < 5:
                self.closest_angle = i

        for i in range(340,361): 
            if self.scan_msg.ranges[i] < 5:
                self.closest_angle = i

    def move_robot(self):
        '''
        If the object is on the left side of the bot, it turns right. If the object is on the
        right side of the bot, it turns left. If the object is neither, it keeps driving forward.

        Should there be an object in front of it initially, the ROS loop will keep it turning
        until it no longer sees something.
        '''
        # if the object is on left side 
        if self.closest_angle <= 20 and self.closest_angle > 0:
            self.move.linear.x = 0.0
            self.move.angular.z = -0.3
            print("turn right")
            
        # if object is on right side 
        if self.closest_angle > 340 and self.closest_angle <= 361:
            self.move.linear.x = 0.0
            self.move.angular.z = 0.3
            print("turn left")

        # if no object in front 
        else: 
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
