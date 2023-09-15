import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from math import pi
import time


class DriveSquare(Node):
    def __init__(self):
        super().__init__('drive_square')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.move = Twist() # messages to move the robot 
        self.linear_speed = 0.3
        self.angular_speed = 1.0
        self.acceleration_time = 0.03
        self.line_time = 1/self.linear_speed
        self.turn_time = (pi/2)/self.angular_speed + self.acceleration_time
        self.run_loop()
    
    def move_line(self):
        '''
        Drives in a straight, 1m line for a determined time.
        '''
        self.move.linear.x = self.linear_speed
        self.move.angular.z = 0.0
        self.publisher.publish(self.move)
        time.sleep(self.line_time)

    def move_corner(self):
        '''
        Turns 90 degrees using calculations.
        '''
        self.move.angular.z = self.angular_speed 
        self.move.linear.x = 0.0
        self.publisher.publish(self.move)
        time.sleep(self.turn_time)

    def run_loop(self):
        '''
        Drives straight and turns 90 degrees 4 times to drive in a 1m square.
        '''
        for x in range(4):
            self.move_line()
            self.move_corner()
        self.move.linear.x = 0.0
        self.move.angular.z = 0.0
        self.publisher.publish(self.move)
        return
    
def main(args=None):
    rclpy.init(args=args)
    node = DriveSquare()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
