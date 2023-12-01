import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from math import pi
import time


class Lawnmower(Node):
    def __init__(self):
        super().__init__('lawnmower')
        self.state = "move_straight"
        self.scan = self.create_subscription(LaserScan, 'scan', self.parse_scans, 10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.move = Twist()
        self.scan_msg = None
        self.closest_angle = 0
        #Hard coded now, but brain will control later
        # in meters
        self.init_x = 0 
        self.init_y = 0
        self.current_x = 0
        self.current_y = 0
        self.max_x = 6
        self.max_y = 6
        self.linear_speed = 0.6 # CHANGE VALUE 
        ## orienting robot in direction of most weighted/closest objects 
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.run_loop)



    def move_straight(self):
        '''
        Moves robot based on "prey" state behavior-- it moves away from any object in front of it.
        '''
        print(self.state)
        self.move.linear.x = self.linear_speed
        self.publisher.publish(self.move)

    def move_up_right(self):
        '''
        Moves robot according to "predator" state behavior-- it moves towards the closest object.
        '''
        print(self.state)
    
        self.publisher.publish(self.move)
    def move_up_left(self):
        self.publisher.publish(self.move)
    def find_object(): 
        """
        Publish message to main brain 
        """
        print("Send coordiante to main brain, activating other robots to move to locaization")
    def complete_search():
        print("Send message that area assigned is searched")


    def run_loop(self):
        '''
        Switches state from predator to prey when the robot "tags" something/someone.
        '''
        #logic for turning
        if self.current_x  == self.init_x and self.current_y > self.init_y: 
            self.state = "move_up_right"
        elif self.current_x == self.max_x: 
            self.state = "move_up_left"

        match self.state:
            case "move_straight":
                self.move_straight()
            case "move_up_right":
                self.move_up_right()
            case "move_up_left":
                self.move_up_left
            case "find_object": 
                self.find_object
            case "complete_area": 
                self.complete_search
            case default:
                return "stuck"

def main(args=None):
    rclpy.init(args=args)
    node = Lawnmower()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()