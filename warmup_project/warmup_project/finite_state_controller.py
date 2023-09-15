import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

class FiniteStateController(Node):
    def __init__(self):
        super().__init__('finite_state_controller')
        self.state = "predator"
        self.scan = self.create_subscription(LaserScan, 'scan', self.parse_scans, 10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.move = Twist()
        self.scan_msg = None
        self.closest_dist = math.inf
        self.closest_angle = 0
        ## orienting robot in direction of most weighted/closest objects 
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.run_loop)


    def parse_scans(self, msg):
        '''
        Checks laserscan messages from /scan topic, figures out where the closest object
        is by parsing each angle and finding the closest distance, as well as which angle.
        '''
        self.scan_msg = msg
        if self.state == "predator":
            #Find closest object
            self.scan_msg = msg    
            self.closest_angle = 0
            for i in range(0,360):
                if self.scan_msg.ranges[i] < self.closest_dist and self.scan_msg.ranges[i] < 3: # units: meters
                    self.closest_dist = self.scan_msg.ranges[i]
                    self.closest_angle = i

            print(self.closest_dist)
        else: 
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

    def move_robot_prey(self):
        print(self.state)
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

    def move_robot_predator(self):
        '''
        Tells the robot to turn to a certain angle relative to its coordinate frame and
        drive forwards.
        '''
        print(self.state)
        if self.closest_angle > 20 and self.closest_angle <= 180:
            # object is to the left 
            self.move.linear.x = 0.0
            self.move.angular.z = .3
            print("Turning left")
        elif self.closest_angle > 180 and self.closest_angle <= 340:
            # object is to the right
            self.move.linear.x = 0.0
            self.move.angular.z = -0.3
            print("Turning right")
        else:
            # object is in front
            self.move.angular.z = 0.0
            self.move.linear.x = 1.0
            print("going forward")
        print(f"linear = {self.move.linear}")
        print(f"angular = {self.move.angular}")
        self.publisher.publish(self.move)
        

    def run_loop(self):
        if self.closest_dist < 0.3:
            self.state = "prey"
            print("change")

        if self.state == "predator":
            self.move_robot_predator()
        else:
            self.move_robot_prey()


        


def main(args=None):
    rclpy.init(args=args)
    node = FiniteStateController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()
