import tty
import select
import sys
import termios
import rclpy
from rclpy.node import Node 
from geometry_msgs.msg import Twist

 

class Teleop(Node):
    def __init__(self):
        super().__init__('teleop') # where you write name of node 
        timer_period = 0.2
        self.timer = self.create_timer(timer_period, self.run_loop)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.move = Twist() # messages to move the robot 

    def getKey(self): # retrieves the keyboard key pushed 
        settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        self.key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return self.key 

    def run_loop(self):
        key = None
        while key != '\x03':            # ctrl C to quit ros2 and run_loop
            key = self.getKey()         # retrieves key that was pressed 
            
            if key == "w":              # goes forward 
                self.move.angular.z = 0.0
                self.move.linear.x = 0.3
                self.publisher.publish(self.move)

            if key == "a":              # rotates left 
                self.move.linear.x = 0.0
                self.move.angular.z = 1.0
                self.publisher.publish(self.move)

            if key == "s":              # goes backward
                self.move.angular.z = 0.0
                self.move.linear.x = -0.3
                self.publisher.publish(self.move)

            if key == "d":              # rotates right
                self.move.linear.x = 0.0
                self.move.angular.z = -1.0
                self.publisher.publish(self.move)
        print("exit")
        self.move.linear.x = 0.0
        self.move.angular.z = 0.0
        self.publisher.publish(self.move)
        break




def main(args=None):
    rclpy.init(args=args) # instance 
    node = Teleop() 
    rclpy.spin(node)  #starts up node, rns stuff in class 
    node.destroy_node()
    rclpy.shutdown()  # will properly shut down node 

if __name__ == '__main__':
    main()


