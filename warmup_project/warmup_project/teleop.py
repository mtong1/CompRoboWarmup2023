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
        self.publisher = self.create_publisher(Twist, 'vel', 10)
        self.move = Twist()

    def getKey(self):
        settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        self.key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return self.key 

    def run_loop(self):
        key = None
        while key != '\x03':            # ctrl C to quit 
            key = self.getKey()
            
            if key == "w":
                self.move.linear.x = 1.0
                self.publisher.publish(self.move)

            if key == "a":
                self.move.linear.z = 1.0
                self.publisher.publish(self.move)

            if key == "s":
                self.move.linear.x = -1.0
                self.publisher.publish(self.move)

            if key == "d":
                self.move.linear.z = -1.0
                self.publisher.publish(self.move)
            # print(key)



def main(args=None):
    rclpy.init(args=args) # instance 
    node = Teleop() 
    rclpy.spin(node)  #starts up node, rns stuff in class 
    node.destroy_node()
    rclpy.shutdown()  # will properly shut down node 

if __name__ == '__main__':
    main()


