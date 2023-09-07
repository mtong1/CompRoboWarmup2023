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
    def getKey():
        settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key
    def run_loop(self):
        key = None
        while key != '\x03':
            key = getKey()
            print(key)
        

def main(args=None):
    rclpy.init(args=args) # instance 
    node = Teleop() 
    rclpy.spin(node)  #starts up node, rns stuff in class 
    node.destroy_node()
    rclpy.shutdown()  # will properly shut down node 

if __name__ == '__main__':
    main()


