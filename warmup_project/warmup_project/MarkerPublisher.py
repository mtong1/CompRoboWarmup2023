import rclpy
from rclpy.node import Node 
from visualization_msgs.msg import Marker

class MarkerPublisher(Node):
    def __init__(self):
        super().__init__('marker_publisher') # where you write name of node 
        self.visual_pub = self.create_publisher(Marker,'visualization_marker', 10)
        publish_viz
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.publish_marker)

    def publish_marker(self):
        marker = Marker() # initialize marker
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "marker_sphere"
        marker.id = 0

        # making sphere at (1,2)
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = 1.0
        marker.pose.position.y = 2.0
        marker.pose.position.z = 1.0

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1.0
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0; # Don't forget to set the alpha!
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        self.visual_pub.publish(marker)


            

def main(args=None):
    rclpy.init(args=args) # instance 
    node = MarkerPublisher() 
    rclpy.spin(node)  #starts up node, rns stuff in class 
    node.destroy_node()
    rclpy.shutdown()  # will properly shut down node 

if __name__ == '__main__':
    main()
