import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

class PublisherNode(Node):

    def __init__(self):
        super().__init__('publisher_node')
        
        # Create a publisher on topic /target_pixel
        # Message type: Point (has x, y, z fields)
        self.publisher = self.create_publisher(Point, '/target_pixel', 10)
        
        # Every 0.5 seconds, call timer_callback
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.get_logger().info('Publisher node started!')

    def timer_callback(self):
        msg = Point()
        msg.x = 320.0   # pretend pixel u coordinate
        msg.y = 240.0   # pretend pixel v coordinate
        msg.z = 0.0     # we don't use z, always 0
        
        self.publisher.publish(msg)
        self.get_logger().info(f'Published pixel: x={msg.x}, y={msg.y}')

def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
