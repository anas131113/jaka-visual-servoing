import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

class SubscriberNode(Node):

    def __init__(self):
        super().__init__('subscriber_node')
        
        # Subscribe to /target_pixel topic
        # Every time a message arrives, call pixel_callback
        self.subscription = self.create_subscription(
            Point,              # message type
            '/target_pixel',    # topic name
            self.pixel_callback,# function to call
            10                  # queue size
        )
        self.get_logger().info('Subscriber node started! Waiting for coordinates...')

    def pixel_callback(self, msg):
        # This runs every time publisher sends a message
        self.get_logger().info(f'Received pixel coordinates: x={msg.x}, y={msg.y}')

def main(args=None):
    rclpy.init(args=args)
    node = SubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
