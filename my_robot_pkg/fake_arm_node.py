import rclpy
from rclpy.node import Node
from jaka_msgs.srv import Move

class FakeArmNode(Node):

    def __init__(self):
        super().__init__('fake_arm_node')

        # Create a SERVICE SERVER
        # This pretends to be the real jaka_driver
        # It listens on the exact same service name
        self.srv = self.create_service(
            Move,
            '/jaka_driver/linear_move',
            self.handle_move_request
        )

        self.get_logger().info('Fake arm ready. Waiting for move commands...')

    def handle_move_request(self, request, response):
        # This runs every time Node 2 sends a move command
        x = request.pose[0]
        y = request.pose[1]
        z = request.pose[2]

        self.get_logger().info(
            f'MOVE COMMAND RECEIVED → x={x:.1f}mm, y={y:.1f}mm, z={z:.1f}mm'
        )

        # Send back a success response
        response.ret = 0
        response.message = 'success'
        return response

def main(args=None):
    rclpy.init(args=args)
    node = FakeArmNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
