import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from jaka_msgs.srv import Move
from visualization_msgs.msg import Marker

class ControlNode(Node):

    def __init__(self):
        super().__init__('control_node')

        # Subscribe to pixel offset from vision node
        self.subscription = self.create_subscription(
            Point,
            '/target_pixel',
            self.pixel_callback,
            10
        )

        # Service client — connects to jaka_driver (real or fake)
        self.client = self.create_client(Move, '/jaka_driver/linear_move')

        # RViz marker publisher
        self.marker_pub = self.create_publisher(Marker, '/target_marker', 10)

        # ── TUNING PARAMETERS ──────────────────────────────────────
        # Scale: how many mm per pixel (calibrate on real arm with ruler)
        self.mm_per_pixel = 0.5

        # Proportional gain: higher = faster response, lower = smoother
        self.gain = 0.4

        # Max mm per single move command — safety limit
        self.max_step_mm = 50.0

        # Alignment threshold in pixels — stop moving when within this
        self.threshold = 10

        # Fixed Z height — arm never changes height (mm above table)
        self.fixed_z = 300.0

        # Fixed wrist orientation — don't rotate wrist
        self.fixed_rx = 3.14
        self.fixed_ry = 0.0
        self.fixed_rz = 0.0

        # ── WORKSPACE SAFETY LIMITS (mm) ───────────────────────────
        # Arm will never move outside these boundaries
        # Set these based on your physical setup at the academy
        self.x_min = -300.0
        self.x_max =  300.0
        self.y_min = -300.0
        self.y_max =  300.0
        self.z_min =  200.0   # Never go below 200mm (table protection)

        # Accumulated position tracking for workspace limit checking
        self.current_x = 0.0
        self.current_y = 0.0

        # Flag: don't send new command while waiting for arm response
        self.waiting_for_response = False

        self.get_logger().info('Control node started! Waiting for pixel offsets...')

    def pixel_callback(self, msg):
        offset_x = msg.x
        offset_y = msg.y

        # Skip if still waiting for previous arm response
        if self.waiting_for_response:
            return

        # Convert pixel offset to mm using proportional gain
        move_x = offset_x * self.mm_per_pixel * self.gain
        move_y = offset_y * self.mm_per_pixel * self.gain

        # Clamp to max step size per command
        move_x = max(-self.max_step_mm, min(self.max_step_mm, move_x))
        move_y = max(-self.max_step_mm, min(self.max_step_mm, move_y))

        # Check workspace limits — prevent arm going out of bounds
        next_x = self.current_x + move_x
        next_y = self.current_y + move_y

        if next_x < self.x_min or next_x > self.x_max:
            self.get_logger().warn(
                f'X limit reached! current={self.current_x:.1f}, '
                f'requested={next_x:.1f} — move blocked'
            )
            move_x = 0.0

        if next_y < self.y_min or next_y > self.y_max:
            self.get_logger().warn(
                f'Y limit reached! current={self.current_y:.1f}, '
                f'requested={next_y:.1f} — move blocked'
            )
            move_y = 0.0

        # Publish RViz marker at target position
        self.publish_marker(self.current_x + move_x, self.current_y + move_y)

        # Check alignment — stop if object is centered
        if abs(offset_x) < self.threshold and abs(offset_y) < self.threshold:
            self.get_logger().info('TARGET ALIGNED — arm holding position ✓')
            return

        self.get_logger().info(
            f'Offset: x={offset_x:.0f}px, y={offset_y:.0f}px → '
            f'Move: x={move_x:.1f}mm, y={move_y:.1f}mm'
        )

        # Build move request
        req = Move.Request()
        req.pose      = [move_x, move_y, self.fixed_z,
                         self.fixed_rx, self.fixed_ry, self.fixed_rz]
        req.has_ref   = False
        req.ref_joint = [0.0]
        req.mvvelo    = 50.0
        req.mvacc     = 50.0
        req.mvtime    = 0.0
        req.mvradii   = 0.0
        req.coord_mode = 0
        req.index     = 0

        # Wait for service to be available
        if not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Arm service not available!')
            return

        # Send command asynchronously
        self.waiting_for_response = True
        future = self.client.call_async(req)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        self.waiting_for_response = False
        try:
            response = future.result()
            if response.ret == 0:
                self.get_logger().info('Arm moved successfully ✓')
            else:
                self.get_logger().warn(f'Arm error: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def publish_marker(self, x_mm, y_mm):
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'target'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # Convert mm to meters for RViz
        marker.pose.position.x = x_mm / 1000.0
        marker.pose.position.y = y_mm / 1000.0
        marker.pose.position.z = self.fixed_z / 1000.0
        marker.pose.orientation.w = 1.0

        # Sphere size
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05

        # Red color
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
