import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from jaka_msgs.srv import Move
from visualization_msgs.msg import Marker
from tf2_ros import Buffer, TransformListener


class ControlNode(Node):

    def __init__(self):
        super().__init__('control_node')

        # Subscribe to pixel offset from vision node
        self.subscription = self.create_subscription(
            Point, '/target_pixel', self.pixel_callback, 10
        )

        # Service client
        self.client = self.create_client(Move, '/jaka_driver/linear_move')

        # RViz marker publisher — shows dummy_tcp position
        self.marker_pub = self.create_publisher(Marker, '/tcp_marker', 10)

        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Marker timer — 10Hz, independent of move commands
        self.marker_timer = self.create_timer(0.1, self.publish_tcp_marker)

        # ── TUNING PARAMETERS ──────────────────────────────────────
        self.mm_per_pixel = 0.5
        self.gain = 0.4
        self.max_step_mm = 50.0
        self.threshold = 10

        # Fixed wrist orientation
        self.fixed_rx = 3.14
        self.fixed_ry = 0.0
        self.fixed_rz = 0.0

        # ── WORKSPACE SAFETY LIMITS (mm) ───────────────────────────
        self.x_min = -300.0
        self.x_max = 300.0
        self.y_min = -300.0
        self.y_max = 300.0

        # State flags
        self.waiting_for_response = False
        self.service_ready = False
        self.homed = False

        # Fixed Z — locked after first TF2 read
        self.fixed_z_m = None

        # Home pose joint angles in degrees (from RViz slider values)
        self.home_joints = [0.0, 14.9, 40.1, 0.0, 34.3, 0.0]

        self.get_logger().info('Control node started! Waiting for arm service...')

        # Check service availability every second until ready
        self.home_timer = self.create_timer(1.0, self.try_home)

    def try_home(self):
        if self.homed:
            self.home_timer.cancel()
            return

        if not self.client.service_is_ready():
            self.get_logger().info('Waiting for arm service...', throttle_duration_sec=3.0)
            return

        self.service_ready = True
        self.get_logger().info('Arm service ready! Sending home position...')

        req = Move.Request()
        req.pose = self.home_joints
        req.has_ref = False
        req.ref_joint = [0.0]
        req.mvvelo = 30.0
        req.mvacc = 30.0
        req.mvtime = 0.0
        req.mvradii = 0.0
        req.coord_mode = 1  # joint mode
        req.index = 0

        self.homed = True
        self.home_timer.cancel()

        future = self.client.call_async(req)
        future.add_done_callback(self.home_callback)

    def home_callback(self, future):
        try:
            response = future.result()
            if response.ret == 0:
                self.get_logger().info('Home position reached ✓ — visual servoing ready')
            else:
                self.get_logger().warn(f'Home failed: {response.message}')
                self.homed = False
        except Exception as e:
            self.get_logger().error(f'Home failed: {e}')
            self.homed = False

    def get_current_tcp(self):
        try:
            transform = self.tf_buffer.lookup_transform('world', 'dummy_tcp', rclpy.time.Time())
            x = transform.transform.translation.x * 1000.0  # meters to mm
            y = transform.transform.translation.y * 1000.0
            z = transform.transform.translation.z * 1000.0
            return x, y, z
        except Exception:
            return None, None, None

    def publish_tcp_marker(self):
        try:
            transform = self.tf_buffer.lookup_transform('world', 'dummy_tcp', rclpy.time.Time())
            marker = Marker()
            marker.header.frame_id = 'world'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'tcp'
            marker.id = 0
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = transform.transform.translation.x
            marker.pose.position.y = transform.transform.translation.y
            marker.pose.position.z = transform.transform.translation.z
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.04
            marker.scale.y = 0.04
            marker.scale.z = 0.04
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            self.marker_pub.publish(marker)
        except Exception:
            pass

    def pixel_callback(self, msg):
        # Don't process if service not ready or waiting for response
        if not self.service_ready or self.waiting_for_response:
            return

        offset_x = msg.x
        offset_y = msg.y

        # Check alignment threshold
        if abs(offset_x) < self.threshold and abs(offset_y) < self.threshold:
            self.get_logger().info('TARGET ALIGNED — arm holding position ✓', throttle_duration_sec=1.0)
            return

        # Read current Joint 6 TCP position from TF2
        cur_x, cur_y, cur_z = self.get_current_tcp()
        if cur_x is None:
            self.get_logger().warn('TF2 not ready', throttle_duration_sec=2.0)
            return

        # Lock Z on first valid read
        if self.fixed_z_m is None:
            self.fixed_z_m = cur_z
            self.get_logger().info(f'Z locked at {self.fixed_z_m:.1f}mm')

        # Convert pixel offset to mm
        move_x = offset_x * self.mm_per_pixel * self.gain
        move_y = offset_y * self.mm_per_pixel * self.gain

        # Clamp to max step
        move_x = max(-self.max_step_mm, min(self.max_step_mm, move_x))
        move_y = max(-self.max_step_mm, min(self.max_step_mm, move_y))

        # Calculate absolute target from current Joint 6 position
        target_x = cur_x + move_x
        target_y = cur_y + move_y
        target_z = self.fixed_z_m

        # Workspace safety check on absolute target
        if target_x < self.x_min or target_x > self.x_max:
            self.get_logger().warn(f'X limit reached! target={target_x:.1f}mm — blocked')
            move_x = 0.0
            target_x = cur_x

        if target_y < self.y_min or target_y > self.y_max:
            self.get_logger().warn(f'Y limit reached! target={target_y:.1f}mm — blocked')
            move_y = 0.0
            target_y = cur_y

        self.get_logger().info(
            f'Offset: x={offset_x:.0f}px y={offset_y:.0f}px → '
            f'TCP: ({cur_x:.1f}, {cur_y:.1f})mm → '
            f'Target: ({target_x:.1f}, {target_y:.1f})mm'
        )

        req = Move.Request()
        req.pose = [target_x, target_y, target_z, self.fixed_rx, self.fixed_ry, self.fixed_rz]
        req.has_ref = False
        req.ref_joint = [0.0]
        req.mvvelo = 50.0
        req.mvacc = 50.0
        req.mvtime = 0.0
        req.mvradii = 0.0
        req.coord_mode = 0  # Cartesian mode — absolute target in mm
        req.index = 0

        self.waiting_for_response = True
        future = self.client.call_async(req)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        self.waiting_for_response = False
        try:
            response = future.result()
            if response.ret != 0:
                self.get_logger().warn(f'Arm error: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
