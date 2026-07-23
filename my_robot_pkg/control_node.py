import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from jaka_msgs.srv import Move
from visualization_msgs.msg import Marker
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


class ControlNode(Node):

    def __init__(self):
        super().__init__('control_node')

        self.subscription = self.create_subscription(
            Point, '/target_pixel', self.pixel_callback, 10
        )

        self.client = self.create_client(Move, '/jaka_driver/linear_move')
        self.marker_pub = self.create_publisher(Marker, '/tcp_marker', 10)

        # Status publisher — sends warnings to vision_node for overlay
        self.status_pub = self.create_publisher(String, '/arm_status', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.marker_timer = self.create_timer(0.1, self.publish_tcp_marker)

        # ── TUNING PARAMETERS ──────────────────────────────────────
        self.mm_per_pixel = 0.5
        self.gain = 0.4
        self.max_step_mm = 50.0
        self.threshold = 10
        self.fixed_rx = 3.14
        self.fixed_ry = 0.0
        self.fixed_rz = 0.0

        # ── WORKSPACE SAFETY LIMITS (mm) ───────────────────────────
        self.y_min = -400.0
        self.y_max =  400.0
        self.z_min =  100.0
        self.z_max =  800.0

        # State
        self.waiting_for_response = False
        self.service_ready = False
        self.homed = False
        self.servoing_enabled = False

        self.home_joints = [0.0, 14.9, 40.1, 0.0, 34.3, 0.0]

        self.get_logger().info('Control node started! Waiting for arm service...')
        self.home_timer = self.create_timer(1.0, self.try_home)

    def publish_status(self, message, level='info'):
        msg = String()
        msg.data = message
        self.status_pub.publish(msg)
        if level == 'info':
            self.get_logger().info(message)
        elif level == 'warn':
            self.get_logger().warn(message)
        elif level == 'error':
            self.get_logger().error(message)

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
        req.coord_mode = 1
        req.index = 0

        self.homed = True
        self.home_timer.cancel()

        future = self.client.call_async(req)
        future.add_done_callback(self.home_callback)

    def home_callback(self, future):
        try:
            response = future.result()
            if response.ret == 0:
                self.get_logger().info('Home reached ✓ — waiting 3s before visual servoing...')
                self.create_timer(3.0, self.enable_servoing)
            else:
                self.get_logger().warn(f'Home failed: {response.message}')
                self.homed = False
        except Exception as e:
            self.get_logger().error(f'Home failed: {e}')
            self.homed = False

    def enable_servoing(self):
        self.servoing_enabled = True
        self.publish_status('Visual servoing ENABLED ✓', 'info')

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
        if not self.service_ready or not self.servoing_enabled:
            return
        if self.waiting_for_response:
            return

        offset_x = msg.x
        offset_y = msg.y

        if abs(offset_x) < self.threshold and abs(offset_y) < self.threshold:
            self.publish_status('TARGET ALIGNED ✓', 'info')
            return

        move_y = -offset_x * self.mm_per_pixel * self.gain
        move_z = -offset_y * self.mm_per_pixel * self.gain

        move_y = max(-self.max_step_mm, min(self.max_step_mm, move_y))
        move_z = max(-self.max_step_mm, min(self.max_step_mm, move_z))

        try:
            transform = self.tf_buffer.lookup_transform('world', 'dummy_tcp', rclpy.time.Time())
            cur_y = transform.transform.translation.y * 1000.0
            cur_z = transform.transform.translation.z * 1000.0
        except Exception:
            self.get_logger().warn('TF2 not ready', throttle_duration_sec=2.0)
            return

        target_y = cur_y + move_y
        target_z = cur_z + move_z

        # Workspace limit checks
        blocked = False

        if target_y < self.y_min or target_y > self.y_max:
            self.publish_status(
                f'Y LIMIT REACHED — cannot move further {"right" if target_y > self.y_max else "left"}',
                'warn'
            )
            move_y = 0.0
            target_y = cur_y
            blocked = True

        if target_z < self.z_min or target_z > self.z_max:
            self.publish_status(
                f'Z LIMIT REACHED — cannot move further {"up" if target_z > self.z_max else "down"}',
                'warn'
            )
            move_z = 0.0
            target_z = cur_z
            blocked = True

        if blocked and move_y == 0.0 and move_z == 0.0:
            return

        self.get_logger().info(
            f'offset=({offset_x:.0f}px, {offset_y:.0f}px) → '
            f'move=(y={move_y:.1f}mm, z={move_z:.1f}mm) → '
            f'target=(y={target_y:.1f}mm, z={target_z:.1f}mm)'
        )

        req = Move.Request()
        req.pose = [move_y, move_z, 0.0, self.fixed_rx, self.fixed_ry, self.fixed_rz]
        req.has_ref = False
        req.ref_joint = [0.0]
        req.mvvelo = 50.0
        req.mvacc = 50.0
        req.mvtime = 0.0
        req.mvradii = 0.0
        req.coord_mode = 0
        req.index = 0

        self.waiting_for_response = True
        future = self.client.call_async(req)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        self.waiting_for_response = False
        try:
            response = future.result()
            if response.ret == -1:
                # Check if this was an IK failure from sim_arm_node
                if 'singularity' in response.message.lower() or 'ik' in response.message.lower():
                    self.publish_status('SINGULARITY DETECTED — move blocked', 'error')
                else:
                    self.publish_status(
                        f'WORKSPACE LIMIT — target unreachable: {response.message}',
                        'warn'
                    )
            elif response.ret != 0:
                self.publish_status(f'Arm error: {response.message}', 'warn')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
