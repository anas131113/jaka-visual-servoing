import rclpy
from rclpy.node import Node
from jaka_msgs.srv import Move
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
import ikpy.chain
import numpy as np
import warnings
warnings.filterwarnings('ignore')

URDF_PATH = '/home/anas/ros2_ws/src/jaka_description/urdf/jaka_minicobo.urdf'

# Home joint angles in radians
HOME_JOINTS = [0.000, 0.260, 0.700, 0.000, 0.599, 0.000]

class SimArmNode(Node):

    def __init__(self):
        super().__init__('sim_arm_node')

        # Load IK chain
        self.chain = ikpy.chain.Chain.from_urdf_file(
            URDF_PATH,
            base_elements=['world'],
            active_links_mask=[False, False, True, True, True, True, True, True, False]
        )
        self.get_logger().info('IK chain loaded from URDF')

        # Service server
        self.srv = self.create_service(
            Move, '/jaka_driver/linear_move', self.handle_move_request
        )

        # Joint state publisher
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Current joint angles — start at home
        self.current_joints = HOME_JOINTS.copy()

        # Fixed X — locked after first move
        self.fixed_x = None

        # Home target position — computed once from FK
        self.home_target = self._compute_home_fk()

        # Publish at 20Hz
        self.timer = self.create_timer(0.05, self.publish_joint_states)

        self.get_logger().info('Sim arm node ready')

    def _compute_home_fk(self):
        """Compute the FK of home position so we know the starting TCP."""
        full_angles = [0.0, 0.0] + HOME_JOINTS + [0.0]
        fk = self.chain.forward_kinematics(full_angles)
        x, y, z = fk[0, 3], fk[1, 3], fk[2, 3]
        self.get_logger().info(f'Home FK: x={x:.3f} y={y:.3f} z={z:.3f}')
        return x, y, z

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        msg.position = self.current_joints
        self.joint_pub.publish(msg)

    def get_current_tcp(self):
        try:
            t = self.tf_buffer.lookup_transform('world', 'dummy_tcp', rclpy.time.Time())
            return t.transform.translation.x, t.transform.translation.y, t.transform.translation.z
        except Exception:
            return None, None, None

    def handle_move_request(self, request, response):
        pose = request.pose
        coord_mode = request.coord_mode

        # Joint mode — homing
        if coord_mode == 1:
            self.current_joints = [np.radians(a) for a in pose[:6]]
            self.fixed_x = None  # reset fixed X on home
            self.home_target = self._compute_home_fk()
            self.get_logger().info(f'Home → joints: {[f"{np.degrees(j):.1f}" for j in self.current_joints]}°')
            response.ret = 0
            response.message = 'success'
            return response

        # Cartesian mode
        # pose[0] = move_y in mm, pose[1] = move_z in mm
        move_y = pose[0] / 1000.0
        move_z = pose[1] / 1000.0

        # Read current TCP
        cur_x, cur_y, cur_z = self.get_current_tcp()
        if cur_x is None:
            self.get_logger().warn('TF2 not ready')
            response.ret = -1
            response.message = 'TF2 not ready'
            return response

        # Lock X on first move
        if self.fixed_x is None:
            self.fixed_x = cur_x
            self.get_logger().info(f'X locked at {self.fixed_x:.3f}m')

        # New absolute target
        target_x = self.fixed_x
        target_y = cur_y + move_y
        target_z = cur_z + move_z

        self.get_logger().info(
            f'TCP: ({cur_x:.3f}, {cur_y:.3f}, {cur_z:.3f}) '
            f'→ target: ({target_x:.3f}, {target_y:.3f}, {target_z:.3f})'
        )

        # Use current joints as initial guess — smoother transitions
        initial_guess = [0.0, 0.0] + self.current_joints + [0.0]

        try:
            angles = self.chain.inverse_kinematics(
                [target_x, target_y, target_z],
                initial_position=initial_guess
            )
            new_joints = [angles[2], angles[3], angles[4], angles[5], angles[6], angles[7]]

            # Sanity check — reject solutions with extreme joint changes
            max_joint_change = max(
                abs(new_joints[i] - self.current_joints[i]) for i in range(6)
            )
            if max_joint_change > np.radians(120):
                self.get_logger().warn(
                    f'IK rejected — joint change too large: {np.degrees(max_joint_change):.1f}°'
                )
                response.ret = -1
                response.message = 'singularity: joint change too large'
                return response

            self.current_joints = new_joints
            self.get_logger().info(
                f'IK → joints: {[f"{np.degrees(j):.1f}" for j in new_joints]}°'
            )
            response.ret = 0
            response.message = 'success'

        except Exception as e:
            self.get_logger().error(f'IK failed — possible singularity: {e}')
            response.ret = -1
            response.message = f'singularity: {str(e)}'

        return response


def main(args=None):
    rclpy.init(args=args)
    node = SimArmNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
