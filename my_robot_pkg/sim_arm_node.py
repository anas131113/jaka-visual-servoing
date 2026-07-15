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

class SimArmNode(Node):

    def __init__(self):
        super().__init__('sim_arm_node')

        self.chain = ikpy.chain.Chain.from_urdf_file(
            URDF_PATH,
            base_elements=['world'],
            active_links_mask=[False, False, True, True, True, True, True, True, False]
        )
        self.get_logger().info('IK chain loaded from URDF')

        self.srv = self.create_service(
            Move,
            '/jaka_driver/linear_move',
            self.handle_move_request
        )

        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.current_joints = [0.000, 0.260, 0.700, 0.000, 0.599, 0.000]

        self.fixed_z = None

        self.timer = self.create_timer(0.05, self.publish_joint_states)

        self.get_logger().info('Sim arm node ready — arm visible in RViz')

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        msg.position = self.current_joints
        self.joint_pub.publish(msg)

    def get_current_tcp(self):
        try:
            transform = self.tf_buffer.lookup_transform('world', 'dummy_tcp', rclpy.time.Time())
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z
            return x, y, z
        except Exception:
            return None, None, None

    def handle_move_request(self, request, response):
        pose = request.pose
        coord_mode = request.coord_mode

        if coord_mode == 1:
            self.current_joints = [np.radians(a) for a in pose[:6]]
            self.get_logger().info(f'Joint move → joints: {[f"{np.degrees(j):.1f}" for j in self.current_joints]}°')
            response.ret = 0
            response.message = 'success'
            return response

        move_x = pose[0] / 1000.0
        move_y = pose[1] / 1000.0

        cur_x, cur_y, cur_z = self.get_current_tcp()

        if cur_x is None:
            self.get_logger().warn('TF2 not ready yet — skipping move')
            response.ret = -1
            response.message = 'TF2 not ready'
            return response

        if self.fixed_z is None:
            self.fixed_z = cur_z

        target_x = cur_x + move_x
        target_y = cur_y + move_y
        target_z = self.fixed_z

        self.get_logger().info(f'Move: current=({cur_x:.3f}, {cur_y:.3f}, {cur_z:.3f}) offset=({move_x:.3f}, {move_y:.3f}) target=({target_x:.3f}, {target_y:.3f}, {target_z:.3f})')

        target = [target_x, target_y, target_z]
        initial_guess = [0.0, 0.0] + self.current_joints + [0.0]

        try:
            angles = self.chain.inverse_kinematics(target, initial_position=initial_guess)
            new_joints = [angles[2], angles[3], angles[4], angles[5], angles[6], angles[7]]
            self.current_joints = new_joints
            self.get_logger().info(f'IK solved → joints: {[f"{np.degrees(j):.1f}" for j in new_joints]}°')
            response.ret = 0
            response.message = 'success'
        except Exception as e:
            self.get_logger().error(f'IK failed: {e}')
            response.ret = -1
            response.message = str(e)

        return response


def main(args=None):
    rclpy.init(args=args)
    node = SimArmNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
