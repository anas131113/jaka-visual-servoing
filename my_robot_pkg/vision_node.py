import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
import numpy as np


class VisionNode(Node):

    def __init__(self):
        super().__init__('vision_node')

        # Publisher: pixel offset
        self.publisher = self.create_publisher(Point, '/target_pixel', 10)

        # Publisher: camera feed with overlay
        self.image_pub = self.create_publisher(Image, '/camera/image', 10)

        # Subscriber: arm status messages from control_node
        self.status_sub = self.create_subscription(
            String, '/arm_status', self.status_callback, 10
        )

        # Open camera
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().error('Cannot open camera!')

        # Image center
        self.image_center_u = 320
        self.image_center_v = 240

        # HSV range for blue detection
        self.lower_blue = np.array([100, 150, 50])
        self.upper_blue = np.array([140, 255, 255])

        # Minimum contour area
        self.min_area = 500

        # Current arm status message to overlay
        self.arm_status = ''
        self.arm_status_color = (0, 255, 0)  # green by default

        # Run at 10Hz
        self.timer = self.create_timer(0.1, self.detect_and_publish)

        self.get_logger().info('Vision node started!')
        self.get_logger().info('Run: ros2 run rqt_image_view rqt_image_view')
        self.get_logger().info('Select /camera/image to see live feed')

    def status_callback(self, msg):
        self.arm_status = msg.data

        # Set color based on message type
        if '✓' in msg.data or 'ENABLED' in msg.data:
            self.arm_status_color = (0, 255, 0)       # green
        elif 'LIMIT' in msg.data or 'SINGULARITY' in msg.data or 'WORKSPACE' in msg.data:
            self.arm_status_color = (0, 0, 255)        # red
        elif 'JOINT' in msg.data:
            self.arm_status_color = (0, 165, 255)      # orange
        else:
            self.arm_status_color = (255, 255, 255)    # white

    def detect_and_publish(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Cannot read frame')
            return

        # Fix mirror
        frame = cv2.flip(frame, 1)

        # Draw image center crosshair
        cv2.line(frame,
                 (self.image_center_u - 20, self.image_center_v),
                 (self.image_center_u + 20, self.image_center_v),
                 (255, 255, 255), 1)
        cv2.line(frame,
                 (self.image_center_u, self.image_center_v - 20),
                 (self.image_center_u, self.image_center_v + 20),
                 (255, 255, 255), 1)

        # Color detection
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_blue, self.upper_blue)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detected = False

        if len(contours) > 0:
            largest = max(contours, key=cv2.contourArea)

            if cv2.contourArea(largest) >= self.min_area:
                M = cv2.moments(largest)
                if M["m00"] != 0:
                    u = int(M["m10"] / M["m00"])
                    v = int(M["m01"] / M["m00"])

                    offset_x = u - self.image_center_u
                    offset_y = v - self.image_center_v

                    detected = True

                    # Draw green contour
                    cv2.drawContours(frame, [largest], -1, (0, 255, 0), 2)

                    # Draw red dot at centroid
                    cv2.circle(frame, (u, v), 8, (0, 0, 255), -1)

                    # Draw line from center to object
                    cv2.line(frame,
                             (self.image_center_u, self.image_center_v),
                             (u, v), (0, 255, 255), 1)

                    # Draw offset text
                    cv2.putText(frame,
                                f'offset x={offset_x} y={offset_y}',
                                (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                    # Publish offset
                    msg = Point()
                    msg.x = float(offset_x)
                    msg.y = float(offset_y)
                    msg.z = 0.0
                    self.publisher.publish(msg)
                    self.get_logger().info(f'Offset: x={offset_x}, y={offset_y}')

        if not detected:
            cv2.putText(frame,
                        'No object detected',
                        (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            self.get_logger().info('No object detected', throttle_duration_sec=2.0)

        # Overlay arm status message at bottom of frame
        if self.arm_status:
            cv2.putText(frame,
                        self.arm_status,
                        (10, frame.shape[0] - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        self.arm_status_color, 2)

        # Publish frame
        self.publish_image(frame)

    def publish_image(self, frame):
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        msg.height = frame.shape[0]
        msg.width = frame.shape[1]
        msg.encoding = 'bgr8'
        msg.is_bigendian = False
        msg.step = frame.shape[1] * 3
        msg.data = frame.tobytes()
        self.image_pub.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
