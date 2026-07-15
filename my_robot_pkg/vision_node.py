import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
import cv2
import numpy as np

class VisionNode(Node):

    def __init__(self):
        super().__init__('vision_node')

        # Publisher: sends pixel offset to /target_pixel
        self.publisher = self.create_publisher(Point, '/target_pixel', 10)

        # Publisher: sends camera feed with overlay to /camera/image
        self.image_pub = self.create_publisher(Image, '/camera/image', 10)

        # Open camera using V4L2 backend
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().error('Cannot open camera!')

        # Image center for 640x480
        self.image_center_u = 320
        self.image_center_v = 240

        # HSV range for blue detection
        self.lower_blue = np.array([100, 150, 50])
        self.upper_blue = np.array([140, 255, 255])

        # Minimum contour area to ignore noise
        self.min_area = 500

        # Run at 10Hz
        self.timer = self.create_timer(0.1, self.detect_and_publish)

        self.get_logger().info('Vision node started!')
        self.get_logger().info('Run in another terminal: ros2 run rqt_image_view rqt_image_view')
        self.get_logger().info('Then select /camera/image topic to see live feed')

    def detect_and_publish(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Cannot read frame from camera')
            return

        # Fix horizontal mirror
        frame = cv2.flip(frame, 1)

        # Draw image center crosshair
        cv2.line(frame, (self.image_center_u - 20, self.image_center_v),
                        (self.image_center_u + 20, self.image_center_v), (255, 255, 255), 1)
        cv2.line(frame, (self.image_center_u, self.image_center_v - 20),
                        (self.image_center_u, self.image_center_v + 20), (255, 255, 255), 1)

        # Convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_blue, self.upper_blue)

        # Find contours
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        offset_x = 0
        offset_y = 0
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

                    # Draw green contour around detected object
                    cv2.drawContours(frame, [largest], -1, (0, 255, 0), 2)

                    # Draw red dot at centroid
                    cv2.circle(frame, (u, v), 8, (0, 0, 255), -1)

                    # Draw line from center to object
                    cv2.line(frame, (self.image_center_u, self.image_center_v),
                                    (u, v), (0, 255, 255), 1)

                    # Draw offset text
                    cv2.putText(frame, f'offset x={offset_x} y={offset_y}',
                                (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                                0.7, (0, 255, 0), 2)

                    # Publish offset
                    msg = Point()
                    msg.x = float(offset_x)
                    msg.y = float(offset_y)
                    msg.z = 0.0
                    self.publisher.publish(msg)
                    self.get_logger().info(f'Offset: x={offset_x}, y={offset_y}')

        if not detected:
            cv2.putText(frame, 'No object detected',
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 0, 255), 2)
            self.get_logger().info('No object detected', throttle_duration_sec=2.0)

        # Publish frame as ROS 2 image message
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
