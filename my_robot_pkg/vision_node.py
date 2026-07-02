import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import cv2
import numpy as np

class VisionNode(Node):

    def __init__(self):
        super().__init__('vision_node')

        # Publisher: sends pixel offset to /target_pixel
        self.publisher = self.create_publisher(Point, '/target_pixel', 10)

        # Open camera using V4L2 backend (GStreamer fails on this system)
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().error('Cannot open camera!')

        # Image center for 640x480 resolution
        self.image_center_u = 320
        self.image_center_v = 240

        # HSV range for blue color detection
        # Tune these values if detection is unreliable
        self.lower_blue = np.array([100, 150, 50])
        self.upper_blue = np.array([140, 255, 255])

        # Minimum contour area to ignore small noise blobs
        self.min_area = 500

        # Run at 10Hz
        self.timer = self.create_timer(0.1, self.detect_and_publish)

        self.get_logger().info('Vision node started! Looking for blue object...')

    def detect_and_publish(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Cannot read frame from camera')
            return

        # Fix horizontal mirror effect on laptop webcam
        frame = cv2.flip(frame, 1)

        # Convert BGR to HSV for reliable color detection
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Create binary mask — white where blue detected, black elsewhere
        mask = cv2.inRange(hsv, self.lower_blue, self.upper_blue)

        # Find contours of white blobs
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        if len(contours) == 0:
            self.get_logger().info('No object detected', throttle_duration_sec=2.0)
            return

        # Pick largest contour — ignores small noise
        largest = max(contours, key=cv2.contourArea)

        # Ignore if too small
        if cv2.contourArea(largest) < self.min_area:
            self.get_logger().info('Object too small — ignored', throttle_duration_sec=2.0)
            return

        # Calculate centroid using image moments
        M = cv2.moments(largest)
        if M["m00"] == 0:
            return

        u = int(M["m10"] / M["m00"])
        v = int(M["m01"] / M["m00"])

        # Calculate pixel offset from image center
        offset_x = u - self.image_center_u
        offset_y = v - self.image_center_v

        # Publish offset as Point message
        msg = Point()
        msg.x = float(offset_x)
        msg.y = float(offset_y)
        msg.z = 0.0

        self.publisher.publish(msg)
        self.get_logger().info(f'Offset: x={offset_x}, y={offset_y}')

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
