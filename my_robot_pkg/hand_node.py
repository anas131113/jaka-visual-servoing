import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
import numpy as np
import mediapipe as mp
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import vision as mp_vision
import warnings
warnings.filterwarnings('ignore')

# Landmark indices
WRIST = 0
THUMB_TIP = 4; THUMB_IP = 3; THUMB_MCP = 2
INDEX_TIP = 8; INDEX_PIP = 6
MIDDLE_TIP = 12; MIDDLE_PIP = 10
RING_TIP = 16; RING_PIP = 14
PINKY_TIP = 20; PINKY_PIP = 18

# Palm center landmarks
PALM_INDICES = [0, 5, 9, 13, 17]

# MediaPipe skeleton connections
MP_CONNECTIONS = [
    (0,1),(1,2),(2,3),(3,4),
    (0,5),(5,6),(6,7),(7,8),
    (0,9),(9,10),(10,11),(11,12),
    (0,13),(13,14),(14,15),(15,16),
    (0,17),(17,18),(18,19),(19,20),
    (5,9),(9,13),(13,17),
]


class HandNode(Node):

    def __init__(self):
        super().__init__('hand_node')

        # Publisher: pixel offset → only sent when fist closed
        self.pixel_pub = self.create_publisher(Point, '/target_pixel', 10)

        # Publisher: camera feed with overlay
        self.image_pub = self.create_publisher(Image, '/camera/image', 10)

        # Subscriber: arm status from control_node
        self.status_sub = self.create_subscription(
            String, '/arm_status', self.status_callback, 10
        )

        # Open camera
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().error('Cannot open camera!')

        # Image center for 640x480
        self.cx = 320
        self.cy = 240

        # MediaPipe hands — legacy API still works in 0.10.x
        self.mp_hands = mp.solutions.hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            model_complexity=0,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5,
        )

        # Arm status overlay
        self.arm_status = ''
        self.arm_status_color = (0, 255, 0)

        # Last detected palm center offset (updated every frame)
        self.last_offset_x = 0.0
        self.last_offset_y = 0.0
        self.hand_detected = False

        # Temporal filter for smooth tracking
        self.filtered_cx = 0.5
        self.filtered_cy = 0.5
        self.alpha = 0.35

        # Run at 10Hz
        self.timer = self.create_timer(0.1, self.process_frame)

        self.get_logger().info('Hand node started!')
        self.get_logger().info('Open hand = track position (arm does NOT move)')
        self.get_logger().info('Close fist = arm moves to that position')

    def status_callback(self, msg):
        self.arm_status = msg.data
        if '✓' in msg.data or 'ENABLED' in msg.data:
            self.arm_status_color = (0, 255, 0)
        elif 'LIMIT' in msg.data or 'SINGULARITY' in msg.data or 'WORKSPACE' in msg.data:
            self.arm_status_color = (0, 0, 255)
        elif 'JOINT' in msg.data:
            self.arm_status_color = (0, 165, 255)
        else:
            self.arm_status_color = (255, 255, 255)

    def is_fist_closed(self, lm, w, h):
        # All 4 fingers curled: TIP.y > PIP.y (tip below middle joint)
        idx_curled    = lm[INDEX_TIP].y  > lm[INDEX_PIP].y
        mid_curled    = lm[MIDDLE_TIP].y > lm[MIDDLE_PIP].y
        ring_curled   = lm[RING_TIP].y   > lm[RING_PIP].y
        pinky_curled  = lm[PINKY_TIP].y  > lm[PINKY_PIP].y
        return idx_curled and mid_curled and ring_curled and pinky_curled

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Cannot read frame')
            return

        # Fix mirror
        frame = cv2.flip(frame, 1)
        h, w = frame.shape[:2]

        # Draw image center crosshair
        cv2.line(frame, (self.cx - 20, self.cy), (self.cx + 20, self.cy), (255, 255, 255), 1)
        cv2.line(frame, (self.cx, self.cy - 20), (self.cx, self.cy + 20), (255, 255, 255), 1)

        # Run MediaPipe
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.mp_hands.process(rgb)

        self.hand_detected = False
        gesture_text = 'No hand detected'
        gesture_color = (0, 0, 255)

        if results.multi_hand_landmarks:
            hand_lm = results.multi_hand_landmarks[0]
            lm = hand_lm.landmark
            self.hand_detected = True

            # Draw skeleton
            pts = [(int(lm[i].x * w), int(lm[i].y * h)) for i in range(21)]
            for (a, b) in MP_CONNECTIONS:
                cv2.line(frame, pts[a], pts[b], (220, 220, 220), 2)
            for i, (px, py) in enumerate(pts):
                r = 7 if i in (4, 8, 12, 16, 20) else 5
                cv2.circle(frame, (px, py), r + 1, (0, 0, 0), -1)
                cv2.circle(frame, (px, py), r, (255, 48, 255), -1)

            # Palm center (average of PALM_INDICES landmarks)
            raw_cx = np.mean([lm[i].x for i in PALM_INDICES])
            raw_cy = np.mean([lm[i].y for i in PALM_INDICES])

            # Temporal filter for smooth tracking
            self.filtered_cx = self.alpha * raw_cx + (1 - self.alpha) * self.filtered_cx
            self.filtered_cy = self.alpha * raw_cy + (1 - self.alpha) * self.filtered_cy

            # Convert to pixel
            palm_px = int(self.filtered_cx * w)
            palm_py = int(self.filtered_cy * h)

            # Offset from image center
            self.last_offset_x = float(palm_px - self.cx)
            self.last_offset_y = float(palm_py - self.cy)

            # Draw palm center
            cv2.circle(frame, (palm_px, palm_py), 10, (0, 255, 255), 2)
            cv2.circle(frame, (palm_px, palm_py), 3, (0, 255, 255), -1)

            # Draw line from image center to palm
            cv2.line(frame, (self.cx, self.cy), (palm_px, palm_py), (0, 255, 255), 1)

            # Check gesture
            fist = self.is_fist_closed(lm, w, h)

            if fist:
                # Fist closed — publish offset to move arm
                gesture_text = 'FIST — ARM MOVING'
                gesture_color = (0, 0, 255)

                msg = Point()
                msg.x = self.last_offset_x
                msg.y = self.last_offset_y
                msg.z = 0.0
                self.pixel_pub.publish(msg)
                self.get_logger().info(
                    f'FIST → sending offset: x={self.last_offset_x:.0f}, y={self.last_offset_y:.0f}'
                )

                # Draw red dot on palm when fist
                cv2.circle(frame, (palm_px, palm_py), 12, (0, 0, 255), -1)

            else:
                # Hand open — track but don't move arm
                gesture_text = f'OPEN — tracking ({self.last_offset_x:.0f}, {self.last_offset_y:.0f})'
                gesture_color = (0, 255, 0)

            # Draw gesture text
            cv2.putText(frame, gesture_text, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, gesture_color, 2)

            # Draw offset text
            cv2.putText(frame,
                        f'offset x={self.last_offset_x:.0f} y={self.last_offset_y:.0f}',
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

        else:
            cv2.putText(frame, gesture_text, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, gesture_color, 2)

        # Arm status overlay at bottom
        if self.arm_status:
            cv2.putText(frame, self.arm_status,
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
        self.mp_hands.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
