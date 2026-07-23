# Camera-Based Robot Alignment — Visual Servoing
### JAKA Mini 2 Robotic Arm | ROS 2 Humble | Python | OpenCV | MediaPipe

A real-time visual servoing system that uses a camera to detect either a **colored object** or a **hand gesture** and autonomously moves a JAKA Mini 2 robotic arm to align with it. Built during a summer robotics academy as a portfolio project — starting from zero ROS 2 experience.

---

## How It Works

```
Camera (laptop/USB)
       ↓
[vision_node OR hand_node]
detects target → calculates pixel offset from image center
       ↓  /target_pixel topic
[control_node]
reads Joint 6 (dummy_tcp) position from TF2
converts pixel offset → mm movement
sends absolute Cartesian target to arm
       ↓  /jaka_driver/linear_move service
[sim_arm_node (simulation) OR jaka_driver (real arm)]
runs inverse kinematics → moves arm
       ↓
RViz shows arm moving in real time
```

---

## Features

- **Two detection modes:**
  - `vision_node` — HSV color masking detects a blue object
  - `hand_node` — MediaPipe detects hand; open hand tracks position, closed fist triggers arm movement
- **Real IK simulation** using `ikpy` + JAKA URDF — arm moves correctly in RViz before touching real hardware
- **TCP marker** — red sphere always locked to Joint 6 (dummy_tcp) via TF2
- **Auto home position** on startup — arm moves to safe starting pose automatically
- **3-second delay** after homing before visual servoing begins
- **Axis mapping** — camera X → arm Y (left/right along wall), camera Y → arm Z (up/down along wall), arm X (depth) fixed
- **Safety limits** — workspace boundaries, max step per command, singularity detection
- **Status overlay** — warnings appear both in terminal and on live camera feed:
  - `TARGET ALIGNED ✓`
  - `Y LIMIT REACHED`
  - `Z LIMIT REACHED`
  - `SINGULARITY DETECTED`
  - `WORKSPACE LIMIT — target unreachable`
- **Live camera feed** viewable in `rqt_image_view`

---

## Tech Stack

| Tool | Purpose |
|---|---|
| ROS 2 Humble | Node communication, topics, services |
| Python 3.10 | All node logic |
| OpenCV 4.5 | Camera capture, HSV detection |
| MediaPipe 0.10.9 | Hand detection and gesture recognition |
| ikpy 4.0 | Inverse kinematics from JAKA URDF |
| TF2 | Real-time coordinate frame transforms |
| JAKA SDK (jaka_ros2) | Arm driver and Move service |
| RViz2 | 3D arm visualization |
| rqt_image_view | Live camera feed viewer |
| Ubuntu 22.04 + ROS 2 Humble | OS |

---

## Project Structure

```
ros2_ws/src/
├── my_robot_pkg/
│   ├── my_robot_pkg/
│   │   ├── vision_node.py      ← Node 1A: blue object detection
│   │   ├── hand_node.py        ← Node 1B: hand/gesture detection
│   │   ├── control_node.py     ← Node 2: pixel→mm + arm control + safety
│   │   ├── sim_arm_node.py     ← Simulation: IK + joint state publisher
│   │   └── fake_arm_node.py    ← Simple test stub (no IK)
│   ├── setup.py
│   └── package.xml
├── jaka_driver/                ← Official JAKA ROS 2 driver
├── jaka_msgs/                  ← JAKA message/service types
└── jaka_description/           ← JAKA URDF + RViz launch files
```

---

## Nodes Explained

### `vision_node.py`
Captures camera frames at 10Hz using OpenCV (V4L2 backend). Converts to HSV, applies blue color mask, finds largest contour, calculates centroid. Publishes pixel offset from image center to `/target_pixel`. Also publishes annotated camera feed to `/camera/image`.

### `hand_node.py`
Uses MediaPipe Hands to detect 21 hand landmarks. Calculates palm center from wrist + knuckle landmarks. When hand is **open** — shows tracking info on screen but does NOT publish to `/target_pixel`. When hand is **closed (fist)** — publishes current palm offset to `/target_pixel`, triggering arm movement. Temporal filter smooths tracking.

### `control_node.py`
Subscribes to `/target_pixel`. On startup, sends home pose (joint mode). After 3-second delay, enables visual servoing. For each offset received: reads current Joint 6 TCP position from TF2, converts pixel offset to mm using proportional gain, checks workspace limits, sends absolute Cartesian target to arm service.

### `sim_arm_node.py`
Exposes the same `/jaka_driver/linear_move` service as the real JAKA driver. For joint mode commands (homing): sets joint angles directly. For Cartesian commands: reads current TCP from TF2, calculates new absolute target, runs ikpy IK to find joint angles, validates solution (rejects if any joint changes >120°), publishes joint states to `/joint_states` so RViz updates.

---

## Installation

### Prerequisites
- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10

### Install dependencies
```bash
pip3 install ikpy mediapipe==0.10.9 "numpy<2"
sudo apt install ros-humble-joint-state-publisher-gui
```

### Setup workspace
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone this repo
git clone https://github.com/anas131113/jaka-visual-servoing.git my_robot_pkg

# Clone JAKA driver packages (only what we need)
git clone https://github.com/JAKARobotics/jaka_ros2.git
cp -r jaka_ros2/src/jaka_driver .
cp -r jaka_ros2/src/jaka_msgs .
cp -r jaka_ros2/src/jaka_description .
rm -rf jaka_ros2

# Build
cd ~/ros2_ws
colcon build --packages-skip jaka_planner
source install/setup.bash
```

---

## Running the System

### Simulation (no real arm needed)

```bash
# Terminal 1 — RViz visualization
ros2 launch jaka_description simple_rviz.launch.py

# Terminal 2 — IK simulation node (replaces real arm)
ros2 run my_robot_pkg sim_arm_node

# Terminal 3 — control node
ros2 run my_robot_pkg control_node

# Terminal 4A — blue object detection
ros2 run my_robot_pkg vision_node
# OR
# Terminal 4B — hand gesture detection
ros2 run my_robot_pkg hand_node

# Terminal 5 — live camera feed
ros2 run rqt_image_view rqt_image_view
# Select /camera/image topic
```

In RViz: set Fixed Frame = `world`, add `RobotModel`, add `Marker` with topic `/tcp_marker`.

### Real JAKA arm (academy)

```bash
# Replace sim_arm_node with real driver:
ros2 launch jaka_driver robot_start.launch.py ip:=<ARM_IP>

# Everything else stays identical
```

---

## Tuning Parameters

All parameters are at the top of `control_node.py`:

```python
self.mm_per_pixel = 0.5      # calibrate with ruler on real arm
self.gain         = 0.4      # proportional gain
self.max_step_mm  = 50.0     # max mm per command (safety)
self.threshold    = 10       # alignment threshold in pixels
self.fixed_rx     = 3.14     # wrist orientation (keep fixed)

# Workspace safety limits (mm)
self.y_min = -400.0
self.y_max =  400.0
self.z_min =  100.0
self.z_max =  800.0
```

Speed (mm/s) — reduce for real arm safety:
```python
req.mvvelo = 50.0   # change to 10.0 or 20.0 for real arm
```

Home pose joint angles in degrees (update after testing on real arm):
```python
self.home_joints = [0.0, 14.9, 40.1, 0.0, 34.3, 0.0]
```

Color detection HSV range in `vision_node.py`:
```python
self.lower_blue = np.array([100, 150, 50])
self.upper_blue = np.array([140, 255, 255])
```

---

## Coordinate System

```
Arm at home position (facing wall):
  X axis = depth (toward wall) → FIXED, never changes
  Y axis = left/right along wall → controlled by camera X offset
  Z axis = up/down along wall → controlled by camera Y offset

Camera pixel mapping:
  object moves RIGHT → offset_x positive → arm moves RIGHT (Y+)
  object moves LEFT  → offset_x negative → arm moves LEFT  (Y-)
  object moves UP    → offset_y negative → arm moves UP    (Z+)
  object moves DOWN  → offset_y positive → arm moves DOWN  (Z-)
```

---

## Academy Checklist

When you arrive at the academy with the real arm:

1. Find the arm IP address from the JAKA controller
2. Launch real driver: `ros2 launch jaka_driver robot_start.launch.py ip:=<IP>`
3. Physically move arm to home position using teach pendant
4. Read real joint angles from JAKA software
5. Update `self.home_joints` in `control_node.py`
6. Set `req.mvvelo = 10.0` for safe speed
7. Calibrate `mm_per_pixel` using a ruler
8. Verify Y axis direction (flip sign if needed)
9. Verify Z axis direction (flip sign if needed)
10. Record demo video

---

## Author

Anas Mokdad — Cybersecurity Engineering Student
Summer Robotics Academy 2025
