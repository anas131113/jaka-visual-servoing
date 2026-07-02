# Camera-Based Robot Alignment (Visual Servoing)
### JAKA Mini 2 Robotic Arm — ROS 2 Humble

A real-time visual servoing system that uses a camera to detect a colored object and autonomously moves a JAKA Mini 2 robotic arm to align with it. Built during a summer robotics academy as a portfolio project.

---

## Demo

The system detects a blue object in the camera feed, calculates its pixel offset from the image center, converts that offset to millimeter coordinates, and continuously sends correction commands to the arm until it is aligned with the target.

```
Camera detects object → calculates pixel offset → converts to mm → moves arm → repeat
```

---

## System Architecture

```
┌─────────────────────┐        /target_pixel        ┌──────────────────────┐
│   vision_node.py    │  ──────────────────────────► │   control_node.py    │
│                     │     Point(x, y, z=0)         │                      │
│  - USB/laptop cam   │                              │  - pixel → mm        │
│  - HSV color mask   │                              │  - proportional ctrl │
│  - centroid detect  │                              │  - safety limits     │
│  - offset publish   │                              │  - calls JAKA driver │
└─────────────────────┘                              └──────────────────────┘
                                                               │
                                                               │ /jaka_driver/linear_move
                                                               ▼
                                                    ┌──────────────────────┐
                                                    │   jaka_driver node   │
                                                    │  (JAKARobotics/      │
                                                    │   jaka_ros2)         │
                                                    │  - moves real arm    │
                                                    └──────────────────────┘
```

---

## Features

- Real-time color detection using OpenCV HSV masking (no neural network required)
- Proportional control loop — moves fast when far from target, slows down when close
- Safety limits: maximum step size per command, workspace boundary enforcement
- Horizontal mirror correction for laptop webcams
- Minimum contour area filter to ignore noise
- Fake arm node for full pipeline testing without hardware
- RViz marker visualization of target position

---

## Tech Stack

| Tool | Purpose |
|---|---|
| ROS 2 Humble | Node communication, topics, services |
| Python 3.10 | All node logic |
| OpenCV 4.5 | Camera capture, HSV detection, contours |
| JAKA SDK (jaka_ros2) | Arm driver and Move service |
| RViz2 | Target position visualization |
| Ubuntu 22.04 | Operating system |

---

## Project Structure

```
ros2_ws/
└── src/
    ├── my_robot_pkg/
    │   ├── my_robot_pkg/
    │   │   ├── vision_node.py      ← Node 1: camera detection + offset publishing
    │   │   ├── control_node.py     ← Node 2: pixel→mm conversion + arm control
    │   │   └── fake_arm_node.py    ← Test stub: simulates JAKA driver
    │   ├── setup.py
    │   └── package.xml
    └── jaka_ros2/                  ← Official JAKA ROS 2 driver (submodule)
```

---

## Installation

### Prerequisites

- Ubuntu 22.04
- ROS 2 Humble ([installation guide](https://docs.ros.org/en/humble/Installation.html))
- OpenCV: `pip3 install opencv-python`

### Setup

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone this repo
git clone https://github.com/YOUR_USERNAME/jaka-visual-servoing.git my_robot_pkg

# Clone JAKA driver
git clone https://github.com/JAKARobotics/jaka_ros2.git

# Build (skip jaka_planner — requires unavailable moveit_visual_tools)
cd ~/ros2_ws
colcon build --packages-skip jaka_planner

# Source workspace
source install/setup.bash
```

---

## Running the System

### Option 1 — Full simulation (no arm required)

Open 3 terminals:

```bash
# Terminal 1: fake arm (simulates JAKA driver)
ros2 run my_robot_pkg fake_arm_node

# Terminal 2: control node
ros2 run my_robot_pkg control_node

# Terminal 3: vision node (hold a blue object in front of camera)
ros2 run my_robot_pkg vision_node
```

### Option 2 — Real JAKA arm

```bash
# Terminal 1: real JAKA driver (replace IP with your arm's address)
ros2 launch jaka_driver robot_start.launch.py ip:=192.168.1.100

# Terminal 2: control node
ros2 run my_robot_pkg control_node

# Terminal 3: vision node
ros2 run my_robot_pkg vision_node
```

### Optional — RViz visualization

```bash
# Terminal 4: launch RViz with robot model
ros2 launch jaka_description simple_rviz.launch.py
```

In RViz: set Fixed Frame to `world`, add a Marker display, set topic to `/target_marker`.

---

## Tuning Parameters

All tuning parameters are at the top of `control_node.py`:

```python
self.mm_per_pixel = 0.5      # calibrate with ruler on real arm
self.gain         = 0.4      # proportional gain (higher = faster response)
self.max_step_mm  = 50.0     # max mm per single move command (safety)
self.threshold    = 10       # alignment threshold in pixels
self.fixed_z      = 300.0    # fixed arm height in mm above table

# Workspace safety limits (mm)
self.x_min = -300.0
self.x_max =  300.0
self.y_min = -300.0
self.y_max =  300.0
self.z_min =  200.0          # never go below this height
```

Color detection range is in `vision_node.py`:

```python
self.lower_blue = np.array([100, 150, 50])
self.upper_blue = np.array([140, 255, 255])
```

---

## How It Works

**Node 1 — vision_node.py**

Captures frames from the camera at 10Hz using OpenCV with the V4L2 backend. Converts each frame to HSV color space and applies a color mask to isolate the target object. Finds the largest contour in the mask, calculates its centroid using image moments, and publishes the pixel offset from the image center (320, 240 for 640×480) as a `geometry_msgs/Point` message on `/target_pixel`.

**Node 2 — control_node.py**

Subscribes to `/target_pixel`. Converts the pixel offset to millimeters using a configurable scale factor and proportional gain. Clamps the result to a maximum step size for safety. Checks workspace boundaries before sending any command. If the offset is within the alignment threshold, holds position. Otherwise sends a `jaka_msgs/srv/Move` service request to the JAKA driver.

---

## Known Limitations

- Laptop webcams have a horizontal mirror effect — corrected with `cv2.flip(frame, 1)`
- GStreamer backend fails on some systems — camera opened with `cv2.CAP_V4L2`
- Full Gazebo simulation requires `moveit_visual_tools` which has a build dependency issue — use fake arm node for testing instead
- `mm_per_pixel` scale factor must be calibrated physically on the real arm with a ruler
- Y-axis direction may need to be flipped depending on arm orientation at the academy

---

## Author

Anas Mokdad — Cybersecurity Engineering Student  
Summer Robotics Academy 2025
