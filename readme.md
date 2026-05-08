**Language / 语言：** [English](readme.md) | [中文](readme_cn.md)

---

# Distributed YOLO-Based Semantic Target Following System

A distributed ROS-based mobile robot target following system using **YOLO semantic detection**, **RGB-D localization**, **TCP bridge communication**, and **robot-side motion control**.

This project was developed as the final project for **AIE1902 Embodied AI**.  
Compared with our mid-term color-based tracking system, this final system supports **semantic target selection**, **multi-class following**, and **real robot deployment**.

---

## 1. Project Overview

This system is designed for semantic object following in real-world environments.  
The user can select a target class such as:

- `person`
- `chair`
- `bottle`
- `backpack`
- `cup`
- `laptop`
- `book`
- `cell phone`

The robot then detects the selected object category, estimates its 3D position using RGB-D data, and follows it autonomously.

Because the robot-side onboard computer was not suitable for stable real-time YOLO inference, we adopted a **distributed deployment**:

- the **local computer / WSL side** runs YOLO detection and the target-selection UI
- the **robot side** performs TCP reception, depth-based localization, TF transformation, and motion control

To improve cross-machine reliability in our real deployment environment, we used a **TCP bridge** instead of native ROS cross-machine detection topic transport.

---

## 2. Repository Structure

```text
.
├── set_ros_robot.sh
├── set_ros_wsl.sh
├── WSL端·虚拟机端·本地电脑端
│   └── yolo_tracking
│       ├── CMakeLists.txt
│       ├── package.xml
│       ├── launch
│       │   └── wsl_all_in_one.launch
│       ├── models
│       │   └── yolo26s.pt
│       ├── msg
│       │   └── YoloDetection.msg
│       ├── scripts
│       │   ├── run_yolo_detector.sh
│       │   └── yolo_detector_sender_ui_node.py
│       ├── urdf
│       │   └── tb3_waffle_rgbd.gazebo.xacro
│       └── worlds
│           └── yolo_tracking_world.world
└── 机器人端
    └── yolo_tracking
        ├── CMakeLists.txt
        ├── package.xml
        ├── launch
        │   └── robot_all_in_one.launch
        ├── msg
        │   └── YoloDetection.msg
        └── scripts
            ├── yolo_detection_bridge_node.py
            └── yolo_node_release.py
```

### Directory Meaning

- **WSL / local computer side**
  - receives RGB image stream from robot
  - runs YOLO inference
  - provides GUI for target category switching
  - sends lightweight detection results to robot via TCP

- **Robot side**
  - runs robot bringup
  - receives detection results through TCP
  - converts JSON into local ROS topic
  - performs depth-based 3D localization
  - executes following control through a state machine

---

## 3. System Architecture

![System Architecture](images/system_architecture.png)

*Figure: Distributed system architecture showing the local computer side, TCP bridge, and robot-side following pipeline.*

The full system can be divided into three layers:

### 3.1 Local Computer Side
- RGB image subscriber
- YOLO semantic detection
- target selection UI
- continuity-based target selection
- JSON sender

### 3.2 TCP Communication Layer
- lightweight JSON transmission
- stable cross-machine bridge

### 3.3 Robot Side
- TCP receiver / bridge
- local ROS topic `/yolo_detection`
- depth image + camera info
- TF transformation
- state-machine following controller
- `/cmd_vel` output

---

## 4. Core Features

- Semantic target detection using YOLO
- User-selectable target category
- Multi-class following support
- RGB-D based 3D localization
- State-machine-based following controller
- Velocity smoothing for stable motion
- Target-loss recovery
- Distributed deployment between local computer and real robot
- TCP bridge for reliable cross-machine detection transfer

---

## 5. How It Works

The full execution pipeline is:

1. The robot publishes RGB images.
2. The local computer subscribes to the RGB stream and runs YOLO detection.
3. The user selects the desired target class in the GUI.
4. The detector keeps only detections of the selected class.
5. If multiple candidates exist, the detector:
   - prefers the one closest to the previous target center
   - falls back to the largest bounding box if continuity becomes unreliable
6. The selected detection result is packed into JSON.
7. The JSON result is sent to the robot via TCP.
8. The robot-side bridge reconstructs the detection into local ROS topic `/yolo_detection`.
9. The follower node combines:
   - `/yolo_detection`
   - depth image
   - camera intrinsics
   - TF
10. The robot estimates the 3D target position in `base_link`.
11. The state-machine controller publishes `/cmd_vel` for following.

---

## 6. Requirements

### 6.1 Robot Side
- Ubuntu 20.04
- ROS Noetic
- RGB-D camera
- working robot bringup
- TCP server access
- package `yolo_tracking` compiled in catkin workspace

### 6.2 Local Computer / WSL Side
- Ubuntu 20.04 or WSL Ubuntu
- ROS Noetic
- Python 3.8 or Python 3.10
- CUDA-capable GPU recommended
- `ultralytics`
- package `yolo_tracking` compiled in catkin workspace

---

## 7. Environment Scripts

Two environment helper scripts are provided at the root of this repository:

- `set_ros_robot.sh`
- `set_ros_wsl.sh`

### 7.1 Robot Side Script
Place this file at:

```bash
~/set_ros_robot.sh
```

### 7.2 WSL Side Script
Place this file at:

```bash
~/set_ros_wsl.sh
```

These scripts set:

- `ROS_MASTER_URI`
- `ROS_IP`
- workspace environment
- and clear problematic `ROS_HOSTNAME`

---

## 8. Running the System

### 8.1 Robot Side

#### Terminal 1: start ROS master
```bash
source ~/set_ros_robot.sh
roscore
```

#### Terminal 2: start robot bringup + bridge + follower
```bash
source ~/set_ros_robot.sh
roslaunch yolo_tracking robot_all_in_one.launch
```

> Note: `robot_all_in_one.launch` includes:
> - robot bringup
> - TCP detection bridge
> - robot-side follower

---

### 8.2 WSL / Local Computer Side

#### Start the local computer all-in-one node
```bash
source ~/set_ros_wsl.sh
roslaunch yolo_tracking wsl_all_in_one.launch
```

If your local GPU / CUDA / Python environment requires Python 3.10, and `roslaunch` is not suitable for that interpreter, you can directly run:

```bash
source ~/set_ros_wsl.sh
/usr/local/bin/python3.10 /home/<your_username>/catkin_ws/src/yolo_tracking/scripts/yolo_detector_sender_ui_node.py
```

This combined node provides:
- YOLO detection
- target switching UI
- TCP sending to robot

---

## 9. Building Python 3.10 from Source on Ubuntu 20.04

If your machine uses a newer NVIDIA GPU (for example, an RTX 50-series GPU) or a newer CUDA / PyTorch / Ultralytics stack, the default Python 3.8 on Ubuntu 20.04 is often not suitable enough. In this case, it is recommended to **keep the system Python unchanged** and install Python 3.10 **from source** as a separate runtime for YOLO-related nodes.

### 9.1 Design Principle

Do **not** modify the system Python binaries:

```bash
/usr/bin/python
/usr/bin/python3
```

Ubuntu 20.04 and ROS Noetic depend on the default system Python.  
The correct approach is to install Python 3.10 into a separate location, for example:

```bash
/opt/python3.10.20
```

Then provide convenient command links:

```bash
/usr/local/bin/python3.10
/usr/local/bin/pip310
```

This ensures that:

- the system Python remains untouched
- ROS Noetic remains unaffected
- YOLO / PyTorch / Ultralytics can use Python 3.10

---

### 9.2 Install Build Dependencies

```bash
sudo apt update
sudo apt install -y \
  build-essential \
  zlib1g-dev \
  libncurses5-dev \
  libgdbm-dev \
  libnss3-dev \
  libssl-dev \
  libreadline-dev \
  libffi-dev \
  libsqlite3-dev \
  wget \
  libbz2-dev \
  liblzma-dev \
  tk-dev \
  uuid-dev \
  libxml2-dev \
  libxmlsec1-dev \
  xz-utils
```

These packages are needed so that the compiled Python includes common modules such as:

- `ssl`
- `sqlite3`
- `bz2`
- `lzma`
- `readline`
- `tkinter`

---

### 9.3 Download the Source Code

Using Python 3.10.20 as an example:

```bash
cd ~/Downloads
wget https://www.python.org/ftp/python/3.10.20/Python-3.10.20.tgz
```

---

### 9.4 Extract the Source Code

```bash
tar -xf Python-3.10.20.tgz
cd Python-3.10.20
```

---

### 9.5 Configure the Build

```bash
./configure --enable-optimizations --prefix=/opt/python3.10.20
```

Explanation:

- `--enable-optimizations`: enables optimized build
- `--prefix=/opt/python3.10.20`: installs Python into an isolated directory instead of replacing the system Python

---

### 9.6 Compile Python

```bash
make -j"$(nproc)"
```

---

### 9.7 Install to the Target Directory

```bash
sudo make install
```

After installation, Python and pip will be located at:

```bash
/opt/python3.10.20/bin/python3.10
/opt/python3.10.20/bin/pip3.10
```

---

### 9.8 Create Convenient Command Links

To make the new Python easier to use, create symbolic links:

```bash
sudo ln -sf /opt/python3.10.20/bin/python3.10 /usr/local/bin/python3.10
sudo ln -sf /opt/python3.10.20/bin/pip3.10 /usr/local/bin/pip310
```

After that, you can use:

```bash
python3.10
pip310
```

---

### 9.9 Verify the Installation

```bash
python3.10 --version
pip310 --version
```

Expected output should be similar to:

```text
Python 3.10.20
```

---

### 9.10 Do Not Replace the System Python

Do **not** run commands like:

```bash
sudo ln -sf /opt/python3.10.20/bin/python3 /usr/bin/python3
```

And do **not** use `update-alternatives` to replace the default `python3`.

Reasons:

- Ubuntu system tools depend on the default Python
- ROS Noetic depends on Python 3.8 on Ubuntu 20.04
- replacing the system Python may break either the OS or the ROS environment

---

### 9.11 Notes About pip Installation Path

If `/opt/python3.10.20` is not writable for the current user, `pip310 install ...` may show:

```text
Defaulting to user installation because normal site-packages is not writeable
```

In that case, packages are installed into:

```bash
~/.local/lib/python3.10/site-packages
```

If you want Python 3.10 packages to be installed under `/opt/python3.10.20` for easier management, run:

```bash
sudo chown -R $USER:$USER /opt/python3.10.20
```

Then install packages again with:

```bash
pip310 install ...
```

They will usually go into:

```bash
/opt/python3.10.20/lib/python3.10/site-packages
```

---

### 9.12 How to Use It After Installation

For YOLO / PyTorch / Ultralytics-related dependencies, always use:

```bash
python3.10
pip310
```

For example:

```bash
pip310 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu128
pip310 install ultralytics rospkg
```

If a ROS node script must explicitly use Python 3.10, set its shebang to:

```python
#!/usr/local/bin/python3.10
```

This ensures that the detection node runs with Python 3.10 while keeping the ROS Noetic system Python environment intact.

---

## 10. Notes on Communication Design

Originally, we tried native distributed ROS topic communication between the local detector and the robot.  
In practice, topic discovery worked, but the actual delivery of detection results was not stable enough in our WSL–robot deployment environment.

Therefore, we replaced this cross-machine semantic detection channel with a **TCP bridge**:

- local computer actively sends lightweight JSON detection results
- robot receives them through a fixed TCP port
- robot converts them back into local ROS topic `/yolo_detection`

This design keeps:
- heavy visual perception on the external computer
- depth localization and motion control safely local to the robot

---

## 11. Supported Categories

Preset categories in the local UI include:

- person
- chair
- bottle
- backpack
- cup
- laptop
- book
- cell phone

In our experiments, the most stable categories were:

- person
- chair
- bottle

---

## 12. Demo / Visualization

### YOLO Detection Result
![YOLO Detection Result](images/detection_result.png)

### Robot Following Trajectory
![Robot Following Trajectory](images/trajectory.png)

### Depth-Based Localization
![Depth Localization](images/depth_localization.png)

---

## 13. Main Algorithms

### 13.1 Target Selection
The detector uses:
- **minimum movement selection**
- **largest bounding box fallback**

### 13.2 Depth Localization
The robot uses:
- target center from bounding box
- small ROI around center
- median depth filtering
- pixel-to-camera projection
- TF transformation into `base_link`

### 13.3 Following Control
The robot-side follower uses a four-state controller:
- SEARCHING
- ALIGNING
- APPROACHING
- FOLLOWING

It also includes:
- velocity smoothing
- target-loss recovery
- direct following of the currently selected detected class

---
## 14. Project Context

This repository was developed for the **AIE1902 Embodied AI Final Project**.

Compared with our mid-term project:
- the mid-term system used HSV-based color tracking
- the final system upgrades to YOLO-based semantic target following
- the final system also introduces:
  - distributed deployment
  - TCP bridge communication
  - RGB-D localization
  - UI-based target category switching
  - real robot following

So this project is not only a perception demo, but a complete perception-to-action closed-loop robotic system.

---

## 15. Known Limitations

- Small objects are more sensitive to depth noise and may be harder to follow at long distance.
- The final closed-loop system focuses on target following and does not yet integrate obstacle avoidance.
- The distributed design still depends on stable network connectivity between the local computer and the robot.
- Some categories supported by YOLO may not be equally stable in every real scene.

---

## 16. Future Work

Possible future improvements include:

- integrating LiDAR-based obstacle avoidance
- multi-target tracking and re-identification
- motion prediction for dynamic targets
- adaptive depth ROI selection
- richer web-based or mobile-based UI
- tighter integration with navigation stack

---

## 17. License / Academic Use Note

This repository is intended for course project use, academic demonstration, and learning purposes.

Please cite or acknowledge the original authors if you reuse large parts of this project in coursework or demonstrations.

---

## 18. Acknowledgements

We would like to thank:
- the AIE1902 course instructors and teaching assistants
- ROS and OpenCV communities
- Ultralytics YOLO project
- TurtleBot3 / Spark robot related open-source resources

---

https://github.com/user-attachments/assets/af45262d-41a3-413a-be2c-85ba810a5453

