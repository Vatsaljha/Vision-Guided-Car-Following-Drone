# 🚗🎬 Car-Following Drone Simulation — ROS 2 + Gazebo Classic

A ROS 2 Humble and Gazebo Classic simulation featuring a Prius-style car and a drone with **two distinct operating modes**: a **ground-truth cinematic-shot mode** for camera/drone positioning, and a **fixed-camera visual-following mode** that detects roof-mounted ArUco marker ID 7 from the drone's image stream.

> **Important distinction:** Ground-truth cinematic mode uses simulator state to position/aim the drone for cinematic shots. Fixed-camera visual mode detects the marker in camera images and follows its position in the image. They are different approaches and should not be described as two names for the same controller.

<p align="left">
  <img src="https://img.shields.io/badge/ROS_2-Humble-22314E?logo=ros" alt="ROS 2 Humble" />
  <img src="https://img.shields.io/badge/Simulator-Gazebo_Classic_11-orange" alt="Gazebo Classic 11" />
  <img src="https://img.shields.io/badge/Drone_Modes-Ground_Truth_%7C_Vision-6f42c1" alt="Ground truth and vision modes" />
  <img src="https://img.shields.io/badge/License-Apache--2.0-blue" alt="Apache 2.0 for original project code" />
</p>

## 📸 Screenshots and Demo

Add screenshots and a short video recorded from your own simulation here. Good choices are:

| Ground-truth cinematic mode | Fixed-camera visual-following mode |
|---|---|
| A cinematic camera angle showing the car and drone | Gazebo view showing the Prius and drone |
| A shot during an orbit or camera-angle transition, if supported by the current controller | `/drone/car_debug` showing the detected ID 7 marker |

Suggested assets folder: `docs/media/`. Do not use fabricated screenshots or present unmeasured performance values as results.

## ✨ Highlights

- 🎬 **Ground-truth cinematic shot mode:** uses the car's Gazebo ground-truth state to coordinate cinematic drone/camera positioning.
- 🎯 **Fixed-camera visual-following mode:** uses a fixed onboard camera and OpenCV ArUco detection (`DICT_4X4_50`, marker ID 7).
- 🛸 **ROS 2 drone control:** separates visual following commands from the Gazebo drone flight/model-state controller.
- 🚘 **Prius simulation:** figure-eight test-world assets and vehicle description.
- 🧪 **Inspectable ROS interfaces:** raw image, debug image, detection array and drone command topics.
- 📊 **Evaluation tools:** scripts and CSV templates are included; formal accuracy and end-to-end percentages remain *not yet measured* until a test set and trials are recorded.

---

# 🧭 The Two Operating Modes

## Mode 1 — 🎬 Ground-Truth Cinematic Shot

### Purpose

This mode is for generating cinematic-style views of the moving Prius. It uses **simulator ground-truth state** to determine the car's position relative to the drone/camera. It is useful for camera positioning and shot choreography because the controller can use the simulated target state directly.

**This is not a visual-detection accuracy test.** The mode does not depend on the fixed camera successfully detecting roof marker ID 7 to know where the simulated car is; it uses the simulator's ground-truth state for cinematic tracking.

### System concept

```text
        Gazebo car ground-truth state
                    │
                    ▼
       Cinematic camera/drone controller
                    │
                    ▼
      Drone / camera shot positioning
                    │
                    ▼
          Cinematic view of the Prius
```

### Launch

From a terminal, source ROS and your workspace and launch the dedicated cinematic test entry point:

```bash
source /opt/ros/humble/setup.bash
source ~/car_drone_ws/install/setup.bash
ros2 launch drone_description spawn_cinematic_test.launch.py
```

This is the repository's cinematic test launch file. Check the launch output for the nodes and models it starts. Do **not** start `spawn_drone.launch.py` at the same time: both launch paths are intended to start their own Gazebo scene.

### What this mode demonstrates

- Ground-truth-based target positioning in simulation.
- Cinematic drone/camera following, distinct from marker-detection-driven following.
- A useful baseline for testing shot behaviour independently of visual detector dropouts.

## Mode 2 — 🎯 Fixed Camera with Visual Detection

### Purpose

This mode demonstrates **vision-based following**. The drone's fixed camera publishes images, the detector looks specifically for roof marker **ID 7**, and the follower converts the marker's normalized image position into movement commands.

Unlike Mode 1, the target-following decision is driven by image detections rather than the car's ground-truth position. Gazebo/model-state control is still used to move the simulated drone, but the follow controller uses the detector output as its target signal.

### System flow

```text
Prius roof marker: ArUco ID 7
             │
             ▼
/drone/camera/image_raw
             │
             ▼
car_detector (OpenCV ArUco)
             │
             ▼
/drone/car_detection
             │
             ▼
drone_follow_controller
             │
             ▼
/drone/cmd_vel
             │
             ▼
drone_flight_controller
             │
             ▼
     Gazebo drone motion
```

### Launch the visual-following mode

Use separate terminals and source ROS plus the workspace in each one. Start each node only once.

**Terminal 1 — Start the visual-following world and spawn models**

```bash
source /opt/ros/humble/setup.bash
source ~/car_drone_ws/install/setup.bash
ros2 launch drone_description spawn_drone.launch.py
```

Wait for Gazebo and the Prius/drone models to appear.

**Terminal 2 — Start the drone flight controller**

```bash
source /opt/ros/humble/setup.bash
source ~/car_drone_ws/install/setup.bash
ros2 run drone_controller drone_flight_controller
```

Run exactly one instance of this controller.

**Terminal 3 — Start visual detection and following**

```bash
source /opt/ros/humble/setup.bash
source ~/car_drone_ws/install/setup.bash
ros2 launch drone_tracker drone_follow.launch.py
```

This launch starts `car_detector` and `drone_follow_controller`.

**Optional Terminal 4 — Start lane detection for autonomous Prius motion**

```bash
source /opt/ros/humble/setup.bash
source ~/car_drone_ws/install/setup.bash
ros2 run prius_adas_control lane_detection
```

**Optional Terminal 5 — Start the autonomous driver**

```bash
source /opt/ros/humble/setup.bash
source ~/car_drone_ws/install/setup.bash
ros2 run prius_adas_control autonomous_driver
```

The two Prius lane-driving nodes are supporting vehicle-motion nodes, **not the two drone operating modes**. Start them only when autonomous car movement is required and the world exposes the expected Prius camera/control topics.

### Main visual-following topics

| Topic | Type | Purpose |
|---|---|---|
| `/drone/camera/image_raw` | `sensor_msgs/msg/Image` | Fixed onboard camera stream consumed by the detector |
| `/drone/car_detection` | `std_msgs/msg/Float32MultiArray` | Detection flag and normalized marker geometry |
| `/drone/car_debug` | `sensor_msgs/msg/Image` | Annotated detector image for inspection |
| `/drone/cmd_vel` | `geometry_msgs/msg/Twist` | Follower movement commands for the flight controller |

`/drone/car_detection` contains six values, in order:

1. `detected` — `0.0` or `1.0`.
2. `center_x` — normalized horizontal marker centre (0–1).
3. `center_y` — normalized vertical marker centre (0–1).
4. `box_w` — normalized bounding-box width.
5. `box_h` — normalized bounding-box height.
6. `size_score` — a marker-size heuristic, **not** a calibrated confidence probability.

To inspect the fixed-camera view or annotated detection:

```bash
ros2 run rqt_image_view rqt_image_view
```

Choose `/drone/camera/image_raw` or `/drone/car_debug` from the viewer. Close the viewer with `Ctrl+C`; rendering live images can consume extra resources on lower-powered computers.

---

## 🧩 Other Components in the Workspace

The repository also contains Prius lane-following nodes and configurations for SLAM Toolbox and Nav2 experiments. These are supporting or additional ADAS components, not the two main drone modes described above. Their inclusion does not mean every SLAM/Nav2 configuration has been validated in a clean installation.

## 🧱 Software Stack

| Component | Role |
|---|---|
| Ubuntu 22.04 | Development environment |
| ROS 2 Humble | Middleware and launch system |
| Gazebo Classic 11 | Vehicle/drone simulation |
| URDF / Xacro | Robot models and sensor definitions |
| Python + OpenCV ArUco | Fixed-camera marker detection |
| `colcon` / `rosdep` | Build and dependency management |
| SLAM Toolbox / Nav2 | Optional ADAS experiments |

Install commonly used dependencies if they are not already available:

```bash
sudo apt update
sudo apt install \
  python3-colcon-common-extensions \
  python3-rosdep \
  ros-humble-desktop \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-xacro \
  ros-humble-cv-bridge \
  python3-opencv
```

Verify that Python OpenCV exposes ArUco:

```bash
python3 -c 'import cv2; print("OpenCV:", cv2.__version__); print("ArUco available:", hasattr(cv2, "aruco"))'
```

Install ROS dependencies where rosdep is configured:

```bash
cd ~/car_drone_ws
rosdep install --from-paths src --ignore-src -r -y
```

## 🔨 Build

```bash
source /opt/ros/humble/setup.bash
cd ~/car_drone_ws
colcon build --symlink-install
source install/setup.bash
```

In each new terminal, source both `/opt/ros/humble/setup.bash` and `~/car_drone_ws/install/setup.bash`.

## 🗂️ Repository Structure

```text
car_drone_ws/
├── src/
│   ├── car_lane_following/   # Lane-following source modules
│   ├── drone_controller/    # Drone flight/model-state controller
│   ├── drone_description/   # Drone descriptions, worlds, models, launch files
│   ├── drone_tracker/       # Visual detector and camera/following controllers
│   ├── prius_adas_control/  # Prius control, worlds, SLAM/Nav2 configuration
│   ├── prius_description/   # Prius URDF and roof marker assets
│   └── prius_msgs/          # Prius-related ROS message definitions
├── docs/                    # Architecture, notes and demo guidance
├── evaluation/              # Evaluation instructions and CSV templates
├── tools/                   # Evaluation scripts
├── README.md
├── LICENSE
└── THIRD_PARTY_NOTICES.md
```

## 📊 Evaluation Status

| Metric | Status |
|---|---|
| Visual detector precision / recall / F1 | **Not yet measured on a labeled dataset** |
| Overall visual-following trial/lap success | **Not yet measured with documented trials** |
| Tracking availability and marker reacquisition time | **Not yet measured** |
| Ground-truth cinematic shot success | **Not yet measured with a defined shot/trial protocol** |

The project contains scripts and CSV templates for later evaluation. Do not enter guessed percentages. Ground-truth cinematic mode and fixed-camera visual mode should be evaluated separately because they use different target information and test different behaviours.

## ⚠️ Current Limitations

- The visual detector may intermittently lose marker ID 7 during turns or near image edges.
- The current visual follower has automatic search disabled; prolonged loss can make it hold position rather than actively search.
- Gazebo Classic may run below real time on lower-powered hardware.
- The detector's size-based score is not a calibrated probability.
- Ground-truth cinematic mode is not evidence of visual detector accuracy; report it separately.

## 📜 License and Third-Party Assets

Original project code is intended to be distributed under Apache-2.0; see [LICENSE](LICENSE). Third-party assets retain their own applicable terms. Review [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) before redistributing bundled models, textures or the compiled Gazebo plugin.

## 🤝 Contributing

Contributions and issue reports are welcome. See [CONTRIBUTING.md](CONTRIBUTING.md) if present, and include the operating system, ROS distribution, Gazebo version, launch command, relevant topic rates and logs when reporting a reproducible issue.
