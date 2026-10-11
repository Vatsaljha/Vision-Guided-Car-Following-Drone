# Vision-Guided Car-Following Drone 

A ROS 2 and Gazebo Classic simulation in which a Prius-style vehicle travels on a figure-eight circuit and a simulated aerial drone uses computer vision to track an ArUco marker (`DICT_4X4_50`, ID `7`) mounted on the vehicle roof.

> **Project status:** Working simulation prototype. The current follower has been reported to work better after tuning, but formal detection and whole-system benchmark results have not yet been collected. No accuracy percentages are claimed in this repository.

## Highlights

- **Vision-based target detection:** OpenCV ArUco detector configured for roof marker ID 7.
- **Image-space following:** A ROS 2 follower converts normalized image-centre error into drone movement commands.
- **Gazebo Classic simulation:** Prius-style vehicle, figure-eight environment, drone model and camera sensor.
- **Inspectable ROS interfaces:** Detector output, raw camera stream and annotated debug image are exposed as ROS topics.
- **Evaluation tooling:** Scripts and CSV templates are provided to calculate detection metrics and log full-system trials once ground-truth data is collected.

## Demo media

Screenshots and a short demo recording have not yet been added to this repository. See [`docs/media/README.md`](docs/media/README.md) for suggested captures and filenames. Add your own recorded simulation media before presenting the project as a polished demo; do not use fabricated images or metrics.

## Tested development environment

| Component | Reference environment |
|---|---|
| OS | Ubuntu 22.04 |
| Middleware | ROS 2 Humble |
| Simulator | Gazebo Classic 11 |
| Robot description | URDF / Xacro |
| Vision | OpenCV ArUco via `cv2.aruco` and `cv_bridge` |
| Build | `colcon` |

Gazebo Classic and image processing can run below real time on lower-powered computers. Actual camera throughput depends on CPU, GPU, world complexity and which viewer tools are open.

## How it works

1. The world and robot-description launch file starts Gazebo Classic and spawns the drone and Prius model.
2. The drone's fixed camera publishes images on `/drone/camera/image_raw`.
3. `car_detector` searches for ArUco ID 7 and publishes normalized marker location and bounding-box information.
4. `drone_follow_controller` uses the detection to generate movement commands on `/drone/cmd_vel`.
5. `drone_flight_controller` consumes the commands and updates the simulated drone state in Gazebo.

See [the system architecture](docs/ARCHITECTURE.md) for a diagram and topic details.

## Repository structure

```text
car_drone_ws/
├── src/
│   ├── car_lane_following/   # Lane-following source modules
│   ├── drone_controller/    # Drone flight/model-state controller
│   ├── drone_description/   # Drone URDF/Xacro, worlds and Gazebo model assets
│   ├── drone_tracker/       # ArUco detector and drone-following controllers
│   ├── prius_adas_control/  # Prius driving nodes, worlds and configuration
│   ├── prius_description/   # Prius URDF and roof-marker assets
│   └── prius_msgs/          # Prius-related ROS message definitions
├── tools/                   # Evaluation utilities
├── evaluation/              # CSV templates and evaluation instructions
├── docs/                    # Architecture, results log and release guidance
├── README.md
├── CONTRIBUTING.md
├── LICENSE
└── THIRD_PARTY_NOTICES.md
```

## Requirements

The primary development environment is Ubuntu 22.04 with ROS 2 Humble and Gazebo Classic 11. Install ROS 2 Humble following the official ROS installation guide, then install common project dependencies:

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

Check that the Python OpenCV installation includes the ArUco module:

```bash
python3 -c 'import cv2; print("OpenCV:", cv2.__version__); print("ArUco available:", hasattr(cv2, "aruco"))'
```

If `rosdep` is initialized and package dependencies are available from configured sources, install workspace dependencies:

```bash
cd ~/car_drone_ws
rosdep install --from-paths src --ignore-src -r -y
```

## Build the workspace

Clone this repository into `~/car_drone_ws` or use your existing workspace, then build:

```bash
source /opt/ros/humble/setup.bash
cd ~/car_drone_ws
colcon build --symlink-install
source install/setup.bash
```

Source both the ROS distribution and workspace setup files in every new terminal used to run ROS commands.

## Run the simulation

Use separate terminals. The sequence below starts the simulation components separately and avoids duplicate nodes.

### Terminal 1 — Start Gazebo and spawn the drone / Prius

```bash
source /opt/ros/humble/setup.bash
source ~/car_drone_ws/install/setup.bash
ros2 launch drone_description spawn_drone.launch.py
```

This launch uses `drone_follow.world`, the cinematic drone Xacro and `prius_no_lidar.urdf`. Wait for Gazebo and both models to spawn before starting the controllers.

### Terminal 2 — Start the drone flight controller

```bash
source /opt/ros/humble/setup.bash
source ~/car_drone_ws/install/setup.bash
ros2 run drone_controller drone_flight_controller
```

Run only one instance of this controller.

### Terminal 3 — Start target detection and drone following

```bash
source /opt/ros/humble/setup.bash
source ~/car_drone_ws/install/setup.bash
ros2 launch drone_tracker drone_follow.launch.py
```

This launch starts `car_detector` and `drone_follow_controller`.

### Terminals 4 and 5 — Start Prius lane following

The drone spawn launch does not itself start the separate lane-detection and autonomous-driver nodes. In the current workspace, these executables are provided by `prius_adas_control`:

Terminal 4:

```bash
source /opt/ros/humble/setup.bash
source ~/car_drone_ws/install/setup.bash
ros2 run prius_adas_control lane_detection
```

Terminal 5:

```bash
source /opt/ros/humble/setup.bash
source ~/car_drone_ws/install/setup.bash
ros2 run prius_adas_control autonomous_driver
```

If either command is unavailable after building, inspect the installed executables with `ros2 pkg executables prius_adas_control` rather than starting duplicate nodes or changing unrelated files. Launch composition may differ if you use a custom local launch file.

## ROS topics

| Topic | Message type | Description |
|---|---|---|
| `/drone/camera/image_raw` | `sensor_msgs/msg/Image` | Fixed drone camera stream used by the detector |
| `/drone/car_detection` | `std_msgs/msg/Float32MultiArray` | Detection flag, normalized centre, normalized box size and size-based score |
| `/drone/car_debug` | `sensor_msgs/msg/Image` | Annotated detector debug view |
| `/drone/cmd_vel` | `geometry_msgs/msg/Twist` | Movement commands sent by the follower |

The six entries in `/drone/car_detection` are ordered as:

1. `detected` — `0` or `1`
2. `center_x` — normalized horizontal centre (`0` to `1`)
3. `center_y` — normalized vertical centre (`0` to `1`)
4. `box_w` — normalized bounding-box width
5. `box_h` — normalized bounding-box height
6. `size_score` — marker-size heuristic, **not** a calibrated confidence probability

Useful graph checks:

```bash
ros2 node list
ros2 topic list
ros2 topic info -v /drone/car_detection
```

To inspect images:

```bash
ros2 run rqt_image_view rqt_image_view
```

Choose `/drone/car_debug` or `/drone/camera/image_raw` in the image viewer. Close the viewer with `Ctrl+C` when finished; rendering an image viewer consumes additional resources.

## Configuration notes

- The fixed camera is configured for a 640 × 480 image and an 80° horizontal field of view.
- The target marker is ArUco `DICT_4X4_50`, ID 7, on the Prius roof.
- Automatic search after long marker loss is intentionally disabled in the current follower behaviour. On extended loss, the drone may enter `HOLD` rather than search for the vehicle.
- Detector processing was optimized to reduce repeated expensive fallback passes. Keep the working detector and controller together when testing changes.

## Evaluation and results

**Detection accuracy and end-to-end success percentages are not yet measured.** The repository contains scripts and templates, not completed measurement datasets. Do not treat topic rate, CPU usage, or a successful individual demo run as detector accuracy.

For a formal evaluation:

1. Assemble representative images from straight segments, turns, image edges, changing distances and difficult visibility conditions.
2. Independently label whether ID 7 is present and detectable in each frame, then compare those labels with the detector output.
3. Run `tools/evaluate_detection.py` against the completed copy of `evaluation/detection_labels_template.csv`.
4. Record repeated full-system trials in a copy of `evaluation/trial_results_template.csv` and run `tools/evaluate_trials.py`.
5. Publish sample counts, test conditions, failure cases and calculated metrics together.

See [`evaluation/README.md`](evaluation/README.md) and [`docs/RESULTS_LOG_TEMPLATE.md`](docs/RESULTS_LOG_TEMPLATE.md). Keep completed datasets and JSON reports outside the repository until reviewed for size and privacy; commit only selected, documented examples if needed.

## Known limitations

- The marker can intermittently disappear during turns or when it approaches the camera image edge.
- Long detection loss can lead to `HOLD`; search is disabled in the current follower configuration.
- Gazebo Classic may run below real time on lower-powered hardware, affecting sensor update timing.
- The `size_score` field is a size-based heuristic, not a probability of correct detection.
- The project has separate car lane-following and drone-following components; verify that all required nodes are started in your chosen launch sequence.

## Troubleshooting

**A package or executable is not found**

```bash
source /opt/ros/humble/setup.bash
source ~/car_drone_ws/install/setup.bash
ros2 pkg executables drone_tracker
ros2 pkg executables drone_controller
ros2 pkg executables prius_adas_control
```

If a source or package manifest changed, rebuild the relevant packages or the full workspace and source `install/setup.bash` again.

**The drone does not follow the car**

Check that `/drone/camera/image_raw` publishes, `car_detector` is running, `/drone/car_detection` changes when ID 7 is in view, the follower is publishing `/drone/cmd_vel`, and exactly one drone flight controller is running.

**The simulation is slow**

Close `rqt_image_view` when not needed, watch `gz stats`, and avoid starting duplicate nodes. Gazebo's real-time factor below `1.0` means simulation time is advancing slower than wall-clock time.

## Licence and third-party assets

Original project code intended for distribution under Apache License 2.0 is licensed under Apache-2.0; see [`LICENSE`](LICENSE).

Third-party packages, models, textures and binaries retain their applicable licences and notices. Review [`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md) before redistributing any bundled assets, particularly the compiled Gazebo plugin. The top-level Apache licence does not automatically override third-party terms.

## Contributing

See [`CONTRIBUTING.md`](CONTRIBUTING.md). Please keep changes focused, document how changes were tested, and do not claim benchmark results unless the measurements and conditions are recorded.
