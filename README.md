# Vision-Guided Car-Following Drone Simulation

A ROS 2 Humble and Gazebo Classic simulation in which a Prius-style vehicle drives a figure-eight circuit and a simulated aerial drone uses a camera and roof-mounted ArUco marker **ID 7** to follow it.

> **Results policy:** performance percentages are not published until they have been measured from labeled detection examples and documented simulation trials. Replace the `Not yet measured` entries below only with outputs from the evaluation tools in this repository.

## Project status

- **Simulation stack:** Ubuntu 22.04, ROS 2 Humble, Gazebo Classic 11 (recorded local environment).
- **Target marker:** ArUco `DICT_4X4_50`, ID 7.
- **Tracking camera:** `/drone/camera/image_raw`; the local configuration uses a 640 × 480 image and an 80° horizontal field of view.
- **Detection output:** `/drone/car_detection` (`std_msgs/msg/Float32MultiArray`).
- **Drone command topic:** `/drone/cmd_vel` (`geometry_msgs/msg/Twist`).
- **Search behavior:** automatic search is intentionally disabled in the current follower configuration. If the marker is lost for a long interval, the follower can hold position rather than find the vehicle again.

## ROS packages

| Package | Role |
|---|---|
| `drone_description` | Drone Xacro/URDF, world, Gazebo models, spawn and controller launch files |
| `drone_controller` | Drone flight/model-state controller |
| `drone_tracker` | ArUco detector, follower and gimbal-related nodes |
| `prius_description` | Prius URDF and marker/model assets |
| `prius_adas_control` | Prius/ADAS worlds, configuration and launch files |
| `car_lane_following` | Lane-detection and autonomous-driver source modules |
| `prius_msgs` | Prius-related ROS messages |

## Architecture

```text
Prius camera / lane-following stack
                 │
                 ▼
       Prius follows circuit

/drone/camera/image_raw
                 │
                 ▼
        car_detector (ID 7)
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

The detection array is published in this order:

1. `detected` (0 or 1)
2. normalized `center_x` (0 to 1)
3. normalized `center_y` (0 to 1)
4. normalized bounding-box width
5. normalized bounding-box height
6. size-based score (not a calibrated probability)

## Requirements

The project was developed against the following environment; other versions may need adjustments:

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic 11 and ROS-Gazebo integration packages
- `colcon`, `rosdep`, Xacro, `cv_bridge`, ROS message packages, and OpenCV with the ArUco module (`cv2.aruco`)

Install common dependencies if they are not already present:

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-rosdep \
  ros-humble-desktop ros-humble-gazebo-ros-pkgs ros-humble-xacro \
  ros-humble-cv-bridge python3-opencv
```

If `rosdep` has been configured, install package dependencies from the workspace:

```bash
cd ~/car_drone_ws
rosdep install --from-paths src --ignore-src -r -y
```

Some OpenCV builds put the ArUco module in `opencv-contrib`; verify it before launching:

```bash
python3 -c 'import cv2; print(cv2.__version__); print("ArUco available:", hasattr(cv2, "aruco"))'
```

## Build

From the workspace root:

```bash
source /opt/ros/humble/setup.bash
cd ~/car_drone_ws
colcon build --symlink-install
source install/setup.bash
```

If a build fails, resolve the reported missing dependency or package error rather than deleting source files or build directories at random.

## Launching the simulation

The main drone spawn launch file is `drone_description/launch/spawn_drone.launch.py`; it configures Gazebo paths and references the drone Xacro, Prius URDF and `drone_follow.world`.

Use separate terminals and source the workspace in each one. Do not start duplicate instances of the same node.

**Terminal 1 — simulation/spawn:**

```bash
source /opt/ros/humble/setup.bash
source ~/car_drone_ws/install/setup.bash
ros2 launch drone_description spawn_drone.launch.py
```

**Terminal 2 — drone flight controller:**

```bash
source /opt/ros/humble/setup.bash
source ~/car_drone_ws/install/setup.bash
ros2 run drone_controller drone_flight_controller
```

**Terminal 3 — marker detection and drone follower:**

```bash
source /opt/ros/humble/setup.bash
source ~/car_drone_ws/install/setup.bash
ros2 launch drone_tracker drone_follow.launch.py
```

The flight controller should only be started once. The follower launch starts `car_detector` and `drone_follow_controller`.

> **Vehicle-autonomy launch note:** the checked-in `prius_adas_control/launch/prius_adas_world.launch.py` comments describe it as starting Gazebo and the Prius only; they explicitly say it does not start lane detection or the autonomous driver. Do not assume it starts the full autonomous stack. Before claiming a one-command autonomous demo, verify and document the specific lane-detection/autonomous-driver launch sequence used in the working local setup.

## Useful topics

| Topic | Type | Purpose |
|---|---|---|
| `/drone/camera/image_raw` | `sensor_msgs/msg/Image` | Fixed drone camera stream used by detector |
| `/drone/car_detection` | `std_msgs/msg/Float32MultiArray` | Marker-7 detection flag and normalized geometry |
| `/drone/car_debug` | `sensor_msgs/msg/Image` | Annotated detector debug image |
| `/drone/cmd_vel` | `geometry_msgs/msg/Twist` | Follower commands for the drone controller |

Inspect the current runtime graph with:

```bash
ros2 node list
ros2 topic list
ros2 topic info -v /drone/car_detection
```

To inspect images, run `ros2 run rqt_image_view rqt_image_view` and select `/drone/car_debug` or `/drone/camera/image_raw` from the topic dropdown. `Ctrl+C` closes the viewer normally.

## Evaluation: detection metrics

Use images collected under representative distances, viewpoints, turns, lighting and marker occlusion. Assign a ground-truth label to each frame **before looking at or editing the model's predicted label** where practical. `ground_truth_id7=1` means ID 7 is visibly present and is considered detectable under the test protocol; `0` means it is absent or the frame is designated negative by the protocol. Fill `predicted_id7` with the detector's result for that same frame.

Create a copy of `evaluation/detection_labels_template.csv`, populate it, and run:

```bash
python3 tools/evaluate_detection.py evaluation/detection_labels.csv
```

The script reports:

- Accuracy = `(TP + TN) / (TP + TN + FP + FN)`
- Precision = `TP / (TP + FP)`
- Recall = `TP / (TP + FN)`
- Specificity = `TN / (TN + FP)`
- F1 = `2 × precision × recall / (precision + recall)`

An undefined metric (for example, precision when there are no positive predictions) is reported as `N/A`, not zero. A high accuracy score alone can be misleading when positive and negative examples are imbalanced, so report precision, recall, F1 and the confusion counts alongside it.

## Evaluation: overall following performance

Detection accuracy and whole-system following success are different measures. Run a predefined set of trials and record the number of planned and completed laps, eligible tracking time, observed tracking time, marker-loss events and recovery time. Use a fresh copy of `evaluation/trial_results_template.csv` and run:

```bash
python3 tools/evaluate_trials.py evaluation/trial_results.csv
```

The tool reports lap completion rate, successful-trial rate, tracking availability, and average recovery time where those values can be computed. Define the test start/end conditions consistently and keep failed trials in the CSV.

## Current results

| Metric | Result |
|---|---|
| Detection accuracy / precision / recall / F1 | **Not yet measured on a labeled dataset** |
| Full-lap success rate | **Not yet measured with a documented trial set** |
| Tracking availability | **Not yet measured** |
| Mean marker reacquisition time | **Not yet measured** |
| Local observed camera delivery rate | Varied substantially in recent tests; see project notes and measure again under a documented setup |

Do not replace these entries with estimates based only on console logs, CPU utilization or camera topic rate. Add dataset size, number of trials, lighting/view conditions and software/hardware versions when publishing results.

## Current limitations

- The marker detector can intermittently lose ID 7 during turns or when it approaches an image edge.
- Long marker loss can cause the follower to enter `HOLD`; automatic search is currently disabled.
- Gazebo Classic can run below real time on low-end hardware, affecting camera delivery and simulation timing.
- The sixth detection array element is based on marker size and is **not a calibrated confidence probability**.
- The autonomous lane-driving launch path should be verified for the exact environment before presenting the project as a one-command demo.

## Repository hygiene

Generated `build/`, `install/` and `log/` folders are not source code and should not be committed. Keep the URDFs, worlds, textures, plugin assets required at runtime, launch files, package manifests and evaluation scripts. Check licenses for models, assets and the included Gazebo plugin before redistributing them publicly.

## License

Original project code intended for distribution under Apache License 2.0
is licensed under Apache-2.0; see [LICENSE](LICENSE).

Third-party assets and components retain their applicable licences and
notices. Review [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) before
redistributing the bundled models, textures, or Gazebo plugin.
