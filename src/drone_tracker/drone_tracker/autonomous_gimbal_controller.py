#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32MultiArray, Float64MultiArray


class AutonomousGimbalController(Node):

    def __init__(self):
        super().__init__("autonomous_gimbal_controller")

        # Simulation-only cinematic camera aiming.
        self.declare_parameter("use_ground_truth_tracking", True)

        # =========================================================
        # JOINT STATE
        # =========================================================

        self.pan = 0.0
        self.tilt = 0.0

        self.pan_velocity = 0.0
        self.tilt_velocity = 0.0

        self.joint_state_received = False

        self.joint_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10,
        )

        # =========================================================
        # CAR DETECTOR
        # =========================================================

        self.detection_sub = self.create_subscription(
            Float32MultiArray,
            "/drone/gimbal_car_detection",
            self.detection_callback,
            10,
        )

        # Gazebo model poses let the gimbal keep aiming at the Prius
        # even when image-based detection drops frames.
        self.ground_truth = None
        self.ground_truth_time = 0.0
        self.models_sub = self.create_subscription(
            ModelStates,
            "/model_states",
            self.models_callback,
            10,
        )

        self.car_detected = False
        self.car_x = 0.5
        self.car_y = 0.5
        self.car_width = 0.0
        self.car_height = 0.0
        self.car_score = 0.0

        self.last_detection_time = 0.0

        # Remember the car's recent horizontal image movement.
        # +1 means left-to-right; -1 means right-to-left.
        self.last_car_x = None
        self.last_car_x_time = 0.0
        self.last_motion_direction = 1
        self.search_retarget_pending = False

        # =========================================================
        # DETECTION STABILITY
        # =========================================================

        self.lock_counter = 0
        self.unlock_counter = 0

        self.lock_required = 4
        self.loss_required = 10

        # =========================================================
        # MODES
        # =========================================================

        self.mode = "SEARCH"

        # SEARCH
        self.search_pan_min = math.radians(-160.0)
        self.search_pan_max = math.radians(160.0)

        self.search_direction = 1.0

        self.search_speed = math.radians(20.0)

        self.search_tilt = math.radians(
            -10.0
        )

        # TRACK
        self.track_tilt_min = math.radians(
            -45.0
        )

        self.track_tilt_max = math.radians(
            -8.0
        )

        # =========================================================
        # IMAGE TARGET
        # =========================================================

        self.image_center_x = 0.5
        self.image_center_y = 0.5

        self.pan_gain = 0.14
        self.tilt_gain = 0.10

        # =========================================================
        # LIMITS
        # =========================================================

        self.pan_min = math.radians(
            -165.0
        )

        self.pan_max = math.radians(
            165.0
        )

        self.tilt_min = math.radians(
            -75.0
        )

        self.tilt_max = math.radians(
            5.0
        )

        # =========================================================
        # TARGET ANGLES
        # =========================================================

        self.pan_target = 0.0
        self.tilt_target = self.search_tilt

        # =========================================================
        # POSITION CONTROLLER
        # =========================================================

        self.position_pub = self.create_publisher(
            Float64MultiArray,
            "/gimbal_position_controller/commands",
            10,
        )

        # =========================================================
        # TIMER
        # =========================================================

        self.timer = self.create_timer(
            0.10,
            self.control_loop,
        )

        # =========================================================
        # LOGGING
        # =========================================================

        self.last_log_time = 0.0

        self.get_logger().info(
            "AUTONOMOUS GIMBAL CONTROLLER STARTED"
        )

    # =============================================================
    # JOINT STATE
    # =============================================================

    def joint_state_callback(self, msg):

        for i, name in enumerate(msg.name):

            if i >= len(msg.position):
                continue

            if name == "gimbal_pan_joint":

                self.pan = float(
                    msg.position[i]
                )

                if i < len(msg.velocity):
                    self.pan_velocity = float(
                        msg.velocity[i]
                    )

            elif name == "gimbal_tilt_joint":

                self.tilt = float(
                    msg.position[i]
                )

                if i < len(msg.velocity):
                    self.tilt_velocity = float(
                        msg.velocity[i]
                    )

        self.joint_state_received = True

    # =============================================================
    # GAZEBO GROUND-TRUTH CAMERA AIMING
    # =============================================================

    def models_callback(self, msg):
        try:
            car_i = msg.name.index("prius")
            drone_i = msg.name.index("drone")
        except ValueError:
            return

        car = msg.pose[car_i].position
        drone_pose = msg.pose[drone_i]
        drone = drone_pose.position
        q = drone_pose.orientation

        drone_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

        self.ground_truth = (
            car.x, car.y, car.z,
            drone.x, drone.y, drone.z,
            drone_yaw,
        )
        self.ground_truth_time = time.monotonic()

    def update_ground_truth_tracking(self):
        (
            car_x, car_y, car_z,
            drone_x, drone_y, drone_z,
            drone_yaw,
        ) = self.ground_truth

        dx = car_x - drone_x
        dy = car_y - drone_y
        horizontal_distance = math.hypot(dx, dy)

        # Aim pan toward the car, relative to the drone's heading.
        bearing_world = math.atan2(dy, dx)
        desired_pan = self.normalize_angle(bearing_world - drone_yaw)
        desired_pan = self.clamp(
            desired_pan, self.pan_min, self.pan_max
        )

        # Smooth pan motion to avoid abrupt camera swings.
        pan_error = self.normalize_angle(
            desired_pan - self.pan_target
        )
        self.pan_target += self.clamp(
            0.45 * pan_error,
            -math.radians(4.0),
            math.radians(4.0),
        )

        # Camera mounting is pitched by 90 degrees in the URDF.
        # Convert the target's downward viewing angle into joint tilt.
        vertical_drop = max(0.1, drone_z - car_z)
        desired_tilt = (
            math.atan2(vertical_drop, max(0.05, horizontal_distance))
            - math.pi / 2.0
        )
        desired_tilt = self.clamp(
            desired_tilt, self.tilt_min, self.tilt_max
        )

        # Smooth tilt motion as the camera follows the orbit.
        self.tilt_target += self.clamp(
            0.35 * (desired_tilt - self.tilt_target),
            -math.radians(3.0),
            math.radians(3.0),
        )

        self.car_detected = True
        self.car_score = 1.0

    # =============================================================
    # CAR DETECTION
    # =============================================================

    def detection_callback(self, msg):

        if len(msg.data) < 6:
            return

        detected = (
            float(msg.data[0]) > 0.5
        )

        candidate_x = float(msg.data[1])
        candidate_y = float(msg.data[2])
        candidate_width = float(msg.data[3])
        candidate_height = float(msg.data[4])
        candidate_score = float(msg.data[5])

        valid = detected and candidate_score >= 0.30

        if valid:
            # Update the remembered target only on a valid detection.
            # Brief detector dropouts won't reset it to image center.
            self.car_x = candidate_x
            self.car_y = candidate_y
            self.car_width = candidate_width
            self.car_height = candidate_height
            self.car_score = candidate_score

            now = time.monotonic()

            # Infer movement direction from consecutive detections.
            if self.last_car_x is not None and self.last_car_x_time > 0.0:
                elapsed = now - self.last_car_x_time
                dx = self.car_x - self.last_car_x

                # Ignore stale observations and implausibly large jumps.
                if elapsed < 0.8 and 0.01 < abs(dx) < 0.25:
                    self.last_motion_direction = 1 if dx > 0.0 else -1

            self.last_car_x = self.car_x
            self.last_car_x_time = now

            self.car_detected = True
            self.last_detection_time = now

            self.lock_counter += 1
            self.unlock_counter = 0

        else:

            self.unlock_counter += 1

            if (
                self.unlock_counter
                >=
                self.loss_required
            ):

                self.car_detected = False

                self.lock_counter = 0

    # =============================================================
    # CLAMP
    # =============================================================

    def clamp(
        self,
        value,
        low,
        high,
    ):

        return max(
            low,
            min(
                high,
                value,
            )
        )

    # =============================================================
    # NORMALIZE
    # =============================================================

    def normalize_angle(
        self,
        angle,
    ):

        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle

    # =============================================================
    # MODE DECISION
    # =============================================================

    def update_mode(self):

        if (
            self.car_detected
            and
            self.lock_counter
            >=
            self.lock_required
        ):

            if self.mode != "TRACK":

                self.mode = "TRACK"

                self.get_logger().info(
                    "GIMBAL -> TRACK"
                )

        else:

            age = (
                time.monotonic()
                -
                self.last_detection_time
            )

            if age > 1.0:

                if self.mode != "SEARCH":

                    self.mode = "SEARCH"
                    self.search_retarget_pending = True

                    self.get_logger().info(
                        "GIMBAL -> SEARCH"
                    )

    # =============================================================
    # SEARCH BEHAVIOR
    # =============================================================

    def update_search(self):
        # Initialize sector search state on first use.
        if not hasattr(self, "search_positions"):
            self.search_positions = [
                math.radians(v)
                for v in (-150, -120, -90, -60, -30, 0, 30, 60, 90, 120, 150)
            ]
            self.search_index = min(
                range(len(self.search_positions)),
                key=lambda i: abs(self.search_positions[i] - self.pan_target),
            )
            self.search_direction = 1
            self.search_dwell = 0.7
            self.search_tolerance = math.radians(4.0)
            self.search_settled_since = None

        # After losing the car, search one sector in its last
        # observed direction before continuing the normal sweep.
        if self.search_retarget_pending:
            nearest = min(
                range(len(self.search_positions)),
                key=lambda i: abs(self.search_positions[i] - self.pan),
            )

            direction = self.last_motion_direction
            candidate = nearest + direction

            if candidate >= len(self.search_positions):
                direction = -1
                candidate = nearest - 1
            elif candidate < 0:
                direction = 1
                candidate = nearest + 1

            self.search_direction = direction
            self.search_index = max(
                0, min(len(self.search_positions) - 1, candidate)
            )
            self.search_settled_since = None
            self.search_retarget_pending = False

            self.get_logger().info(
                f"Directional reacquisition: searching "
                f"{'RIGHT' if direction > 0 else 'LEFT'} first, "
                f"sector={math.degrees(self.search_positions[self.search_index]):+.0f} deg"
            )

        self.tilt_target = self.search_tilt
        target = self.search_positions[self.search_index]
        self.pan_target = target

        # Wait until the physical pan joint reaches the sector.
        if abs(self.pan - target) > self.search_tolerance:
            self.search_settled_since = None
            return

        now = time.monotonic()

        if self.search_settled_since is None:
            self.search_settled_since = now
            return

        # Keep the camera still while the detector checks several frames.
        if now - self.search_settled_since < self.search_dwell:
            return

        next_index = self.search_index + self.search_direction

        if next_index >= len(self.search_positions):
            self.search_direction = -1
            next_index = len(self.search_positions) - 2
        elif next_index < 0:
            self.search_direction = 1
            next_index = 1

        self.search_index = next_index
        self.search_settled_since = None

        self.get_logger().info(
            f"SEARCH sector -> "
            f"{math.degrees(self.search_positions[self.search_index]):+.0f} deg"
        )

    def update_tracking(self):

        error_x = (
            self.car_x
            -
            self.image_center_x
        )

        error_y = (
            self.car_y
            -
            self.image_center_y
        )

        # Ignore tiny image-centering errors to reduce jitter.
        if abs(error_x) < 0.04:
            error_x = 0.0
        if abs(error_y) < 0.04:
            error_y = 0.0

        # ---------------------------------------------------------
        # PAN
        # ---------------------------------------------------------

        pan_correction = (
            self.pan_gain
            *
            error_x
        )

        self.pan_target += pan_correction

        # ---------------------------------------------------------
        # TILT
        # ---------------------------------------------------------

        tilt_correction = (
            -self.tilt_gain
            *
            error_y
        )

        self.tilt_target += tilt_correction

        # ---------------------------------------------------------
        # DISTANCE-BASED TILT
        # ---------------------------------------------------------
        #
        # Larger image = Prius closer.
        # Smaller image = Prius farther away.
        #
        # We keep the camera wide when far away and
        # more cinematic when the Prius is larger.
        #

        car_area = (
            self.car_width
            *
            self.car_height
        )

        if car_area < 0.08:

            # Far target -> wider survey view.
            preferred = math.radians(
                -10.0
            )

        elif car_area > 0.25:

            # Close target -> cinematic view.
            preferred = math.radians(
                -38.0
            )

        else:

            # Normal follow view.
            preferred = math.radians(
                -24.0
            )

        self.tilt_target = (
            0.80 * self.tilt_target
            +
            0.20 * preferred
        )

        # ---------------------------------------------------------
        # LIMIT
        # ---------------------------------------------------------

        self.pan_target = self.clamp(
            self.pan_target,
            self.pan_min,
            self.pan_max,
        )

        self.tilt_target = self.clamp(
            self.tilt_target,
            self.tilt_min,
            self.tilt_max,
        )

    # =============================================================
    # CONTROL LOOP
    # =============================================================

    def control_loop(self):

        if not self.joint_state_received:
            return

        use_ground_truth = bool(
            self.get_parameter("use_ground_truth_tracking").value
        )

        if use_ground_truth:
            if (
                self.ground_truth is None
                or time.monotonic() - self.ground_truth_time > 1.0
            ):
                self.get_logger().warn(
                    "Waiting for fresh /model_states",
                    throttle_duration_sec=2.0,
                )
                return

            self.mode = "CINEMATIC"
            self.update_ground_truth_tracking()

        else:
            self.update_mode()

            if self.mode == "SEARCH":
                self.update_search()
            elif self.mode == "TRACK":
                self.update_tracking()

        # Respect the physical limits of the gimbal joints.
        self.pan_target = self.clamp(
            self.pan_target,
            self.pan_min,
            self.pan_max,
        )

        self.tilt_target = self.clamp(
            self.tilt_target,
            self.tilt_min,
            self.tilt_max,
        )

        # Send commanded joint positions to ros2_control.
        command = Float64MultiArray()
        command.data = [
            float(self.pan_target),
            float(self.tilt_target),
        ]
        self.position_pub.publish(command)

        # ---------------------------------------------------------
        # LOGGING
        # ---------------------------------------------------------

        now = time.monotonic()

        if now - self.last_log_time > 1.0:

            self.get_logger().info(
                f"GIMBAL {self.mode} | "
                f"pan={math.degrees(self.pan):+.1f} "
                f"-> {math.degrees(self.pan_target):+.1f} | "
                f"tilt={math.degrees(self.tilt):+.1f} "
                f"-> {math.degrees(self.tilt_target):+.1f} | "
                f"car={self.car_detected} "
                f"score={self.car_score:.2f}"
            )

            self.last_log_time = now


def main(args=None):

    rclpy.init(
        args=args
    )

    node = AutonomousGimbalController()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
