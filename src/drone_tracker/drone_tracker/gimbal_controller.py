#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from gazebo_msgs.srv import ApplyJointEffort


class GimbalController(Node):

    def __init__(self):
        super().__init__("gimbal_controller")

        # =========================================================
        # PARAMETERS
        # =========================================================

        self.declare_parameter("pan_deg", 0.0)
        self.declare_parameter("tilt_deg", -10.0)

        self.pan_target = math.radians(
            float(self.get_parameter("pan_deg").value)
        )

        self.tilt_target = math.radians(
            float(self.get_parameter("tilt_deg").value)
        )

        # =========================================================
        # JOINT STATE
        # =========================================================

        self.pan_position = 0.0
        self.tilt_position = 0.0

        self.pan_velocity = 0.0
        self.tilt_velocity = 0.0

        self.joint_state_received = False

        self.joint_sub = self.create_subscription(
            JointState,
            "/drone/joint_states",
            self.joint_state_callback,
            10,
        )

        # =========================================================
        # GAZEBO EFFORT SERVICE
        # =========================================================

        self.effort_client = self.create_client(
            ApplyJointEffort,
            "/apply_joint_effort",
        )

        # =========================================================
        # PD GAINS
        # =========================================================

        self.kp_pan = 0.05
        self.kd_pan = 0.01

        self.kp_tilt = 0.05
        self.kd_tilt = 0.01

        # Maximum effort.
        self.max_effort = 0.08

        # =========================================================
        # CONTROL LOOP
        # =========================================================

        self.control_timer = self.create_timer(
            0.10,
            self.control_loop,
        )

        self.last_service_time = 0.0

        self.get_logger().info(
            "Gimbal controller started"
        )

        self.get_logger().info(
            f"Target pan  = {math.degrees(self.pan_target):.1f} deg"
        )

        self.get_logger().info(
            f"Target tilt = {math.degrees(self.tilt_target):.1f} deg"
        )

    # =============================================================
    # JOINT STATE
    # =============================================================

    def joint_state_callback(self, msg):

        for i, name in enumerate(msg.name):

            if i >= len(msg.position):
                continue

            if name == "gimbal_pan_joint":

                self.pan_position = float(
                    msg.position[i]
                )

                if i < len(msg.velocity):

                    self.pan_velocity = float(
                        msg.velocity[i]
                    )

            elif name == "gimbal_tilt_joint":

                self.tilt_position = float(
                    msg.position[i]
                )

                if i < len(msg.velocity):

                    self.tilt_velocity = float(
                        msg.velocity[i]
                    )

        self.joint_state_received = True

    # =============================================================
    # CLAMP
    # =============================================================

    def clamp(self, value, low, high):

        return max(
            low,
            min(high, value),
        )

    # =============================================================
    # COMPUTE EFFORT
    # =============================================================

    def compute_pan_effort(self):

        error = (
            self.pan_target
            -
            self.pan_position
        )

        effort = (
            self.kp_pan * error
            -
            self.kd_pan * self.pan_velocity
        )

        return self.clamp(
            effort,
            -self.max_effort,
            self.max_effort,
        )

    def compute_tilt_effort(self):

        error = (
            self.tilt_target
            -
            self.tilt_position
        )

        effort = (
            self.kp_tilt * error
            -
            self.kd_tilt * self.tilt_velocity
        )

        return self.clamp(
            effort,
            -self.max_effort,
            self.max_effort,
        )

    # =============================================================
    # SEND EFFORT
    # =============================================================

    def send_effort(
        self,
        joint_name,
        effort,
    ):

        if not self.effort_client.service_is_ready():
            return

        request = ApplyJointEffort.Request()

        request.joint_name = joint_name
        request.effort = float(effort)

        request.start_time.sec = 0
        request.start_time.nanosec = 0

        # 0.15 second force window.
        request.duration.sec = 0
        request.duration.nanosec = 150000000

        self.effort_client.call_async(
            request
        )

    # =============================================================
    # CONTROL LOOP
    # =============================================================

    def control_loop(self):

        if not self.joint_state_received:
            return

        now = time.monotonic()

        # Prevent excessive service traffic.
        if (
            now - self.last_service_time
            <
            0.08
        ):
            return

        if not self.effort_client.service_is_ready():

            self.get_logger().warning(
                "Waiting for /apply_joint_effort"
            )

            return

        pan_effort = (
            self.compute_pan_effort()
        )

        tilt_effort = (
            self.compute_tilt_effort()
        )

        self.send_effort(
            "gimbal_pan_joint",
            pan_effort,
        )

        self.send_effort(
            "gimbal_tilt_joint",
            tilt_effort,
        )

        self.last_service_time = now

        # ---------------------------------------------------------
        # LOG
        # ---------------------------------------------------------

        self.get_logger().info(
            f"GIMBAL | "
            f"pan={math.degrees(self.pan_position):+.1f} "
            f"-> {math.degrees(self.pan_target):+.1f} deg | "
            f"tilt={math.degrees(self.tilt_position):+.1f} "
            f"-> {math.degrees(self.tilt_target):+.1f} deg | "
            f"effort=({pan_effort:+.4f}, "
            f"{tilt_effort:+.4f})"
        )


def main(args=None):

    rclpy.init(
        args=args
    )

    node = GimbalController()

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:
        pass

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
