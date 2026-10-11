#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from gazebo_msgs.srv import SetEntityState


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def quaternion_from_euler(roll, pitch, yaw):
    """
    Convert roll, pitch, yaw to quaternion.
    """

    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)

    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qw = (
        cr * cp * cy
        + sr * sp * sy
    )

    qx = (
        sr * cp * cy
        - cr * sp * sy
    )

    qy = (
        cr * sp * cy
        + sr * cp * sy
    )

    qz = (
        cr * cp * sy
        - sr * sp * cy
    )

    return qx, qy, qz, qw


class DroneFlightController(Node):

    def __init__(self):

        super().__init__("drone_flight_controller")

        # =========================================================
        # PARAMETERS
        # =========================================================

        self.declare_parameter(
            "entity_name",
            "drone"
        )

        self.declare_parameter(
            "initial_x",
            0.0
        )

        self.declare_parameter(
            "initial_y",
            0.0
        )

        self.declare_parameter(
            "initial_z",
            8.0
        )

        self.entity_name = self.get_parameter(
            "entity_name"
        ).value

        # =========================================================
        # DRONE STATE
        # =========================================================

        self.x = float(
            self.get_parameter("initial_x").value
        )

        self.y = float(
            self.get_parameter("initial_y").value
        )

        self.z = float(
            self.get_parameter("initial_z").value
        )

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        # =========================================================
        # COMMAND
        # =========================================================

        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_vz = 0.0

        self.cmd_roll_rate = 0.0
        self.cmd_pitch_rate = 0.0
        self.cmd_yaw_rate = 0.0

        self.last_command_time = time.monotonic()

        self.command_timeout = 0.5

        # =========================================================
        # LIMITS
        # =========================================================

        self.max_horizontal_speed = 3.80
        self.max_vertical_speed = 1.5
        self.max_altitude = 15.0  # Safety ceiling, metres

        # Publish the altitude maintained by this flight controller.
        # This is not the car's Gazebo ground-truth position.
        self.altitude_pub = self.create_publisher(
            Float32, "/drone/estimated_altitude", 10
        )

        self.max_roll_rate = math.radians(40.0)
        self.max_pitch_rate = math.radians(40.0)
        self.max_yaw_rate = math.radians(90.0)

        # Maximum attitude
        self.max_roll = math.radians(20.0)
        self.max_pitch = math.radians(20.0)

        # =========================================================
        # ROS SUBSCRIBER
        # =========================================================

        self.command_sub = self.create_subscription(
            Twist,
            "/drone/cmd_vel",
            self.command_callback,
            10
        )

        # =========================================================
        # GAZEBO SERVICE
        # =========================================================

        self.set_state_client = self.create_client(
            SetEntityState,
            "/set_entity_state"
        )

        self.get_logger().info(
            "Waiting for /set_entity_state ..."
        )

        while not self.set_state_client.wait_for_service(
            timeout_sec=1.0
        ):

            self.get_logger().warn(
                "/set_entity_state not available yet"
            )

        self.get_logger().info(
            "Gazebo entity-state service connected"
        )

        # =========================================================
        # CONTROL LOOP
        # =========================================================

        self.control_hz = 30.0

        self.dt = 1.0 / self.control_hz

        self.last_update = time.monotonic()

        self.timer = self.create_timer(
            self.dt,
            self.update
        )

        self.get_logger().info(
            "Drone flight controller started"
        )

        self.get_logger().info(
            "Command interface:"
        )

        self.get_logger().info(
            "linear.x = forward/back"
        )

        self.get_logger().info(
            "linear.y = left/right"
        )

        self.get_logger().info(
            "linear.z = up/down"
        )

        self.get_logger().info(
            "angular.x = roll rate"
        )

        self.get_logger().info(
            "angular.y = pitch rate"
        )

        self.get_logger().info(
            "angular.z = yaw rate"
        )

    # =========================================================
    # COMMAND CALLBACK
    # =========================================================

    def command_callback(self, msg):

        self.cmd_vx = clamp(
            float(msg.linear.x),
            -self.max_horizontal_speed,
            self.max_horizontal_speed
        )

        self.cmd_vy = clamp(
            float(msg.linear.y),
            -self.max_horizontal_speed,
            self.max_horizontal_speed
        )

        self.cmd_vz = clamp(
            float(msg.linear.z),
            -self.max_vertical_speed,
            self.max_vertical_speed
        )

        self.cmd_roll_rate = clamp(
            float(msg.angular.x),
            -self.max_roll_rate,
            self.max_roll_rate
        )

        self.cmd_pitch_rate = clamp(
            float(msg.angular.y),
            -self.max_pitch_rate,
            self.max_pitch_rate
        )

        self.cmd_yaw_rate = clamp(
            float(msg.angular.z),
            -self.max_yaw_rate,
            self.max_yaw_rate
        )

        self.last_command_time = time.monotonic()

    # =========================================================
    # UPDATE
    # =========================================================

    def update(self):

        now = time.monotonic()

        dt = now - self.last_update

        self.last_update = now

        dt = clamp(
            dt,
            0.001,
            0.10
        )

        # -----------------------------------------------------
        # COMMAND TIMEOUT
        # -----------------------------------------------------

        if (
            now - self.last_command_time
            >
            self.command_timeout
        ):

            vx = 0.0
            vy = 0.0
            vz = 0.0

            roll_rate = 0.0
            pitch_rate = 0.0
            yaw_rate = 0.0

        else:

            vx = self.cmd_vx
            vy = self.cmd_vy
            vz = self.cmd_vz

            roll_rate = self.cmd_roll_rate
            pitch_rate = self.cmd_pitch_rate
            yaw_rate = self.cmd_yaw_rate

        # =====================================================
        # BODY VELOCITY → WORLD VELOCITY
        # =====================================================

        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)

        world_vx = (
            cos_yaw * vx
            - sin_yaw * vy
        )

        world_vy = (
            sin_yaw * vx
            + cos_yaw * vy
        )

        # =====================================================
        # POSITION
        # =====================================================

        # -----------------------------------------------------
        # HORIZONTAL SAFETY FENCE
        # Keep the drone inside the figure-eight circuit envelope.
        # The limits include space around the road segments.
        # -----------------------------------------------------
        min_x, max_x = -27.0, 27.0
        min_y, max_y = -17.0, 17.0

        # Stop outward movement when a boundary is reached.
        if self.x >= max_x and world_vx > 0.0:
            world_vx = 0.0
        elif self.x <= min_x and world_vx < 0.0:
            world_vx = 0.0

        if self.y >= max_y and world_vy > 0.0:
            world_vy = 0.0
        elif self.y <= min_y and world_vy < 0.0:
            world_vy = 0.0

        # Clamp positions to the safety envelope after each time step.
        self.x = clamp(self.x + world_vx * dt, min_x, max_x)
        self.y = clamp(self.y + world_vy * dt, min_y, max_y)
        self.z += vz * dt

        # Prevent drone going below ground
        # Enforce both a floor and a hard altitude ceiling.
        self.z = clamp(
            self.z,
            0.5,
            self.max_altitude
        )

        altitude_msg = Float32()
        altitude_msg.data = float(self.z)
        self.altitude_pub.publish(altitude_msg)

        # =====================================================
        # ATTITUDE
        # =====================================================

        self.roll += roll_rate * dt
        self.pitch += pitch_rate * dt
        self.yaw += yaw_rate * dt

        self.roll = clamp(
            self.roll,
            -self.max_roll,
            self.max_roll
        )

        self.pitch = clamp(
            self.pitch,
            -self.max_pitch,
            self.max_pitch
        )

        # Keep yaw bounded
        self.yaw = math.atan2(
            math.sin(self.yaw),
            math.cos(self.yaw)
        )

        # =====================================================
        # SEND STATE TO GAZEBO
        # =====================================================

        self.send_state()

    # =========================================================
    # GAZEBO STATE
    # =========================================================

    def send_state(self):

        qx, qy, qz, qw = quaternion_from_euler(
            self.roll,
            self.pitch,
            self.yaw
        )

        request = SetEntityState.Request()

        request.state.name = self.entity_name

        request.state.reference_frame = "world"

        # -----------------------------------------------------
        # POSITION
        # -----------------------------------------------------

        request.state.pose.position.x = self.x
        request.state.pose.position.y = self.y
        request.state.pose.position.z = self.z

        # -----------------------------------------------------
        # ORIENTATION
        # -----------------------------------------------------

        request.state.pose.orientation.x = qx
        request.state.pose.orientation.y = qy
        request.state.pose.orientation.z = qz
        request.state.pose.orientation.w = qw

        # -----------------------------------------------------
        # VELOCITY
        # -----------------------------------------------------

        request.state.twist.linear.x = 0.0
        request.state.twist.linear.y = 0.0
        request.state.twist.linear.z = 0.0

        request.state.twist.angular.x = 0.0
        request.state.twist.angular.y = 0.0
        request.state.twist.angular.z = 0.0

        self.set_state_client.call_async(request)


def main(args=None):

    rclpy.init(args=args)

    node = DroneFlightController()

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
