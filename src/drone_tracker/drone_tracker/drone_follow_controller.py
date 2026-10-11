#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray


class DroneFollowController(Node):

    def __init__(self):
        super().__init__('drone_follow_controller')

        # =========================================================
        # ROS
        # =========================================================

        self.cmd_pub = self.create_publisher(
            Twist,
            '/drone/cmd_vel',
            10
        )

        self.detection_sub = self.create_subscription(
            Float32MultiArray,
            '/drone/car_detection',
            self.detection_callback,
            10
        )

        self.timer = self.create_timer(
            0.1,
            self.control_loop
        )

        # =========================================================
        # TARGET
        # =========================================================

        self.detected = False

        self.car_x = 0.5
        self.car_y = 0.5
        self.car_w = 0.0
        self.car_h = 0.0
        self.score = 0.0

        self.filtered_x = 0.5
        self.filtered_y = 0.5

        # Faster response than previous version.
        self.filter_alpha = 0.55

        self.last_seen = 0.0
        self.reacquire_until = 0.0
        self.detection_timeout = 0.30

        # Short detector dropout allowed.
        self.hold_after_loss = 0.75

        # =========================================================
        # IMAGE TARGET
        # =========================================================

        self.target_x = 0.50
        self.target_y = 0.50

        # Smaller deadband for turning.
        self.deadband = 0.025

        # =========================================================
        # NORMAL FOLLOW SPEED
        # =========================================================

        self.normal_max_speed = 3.60

        # =========================================================
        # TURN / EDGE SPEED
        # =========================================================

        self.turn_max_speed = 3.50
        # When car gets this far from image center,
        # use stronger turn response.
        self.turn_error = 0.14
        # Emergency edge region.
        self.edge_x = 0.82
        self.edge_y = 0.82

        # =========================================================
        # RESPONSE
        # =========================================================

        self.normal_gain = 4.0

        # Stronger gain near turns.
        self.turn_gain = 7.0
        # Minimum useful command.
        self.min_speed = 0.15

        # =========================================================
        # IMAGE ERROR DERIVATIVE
        # =========================================================
        #
        # This is NOT "car velocity".
        #
        # It is simply the rate at which the visual tracking error
        # itself is changing.
        #
        # This lets the drone react earlier when the Prius rapidly
        # moves toward the edge of the camera during a turn.
        # =========================================================

        self.previous_error_x = 0.0
        self.previous_error_y = 0.0

        self.error_rate_x = 0.0
        self.error_rate_y = 0.0

        self.error_rate_alpha = 0.45

        # Turn anticipation.
        self.turn_derivative_gain = 0.55

        # =========================================================
        # AXIS MAPPING
        # =========================================================

        self.image_x_to_drone_y = -1.0
        self.image_y_to_drone_x = -1.0

        # =========================================================
        # COMMAND MEMORY
        # =========================================================

        self.last_command_x = 0.0
        self.last_command_y = 0.0
        self.last_mode_in_turn = False
        self.straight_loss_coast = 0.35

        self.previous_control_time = time.monotonic()

        # =========================================================
        # LOG
        # =========================================================

        self.get_logger().info(
            'Tuned visual drone follower started'
        )

        self.get_logger().info(
            'Turn-aware tracking ENABLED'
        )

        self.get_logger().info(
            'Maximum horizontal speed: 2.90 m/s'
        )

    # =============================================================
    # DETECTION
    # =============================================================

    def detection_callback(self, msg):

        if len(msg.data) < 6:
            return

        detected = msg.data[0] > 0.5

        if not detected:
            self.detected = False
            return

        x = float(msg.data[1])
        y = float(msg.data[2])

        w = float(msg.data[3])
        h = float(msg.data[4])

        score = float(msg.data[5])

        # The detector's score represents apparent marker size, not
        # probability of a correct decode. ID 7 is already decoded there.
        if w < 0.005 or h < 0.005:
            self.detected = False
            return

        now = time.monotonic()
        if (
            self.last_seen > 0.0
            and now - self.last_seen > self.hold_after_loss
        ):
            self.reacquire_until = now + 0.8
            self.get_logger().info(
                'Marker reacquired: briefly reducing speed to recenter'
            )

        self.detected = True
        self.last_seen = now

        self.car_x = x
        self.car_y = y
        self.car_w = w
        self.car_h = h
        self.score = score

        # ---------------------------------------------------------
        # Low-pass filter
        # ---------------------------------------------------------

        self.filtered_x = (
            self.filter_alpha * x
            + (1.0 - self.filter_alpha)
            * self.filtered_x
        )

        self.filtered_y = (
            self.filter_alpha * y
            + (1.0 - self.filter_alpha)
            * self.filtered_y
        )

    # =============================================================
    # SPEED CALCULATION
    # =============================================================

    def base_speed(
        self,
        error,
        gain
    ):

        absolute_error = abs(error)

        if absolute_error <= self.deadband:
            return 0.0

        effective_error = (
            absolute_error - self.deadband
        )

        normalized = math.tanh(
            gain * effective_error
        )

        speed = (
            self.min_speed
            + normalized
            * (
                self.normal_max_speed
                - self.min_speed
            )
        )

        return math.copysign(
            speed,
            error
        )

    # =============================================================
    # TURN BOOST
    # =============================================================

    def apply_turn_boost(
        self,
        speed,
        error,
        error_rate
    ):

        if speed == 0.0:
            return 0.0

        # ---------------------------------------------------------
        # If error is growing in the same direction, the car is
        # moving farther from the center of the image.
        #
        # This usually happens strongly at a turn.
        # ---------------------------------------------------------

        error_growing = (
            error * error_rate > 0.0
        )

        if error_growing:

            boost = min(
                1.0,
                abs(error_rate)
                * self.turn_derivative_gain
            )

            speed *= (
                1.0 + boost
            )

        # ---------------------------------------------------------
        # If target is very near an edge, increase correction.
        # ---------------------------------------------------------

        if abs(error) >= self.turn_error:

            edge_factor = min(
                1.0,
                (
                    abs(error)
                    - self.turn_error
                )
                / 0.30
            )

            speed += (
                math.copysign(
                    0.50 * edge_factor,
                    speed
                )
            )

        return speed

    # =============================================================
    # LIMIT SPEED
    # =============================================================

    def limit_horizontal_speed(
        self,
        x,
        y,
        max_speed
    ):

        magnitude = math.sqrt(
            x * x + y * y
        )

        if magnitude <= max_speed:
            return x, y

        scale = (
            max_speed
            / magnitude
        )

        return (
            x * scale,
            y * scale
        )

    # =============================================================
    # HOLD
    # =============================================================

    def publish_hold(self):

        cmd = Twist()

        self.cmd_pub.publish(cmd)

        self.last_command_x = 0.0
        self.last_command_y = 0.0

    # =============================================================
    # FOLLOW
    # =============================================================

    def follow(self):

        now = time.monotonic()

        dt = (
            now
            - self.previous_control_time
        )

        dt = max(
            0.01,
            min(0.25, dt)
        )

        self.previous_control_time = now

        # ---------------------------------------------------------
        # Image error
        # ---------------------------------------------------------

        error_x = (
            self.filtered_x
            - self.target_x
        )

        error_y = (
            self.filtered_y
            - self.target_y
        )

        # ---------------------------------------------------------
        # Error rate
        # ---------------------------------------------------------

        raw_rate_x = (
            error_x
            - self.previous_error_x
        ) / dt

        raw_rate_y = (
            error_y
            - self.previous_error_y
        ) / dt

        self.error_rate_x = (
            self.error_rate_alpha
            * raw_rate_x
            +
            (1.0 - self.error_rate_alpha)
            * self.error_rate_x
        )

        self.error_rate_y = (
            self.error_rate_alpha
            * raw_rate_y
            +
            (1.0 - self.error_rate_alpha)
            * self.error_rate_y
        )

        self.previous_error_x = error_x
        self.previous_error_y = error_y

        # ---------------------------------------------------------
        # Decide normal vs turn mode.
        # ---------------------------------------------------------

        in_turn = (
            abs(error_x) > self.turn_error
            or abs(error_y) > self.turn_error
            or abs(self.error_rate_x) > 0.25
            or abs(self.error_rate_y) > 0.25
            or self.filtered_x < 0.22
            or self.filtered_x > 0.78
            or self.filtered_y < 0.22
            or self.filtered_y > 0.78
        )

        self.last_mode_in_turn = in_turn

        if in_turn:

            gain_x = self.turn_gain
            gain_y = self.turn_gain
            max_speed = self.turn_max_speed

        else:

            gain_x = self.normal_gain
            gain_y = self.normal_gain
            max_speed = self.normal_max_speed

        # ---------------------------------------------------------
        # Briefly recenter after a sustained detection loss.
        if time.monotonic() < self.reacquire_until:
            gain_x = min(gain_x, 3.0)
            gain_y = min(gain_y, 3.0)
            max_speed = min(max_speed, 1.2)

        # Calculate speeds.
        # ---------------------------------------------------------

        image_x_speed = self.base_speed(
            error_x,
            gain_x
        )

        image_y_speed = self.base_speed(
            error_y,
            gain_y
        )

        # ---------------------------------------------------------
        # Anticipation boost.
        # ---------------------------------------------------------

        image_x_speed = self.apply_turn_boost(
            image_x_speed,
            error_x,
            self.error_rate_x
        )

        image_y_speed = self.apply_turn_boost(
            image_y_speed,
            error_y,
            self.error_rate_y
        )

        # ---------------------------------------------------------
        # Camera -> drone axes.
        # ---------------------------------------------------------

        drone_y = (
            self.image_x_to_drone_y
            * image_x_speed
        )

        drone_x = (
            self.image_y_to_drone_x
            * image_y_speed
        )

        # ---------------------------------------------------------
        # Emergency edge correction.
        # ---------------------------------------------------------

        if self.filtered_x < 0.10:

            drone_y = abs(drone_y)
            drone_y = max(
                drone_y,
                1.20
            )

        elif self.filtered_x > 0.90:

            drone_y = -abs(drone_y)
            drone_y = min(
                drone_y,
                -1.20
            )

        if self.filtered_y < 0.10:

            drone_x = abs(drone_x)
            drone_x = max(
                drone_x,
                1.20
            )

        elif self.filtered_y > 0.90:

            drone_x = -abs(drone_x)
            drone_x = min(
                drone_x,
                -1.20
            )

        # ---------------------------------------------------------
        # Combined speed limit.
        # ---------------------------------------------------------

        drone_x, drone_y = (
            self.limit_horizontal_speed(
                drone_x,
                drone_y,
                max_speed
            )
        )

        # ---------------------------------------------------------
        # Command.
        # ---------------------------------------------------------

        cmd = Twist()

        cmd.linear.x = drone_x
        cmd.linear.y = drone_y
        cmd.linear.z = 0.0

        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)

        self.last_command_x = drone_x
        self.last_command_y = drone_y

        mode = 'TURN' if in_turn else 'NORMAL'

        self.get_logger().info(
            f'{mode} '
            f'car=({self.filtered_x:.2f},'
            f'{self.filtered_y:.2f}) '
            f'err=({error_x:.2f},'
            f'{error_y:.2f}) '
            f'derr=({self.error_rate_x:.2f},'
            f'{self.error_rate_y:.2f}) '
            f'cmd=({drone_x:.2f},'
            f'{drone_y:.2f})',
            throttle_duration_sec=0.5
        )

    # =============================================================
    # MAIN LOOP
    # =============================================================

    def control_loop(self):

        now = time.monotonic()

        if (self.detected and self.last_seen > 0.0 and
                now - self.last_seen > self.detection_timeout):
            self.detected = False

        # ---------------------------------------------------------
        # CAR LOST
        # ---------------------------------------------------------

        if not self.detected:

            time_since_seen = (
                now - self.last_seen
            )

            # Brief dropout:
            # keep moving in the last direction, but decay it.
            #
            # This is particularly useful when the Prius passes
            # through a difficult turn frame.

            if (
                self.last_seen > 0.0
                and time_since_seen
                < self.hold_after_loss
            ):

                if (
                    not self.last_mode_in_turn
                    and time_since_seen < self.straight_loss_coast
                ):
                    decay = 1.0
                elif not self.last_mode_in_turn:
                    remaining = max(
                        0.0,
                        self.hold_after_loss - self.straight_loss_coast,
                    )
                    decay = (
                        max(
                            0.0,
                            1.0 - (
                                time_since_seen - self.straight_loss_coast
                            ) / max(remaining, 0.01),
                        )
                        if remaining > 0.0
                        else 0.0
                    )
                else:
                    decay = max(
                        0.0,
                        1.0 - time_since_seen / self.hold_after_loss,
                    )

                cmd = Twist()

                cmd.linear.x = (
                    self.last_command_x
                    * decay
                )

                cmd.linear.y = (
                    self.last_command_y
                    * decay
                )

                cmd.linear.z = 0.0

                self.cmd_pub.publish(cmd)

                return

            # Long loss -> stop.
            self.publish_hold()

            self.get_logger().warn(
                'Car lost -> HOLD',
                throttle_duration_sec=1.5
            )

            return

        # ---------------------------------------------------------
        # CAR FOUND
        # ---------------------------------------------------------

        self.follow()


def main(args=None):

    rclpy.init(args=args)

    node = DroneFollowController()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
