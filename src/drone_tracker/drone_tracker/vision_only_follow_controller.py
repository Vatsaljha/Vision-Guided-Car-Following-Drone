#!/usr/bin/env python3

import math
import time
from collections import deque

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray


def clamp(value, low, high):
    return max(low, min(high, value))


class VisionOnlyFollowController(Node):

    def __init__(self):
        super().__init__('vision_only_follow_controller')

        # All target information comes from the camera detector.
        # No /prius/ground_truth or /model_states subscription.
        self.detection_sub = self.create_subscription(
            Float32MultiArray,
            '/drone/gimbal_car_detection',
            self.detection_callback,
            10
        )

        # Joint feedback provides the actual camera pan angle.
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            '/drone/cmd_vel',
            10
        )

        # Require consistent detections, not one isolated contour.
        self.detection_history = deque(maxlen=5)
        self.required_hits = 3
        self.locked = False

        self.car_x = 0.5
        self.car_y = 0.5
        self.car_w = 0.0
        self.car_h = 0.0
        self.score = 0.0
        self.last_seen = 0.0

        self.pan = 0.0
        self.pan_received = False

        # Conservative initial visual-servo limits.
        self.target_box_size = 0.18
        self.size_gain = 2.0
        self.max_forward = 0.60
        self.max_reverse = 0.30

        self.pan_gain = 0.9
        self.max_yaw_rate = 0.45

        self.loss_hold_time = 0.65
        self.last_log_time = 0.0
        self.was_locked = False

        self.timer = self.create_timer(
            0.1,
            self.control_loop
        )

        self.get_logger().info(
            'VISION-ONLY FOLLOWER started'
        )
        self.get_logger().info(
            'Target input: /drone/gimbal_car_detection'
        )
        self.get_logger().info(
            'No car ground-truth or Gazebo model-state input'
        )

    def joint_callback(self, msg):
        for i, name in enumerate(msg.name):
            if name == 'gimbal_pan_joint' and i < len(msg.position):
                self.pan = float(msg.position[i])
                self.pan_received = True
                break

    def detection_callback(self, msg):
        if len(msg.data) < 6:
            return

        detected = float(msg.data[0]) > 0.5
        x = float(msg.data[1])
        y = float(msg.data[2])
        w = float(msg.data[3])
        h = float(msg.data[4])
        score = float(msg.data[5])

        valid = (
            detected
            and score >= 0.30
            and 0.0 <= x <= 1.0
            and 0.0 <= y <= 1.0
            and w >= 0.02
            and h >= 0.02
        )

        self.detection_history.append(valid)

        if valid:
            self.car_x = x
            self.car_y = y
            self.car_w = w
            self.car_h = h
            self.score = score
            self.last_seen = time.monotonic()

        hit_count = sum(self.detection_history)

        if hit_count >= self.required_hits:
            self.locked = True
        elif (
            self.last_seen == 0.0
            or time.monotonic() - self.last_seen > self.loss_hold_time
        ):
            self.locked = False

    def publish_hold(self):
        self.cmd_pub.publish(Twist())

    def control_loop(self):
        now = time.monotonic()

        if not self.pan_received:
            self.publish_hold()
            return

        age = now - self.last_seen if self.last_seen else float('inf')

        if age > self.loss_hold_time:
            self.locked = False

        if not self.locked or age > self.loss_hold_time:
            self.publish_hold()

            if self.was_locked:
                self.get_logger().warn(
                    'VISUAL TARGET LOST: holding; gimbal search continues'
                )
            self.was_locked = False
            self.detection_history.clear()
            return

        # The gimbal's pan joint estimates the target bearing relative
        # to the drone's body heading. Rotate gently toward the target.
        yaw_rate = clamp(
            self.pan_gain * self.pan,
            -self.max_yaw_rate,
            self.max_yaw_rate
        )

        # Use apparent box size as a rough relative-distance cue.
        # This is not a metric range measurement.
        apparent_size = max(self.car_h, 0.65 * self.car_w)
        size_error = self.target_box_size - apparent_size

        if abs(size_error) < 0.015:
            forward = 0.0
        else:
            forward = clamp(
                self.size_gain * size_error,
                -self.max_reverse,
                self.max_forward
            )

        cmd = Twist()
        cmd.linear.x = forward
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.z = yaw_rate

        self.cmd_pub.publish(cmd)

        if not self.was_locked:
            self.get_logger().info(
                'VISUAL TARGET ACQUIRED: following using camera data'
            )

        if now - self.last_log_time > 1.0:
            self.get_logger().info(
                f'VISION FOLLOW '
                f'box=({self.car_x:.2f},{self.car_y:.2f}) '
                f'size={apparent_size:.2f} '
                f'confidence={self.score:.2f} '
                f'pan={math.degrees(self.pan):+.1f}deg '
                f'cmd_x={forward:.2f} '
                f'cmd_yaw={yaw_rate:.2f}'
            )
            self.last_log_time = now

        self.was_locked = True


def main(args=None):
    rclpy.init(args=args)
    node = VisionOnlyFollowController()

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
