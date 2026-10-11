#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates


def clamp(value, low, high):
    return max(low, min(high, value))


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


def angle_error(target, current):
    return math.atan2(
        math.sin(target - current),
        math.cos(target - current)
    )


class CinematicFollowController(Node):

    def __init__(self):
        super().__init__('cinematic_follow_controller')

        self.declare_parameter('orbit_enabled', True)
        self.declare_parameter('orbit_radius', 5.0)
        self.declare_parameter('orbit_speed_deg_s', 9.0)
        self.declare_parameter('max_horizontal_speed', 2.2)
        self.declare_parameter('max_acceleration', 1.5)
        self.declare_parameter('target_altitude', 8.0)

        self.follow_distance = 4.0
        self.side_offset = 2.5

        self.position_gain = 0.8
        self.target_smoothing_time = 0.28

        self.yaw_gain = 1.5
        self.max_yaw_rate = math.radians(50.0)

        self.altitude_gain = 0.6
        self.max_vertical_speed = 0.35

        self.car = None
        self.car_time = 0.0
        self.drone = None
        self.drone_time = 0.0

        self.target_x = None
        self.target_y = None

        self.orbit_angle = 0.0
        self.orbit_initialized = False

        self.previous_world_vx = 0.0
        self.previous_world_vy = 0.0
        self.previous_time = time.monotonic()

        self.cmd_pub = self.create_publisher(
            Twist, '/drone/cmd_vel', 10
        )

        self.car_sub = self.create_subscription(
            Odometry,
            '/prius/ground_truth',
            self.car_callback,
            10
        )

        self.models_sub = self.create_subscription(
            ModelStates,
            '/model_states',
            self.models_callback,
            10
        )

        self.timer = self.create_timer(1.0 / 30.0, self.control_loop)

        self.get_logger().info(
            'Cinematic follower started: smooth tracking + optional 360 orbit'
        )
        self.get_logger().info(
            'Horizontal speed cap: 2.2 m/s; orbit: 9 degrees/second'
        )

    def car_callback(self, msg):
        p = msg.pose.pose.position
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.car = (p.x, p.y, p.z, yaw)
        self.car_time = time.monotonic()

    def models_callback(self, msg):
        try:
            i = msg.name.index('drone')
        except ValueError:
            return

        p = msg.pose[i].position
        yaw = yaw_from_quaternion(msg.pose[i].orientation)
        self.drone = (p.x, p.y, p.z, yaw)
        self.drone_time = time.monotonic()

    def publish_hold(self):
        self.cmd_pub.publish(Twist())
        self.previous_world_vx = 0.0
        self.previous_world_vy = 0.0

    def control_loop(self):
        now = time.monotonic()
        dt = clamp(now - self.previous_time, 0.005, 0.08)
        self.previous_time = now

        if (
            self.car is None
            or self.drone is None
            or now - self.car_time > 0.75
            or now - self.drone_time > 0.75
        ):
            self.publish_hold()
            self.target_x = None
            self.target_y = None
            return

        car_x, car_y, _, car_yaw = self.car
        drone_x, drone_y, drone_z, drone_yaw = self.drone

        orbit_enabled = bool(self.get_parameter('orbit_enabled').value)
        orbit_radius = float(self.get_parameter('orbit_radius').value)
        orbit_speed = math.radians(
            float(self.get_parameter('orbit_speed_deg_s').value)
        )
        max_speed = float(
            self.get_parameter('max_horizontal_speed').value
        )
        max_acceleration = float(
            self.get_parameter('max_acceleration').value
        )
        target_altitude = float(
            self.get_parameter('target_altitude').value
        )

        if orbit_enabled:
            # Keep the orbit relative to the Prius heading.
            # Start 35 degrees into the front-left quarter shot.
            if not self.orbit_initialized:
                self.orbit_angle = math.radians(35.0)
                self.orbit_initialized = True

            self.orbit_angle = math.atan2(
                math.sin(self.orbit_angle + orbit_speed * dt),
                math.cos(self.orbit_angle + orbit_speed * dt)
            )

            world_orbit_angle = car_yaw + self.orbit_angle

            desired_x = (
                car_x + orbit_radius * math.cos(world_orbit_angle)
            )
            desired_y = (
                car_y + orbit_radius * math.sin(world_orbit_angle)
            )
            mode = 'ORBIT_FRONT_QUARTER'

        else:
            # Return to a rear three-quarter trailing shot.
            forward_x = math.cos(car_yaw)
            forward_y = math.sin(car_yaw)
            left_x = -math.sin(car_yaw)
            left_y = math.cos(car_yaw)

            desired_x = (
                car_x
                - self.follow_distance * forward_x
                + self.side_offset * left_x
            )
            desired_y = (
                car_y
                - self.follow_distance * forward_y
                + self.side_offset * left_y
            )
            mode = 'FOLLOW'

        # Smooth the desired camera-drone position.
        if self.target_x is None:
            self.target_x = desired_x
            self.target_y = desired_y
            target_vx = 0.0
            target_vy = 0.0
        else:
            old_x = self.target_x
            old_y = self.target_y

            alpha = 1.0 - math.exp(
                -dt / self.target_smoothing_time
            )

            self.target_x += alpha * (desired_x - self.target_x)
            self.target_y += alpha * (desired_y - self.target_y)

            target_vx = (self.target_x - old_x) / dt
            target_vy = (self.target_y - old_y) / dt

        # Position correction plus feed-forward target movement.
        world_vx = target_vx + self.position_gain * (
            self.target_x - drone_x
        )
        world_vy = target_vy + self.position_gain * (
            self.target_y - drone_y
        )

        # Horizontal speed limit.
        speed = math.hypot(world_vx, world_vy)
        if speed > max_speed:
            scale = max_speed / speed
            world_vx *= scale
            world_vy *= scale

        # Acceleration limit for a more cinematic feel.
        dvx = world_vx - self.previous_world_vx
        dvy = world_vy - self.previous_world_vy
        change = math.hypot(dvx, dvy)
        max_change = max_acceleration * dt

        if change > max_change:
            scale = max_change / change
            world_vx = self.previous_world_vx + dvx * scale
            world_vy = self.previous_world_vy + dvy * scale

        self.previous_world_vx = world_vx
        self.previous_world_vy = world_vy

        # The existing flight controller expects body-relative velocity.
        c = math.cos(drone_yaw)
        s = math.sin(drone_yaw)
        body_vx = c * world_vx + s * world_vy
        body_vy = -s * world_vx + c * world_vy

        # Face toward the Prius throughout the orbit.
        desired_yaw = math.atan2(car_y - drone_y, car_x - drone_x)
        yaw_rate = clamp(
            self.yaw_gain * angle_error(desired_yaw, drone_yaw),
            -self.max_yaw_rate,
            self.max_yaw_rate
        )

        vz = clamp(
            self.altitude_gain * (target_altitude - drone_z),
            -self.max_vertical_speed,
            self.max_vertical_speed
        )

        cmd = Twist()
        cmd.linear.x = body_vx
        cmd.linear.y = body_vy
        cmd.linear.z = vz
        cmd.angular.z = yaw_rate
        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f'{mode} drone=({drone_x:.1f},{drone_y:.1f},{drone_z:.1f}) '
            f'target=({self.target_x:.1f},{self.target_y:.1f}) '
            f'speed={math.hypot(world_vx, world_vy):.2f} '
            f'yaw_rate={yaw_rate:.2f}',
            throttle_duration_sec=1.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = CinematicFollowController()

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
