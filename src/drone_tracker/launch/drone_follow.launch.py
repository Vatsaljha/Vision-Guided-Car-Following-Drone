from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    car_detector = Node(
        package='drone_tracker',
        executable='car_detector',
        name='car_detector',
        output='screen'
    )

    drone_follow_controller = Node(
        package='drone_tracker',
        executable='drone_follow_controller',
        name='drone_follow_controller',
        output='screen'
    )

    return LaunchDescription([
        car_detector,
        drone_follow_controller
    ])
