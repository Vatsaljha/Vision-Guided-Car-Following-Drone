from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import FindExecutable


def generate_launch_description():

    ros2 = FindExecutable(name="ros2")

    joint_state_broadcaster = ExecuteProcess(
        cmd=[
            ros2,
            "run",
            "controller_manager",
            "spawner",
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    gimbal_position_controller = ExecuteProcess(
        cmd=[
            ros2,
            "run",
            "controller_manager",
            "spawner",
            "gimbal_position_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    return LaunchDescription([
        joint_state_broadcaster,
        gimbal_position_controller,
    ])
