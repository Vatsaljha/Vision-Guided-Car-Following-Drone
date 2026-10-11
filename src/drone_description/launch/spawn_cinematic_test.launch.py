#!/usr/bin/env python3

import os

import xacro

from ament_index_python.packages import (
    get_package_share_directory,
    get_package_prefix,
)

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
    SetEnvironmentVariable,
)

from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)

from launch_ros.actions import Node


def remove_xml_declaration(xml_text):
    """
    spawn_entity.py receives a Unicode string.
    lxml does not allow an encoding declaration
    inside a Unicode XML string.
    """

    xml_text = xml_text.lstrip()

    if xml_text.startswith("<?xml"):
        end = xml_text.find("?>")

        if end != -1:
            xml_text = xml_text[end + 2:]

    return xml_text.lstrip()


def generate_launch_description():

    # =========================================================
    # PACKAGE PATHS
    # =========================================================

    drone_package = get_package_share_directory(
        "drone_description"
    )

    drone_prefix = get_package_prefix(
        "drone_description"
    )

    prius_package = get_package_share_directory(
        "prius_description"
    )

    gazebo_package = get_package_share_directory(
        "gazebo_ros"
    )

    # =========================================================
    # GAZEBO PLUGIN PATH
    # =========================================================

    drone_plugin_dir = os.path.join(
        drone_prefix,
        "lib",
    )

    old_plugin_path = os.environ.get(
        "GAZEBO_PLUGIN_PATH",
        "",
    )

    if old_plugin_path:
        gazebo_plugin_path = (
            drone_plugin_dir
            + ":"
            + old_plugin_path
        )
    else:
        gazebo_plugin_path = drone_plugin_dir

    # =========================================================
    # GAZEBO MODEL PATH
    # =========================================================

    drone_model_dir = os.path.join(
        drone_package,
        "models",
    )

    old_model_path = os.environ.get(
        "GAZEBO_MODEL_PATH",
        "",
    )

    if old_model_path:
        gazebo_model_path = (
            drone_model_dir
            + ":"
            + old_model_path
        )
    else:
        gazebo_model_path = drone_model_dir

    # =========================================================
    # FILE PATHS
    # =========================================================

    world_file = os.path.join(
        drone_package,
        "worlds",
        "drone_follow.world",
    )

    drone_urdf = os.path.join(
        drone_package,
        "urdf",
        "drone_cinematic.urdf.xacro",
    )

    prius_urdf = os.path.join(
        prius_package,
        "urdf",
        "prius_no_lidar.urdf",
    )

    gazebo_launch = os.path.join(
        gazebo_package,
        "launch",
        "gazebo.launch.py",
    )

    # =========================================================
    # VALIDATE FILES
    # =========================================================

    if not os.path.exists(world_file):
        raise RuntimeError(
            f"World not found: {world_file}"
        )

    if not os.path.exists(drone_urdf):
        raise RuntimeError(
            f"Drone URDF not found: {drone_urdf}"
        )

    if not os.path.exists(prius_urdf):
        raise RuntimeError(
            f"Prius URDF not found: {prius_urdf}"
        )

    # =========================================================
    # PROCESS DRONE XACRO
    # =========================================================

    drone_description = xacro.process_file(
        drone_urdf
    ).toxml()

    drone_description = remove_xml_declaration(
        drone_description
    )

    # =========================================================
    # READ PRIUS URDF
    # =========================================================

    with open(
        prius_urdf,
        "r",
        encoding="utf-8",
    ) as f:

        prius_description = f.read()

    prius_description = remove_xml_declaration(
        prius_description
    )

    # =========================================================
    # LAUNCH
    # =========================================================

    return LaunchDescription([

        # -----------------------------------------------------
        # Gazebo plugin path
        # -----------------------------------------------------

        SetEnvironmentVariable(
            name="GAZEBO_PLUGIN_PATH",
            value=gazebo_plugin_path,
        ),

        # -----------------------------------------------------
        # Gazebo model path
        # -----------------------------------------------------

        SetEnvironmentVariable(
            name="GAZEBO_MODEL_PATH",
            value=gazebo_model_path,
        ),

        # -----------------------------------------------------
        # Start world
        # -----------------------------------------------------

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                gazebo_launch
            ),
            launch_arguments={
                "world": world_file,
                "verbose": "true",
            }.items(),
        ),

        # -----------------------------------------------------
        # DRONE robot_state_publisher
        #
        # Namespace prevents collision with other
        # robot_description topics.
        # -----------------------------------------------------

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace="drone",
            name="robot_state_publisher",
            output="screen",
            parameters=[
                {
                    "robot_description": drone_description,
                    "use_sim_time": True,
                }
            ],
        ),

        # -----------------------------------------------------
        # PRIUS spawn
        # -----------------------------------------------------

        TimerAction(
            period=5.0,
            actions=[

                Node(
                    package="gazebo_ros",
                    executable="spawn_entity.py",
                    name="spawn_prius",
                    output="screen",

                    arguments=[
                        "-entity",
                        "prius",

                        "-file",
                        prius_urdf,

                        "-x",
                        "0.0",

                        "-y",
                        "0.0",

                        "-z",
                        "0.20",

                        "-Y",
                        "0.0",
                    ],
                )
            ],
        ),

        # -----------------------------------------------------
        # DRONE spawn
        #
        # Notice the separate topic:
        # /drone/robot_description
        # -----------------------------------------------------

        TimerAction(
            period=7.0,
            actions=[

                Node(
                    package="gazebo_ros",
                    executable="spawn_entity.py",
                    name="spawn_drone",
                    output="screen",

                    arguments=[
                        "-entity",
                        "drone",

                        "-topic",
                        "/drone/robot_description",

                        "-x",
                        "0.0",

                        "-y",
                        "0.0",

                        "-z",
                        "12.0",

                        "-Y",
                        "0.0",
                    ],
                )
            ],
        ),

        # -----------------------------------------------------
        # LANE DETECTION
        # -----------------------------------------------------

        TimerAction(
            period=10.0,
            actions=[

                Node(
                    package="prius_adas_control",
                    executable="lane_detection",
                    name="lane_detection",
                    output="screen",
                    parameters=[
                        {
                            "use_sim_time": True,
                        }
                    ],
                )
            ],
        ),

        # -----------------------------------------------------
        # AUTONOMOUS DRIVER
        # -----------------------------------------------------

        TimerAction(
            period=12.0,
            actions=[

                Node(
                    package="prius_adas_control",
                    executable="autonomous_driver",
                    name="autonomous_driver",
                    output="screen",
                    parameters=[
                        {
                            "use_sim_time": True,
                        }
                    ],
                )
            ],
        ),
    ])