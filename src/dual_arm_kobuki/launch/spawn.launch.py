#!/usr/bin/env python3
"""
Milestone 1: bring up the manipulation world, publish the robot description,
spawn the dual-arm Kobuki, and bridge sensors back into ROS 2.

    ros2 launch dual_arm_kobuki spawn.launch.py

Optional args:
    gui:=false        headless (server only)
    rviz:=true        also open RViz2
    world:=<name>     load a different .sdf from worlds/
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory("dual_arm_kobuki")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")

    xacro_file = os.path.join(pkg_share, "urdf", "robot.urdf.xacro")
    bridge_config = os.path.join(pkg_share, "config", "bridge.yaml")

    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    world = LaunchConfiguration("world")
    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_args = [
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("world", default_value="manipulation_task"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("home_pose", default_value="true"),
    ]

    # Let Gazebo find anything we ship in the package share.
    set_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            pkg_share,
            os.pathsep,
            os.path.join(pkg_share, "worlds"),
            os.pathsep,
            os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
        ],
    )

    # -r starts unpaused; -s server only when gui:=false; -v 3 = info logging.
    gz_args = PythonExpression([
        "'",
        os.path.join(pkg_share, "worlds", ""),
        "' + '",
        world,
        "' + '.sdf -r -v 3' + ('' if '",
        gui,
        "' == 'true' else ' -s --headless-rendering')",
    ])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": gz_args}.items(),
    )

    # ParameterValue(..., value_type=str) is REQUIRED. Without it the launch
    # system tries to infer the parameter type from the expanded URDF string,
    # fails, and robot_state_publisher never starts - which means
    # /robot_description is never published and the spawn silently does
    # nothing while the world still loads normally.
    robot_description = ParameterValue(
        Command(["xacro ", xacro_file]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_dual_arm_kobuki",
        output="screen",
        arguments=[
            "-topic", "/robot_description",
            "-name", "dual_arm_kobuki",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.02",
            "-Y", "0.0",
        ],
    )

    # Give gz-sim a few seconds to finish loading the world and advertise its
    # spawn service before create tries to call it.
    delayed_spawn = TimerAction(period=4.0, actions=[spawn_robot])

    # Drive the arms to the home pose once everything is up. Without this the
    # controllers hold every joint at 0, which is arms-straight-up and reads as
    # a single fused rod rather than a 4-DOF arm.
    go_home = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="dual_arm_kobuki",
                executable="pose_cmd.py",
                name="go_home",
                output="screen",
                arguments=["home"],
                condition=IfCondition(LaunchConfiguration("home_pose")),
            )
        ],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        output="screen",
        parameters=[
            {"config_file": bridge_config, "use_sim_time": use_sim_time}
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(rviz),
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        declare_args
        + [
            set_resource_path,
            gazebo,
            robot_state_publisher,
            delayed_spawn,
            bridge,
            go_home,
            rviz_node,
        ]
    )
