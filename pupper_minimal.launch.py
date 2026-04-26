"""
Launch only the upstream Pupper control stack needed by the OAK-D demo.

This deliberately avoids the stock upstream `neural_controller/launch.py`
because that file also starts the Pi camera, Hailo detector, and upstream
person_follower_node. The upstream person follower publishes to
/person_following_cmd_vel, which is the mux input owned by this project.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declare_teleop_arg = DeclareLaunchArgument(
        name="teleop",
        default_value="True",
        description="Enable gamepad teleop on /teleop_cmd_vel.",
    )

    neural_config = ParameterFile(
        PathJoinSubstitution(
            [FindPackageShare("neural_controller"), "launch", "config.yaml"]
        ),
        allow_substs=True,
    )
    active_controller_params = {
        "controller_names": ["neural_controller", "neural_controller_three_legged"]
    }
    cmd_vel_mux_params = {"timeout_ms": 500}

    xacro_file = PathJoinSubstitution(
        [
            FindPackageShare("pupper_v3_description"),
            "description",
            "pupper_v3.urdf.xacro",
        ]
    )
    robot_description = {
        "robot_description": Command(
            [PathJoinSubstitution([FindExecutable(name="xacro")]), " ", xacro_file]
        )
    }

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[neural_config],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "30",
        ],
    )

    imu_sensor_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "imu_sensor_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "30",
        ],
    )

    neural_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "neural_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "30",
            "--inactive",
        ],
    )

    three_legged_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "neural_controller_three_legged",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "30",
            "--inactive",
        ],
    )

    cmd_vel_mux_node = Node(
        package="cmd_vel_mux",
        executable="cmd_vel_mux_node",
        parameters=[neural_config, cmd_vel_mux_params],
        output="both",
    )

    joy_linux_node = Node(
        package="joy_linux",
        executable="joy_linux_node",
        parameters=[neural_config],
        output="both",
        name="joy_linux_node",
    )

    teleop_twist_joy_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        parameters=[neural_config],
        output="both",
        condition=IfCondition(LaunchConfiguration("teleop")),
        remappings=[("cmd_vel", "teleop_cmd_vel")],
    )

    joy_util_node = Node(
        package="joy_utils",
        executable="estop_controller",
        parameters=[neural_config, active_controller_params],
        output="both",
        name="joy_util_node",
    )

    return LaunchDescription(
        [
            declare_teleop_arg,
            robot_state_publisher,
            control_node,
            joint_state_broadcaster_spawner,
            imu_sensor_broadcaster_spawner,
            neural_controller_spawner,
            three_legged_controller_spawner,
            cmd_vel_mux_node,
            joy_linux_node,
            teleop_twist_joy_node,
            joy_util_node,
        ]
    )
