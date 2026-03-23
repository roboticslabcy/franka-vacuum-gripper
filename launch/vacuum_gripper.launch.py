from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_ip_arg = DeclareLaunchArgument(
        "robot_ip",
        description="IP address or hostname of the Franka robot",
    )
    state_publish_rate_arg = DeclareLaunchArgument(
        "state_publish_rate",
        default_value="30.0",
        description="Rate [Hz] at which the vacuum gripper state is published",
    )

    vacuum_gripper_node = Node(
        package="franka_vacuum_gripper",
        executable="franka_vacuum_gripper_node",
        name="franka_vacuum_gripper",
        parameters=[
            {
                "robot_ip": LaunchConfiguration("robot_ip"),
                "state_publish_rate": LaunchConfiguration("state_publish_rate"),
            }
        ],
        output="screen",
    )

    return LaunchDescription([
        robot_ip_arg,
        state_publish_rate_arg,
        vacuum_gripper_node,
    ])
