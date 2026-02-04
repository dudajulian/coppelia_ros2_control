# Copyright 2020 ros2_control Development Team
# Modified 2026 by Julian Duda (Pen Limit)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Description:
#   Launch file for integrating CoppeliaSim with ros2_control.
#   Spawns controllers, state publishers, and custom nodes for simulation.
#   Supports both Clearpath Husky and iRobot Create 2 configurations.
#   Modified to work with CoppeliaSim's topic-based hardware interface.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Default values for the example differential drive robot
default_controller_config_path = PathJoinSubstitution([
    FindPackageShare("coppelia_ros2_control"), "config", "example_control.yaml"
    ])
default_controller_name = "platform_velocity_controller"
default_robot_description_path = PathJoinSubstitution([
    FindPackageShare("coppelia_ros2_control"), "description/urdf", "example.urdf"
    ])

def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation clock if true",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_config_path",
            default_value=default_controller_config_path,
            description="Name of the controller (match with control.yaml file)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_name",
            default_value=default_controller_name,
            description="Name of the controller (match with control.yaml file)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_description_path",
            default_value=default_robot_description_path,
            description="Name of the controller (match with control.yaml file)",
        )
    )

    # Initialize Arguments
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    use_sim_time = LaunchConfiguration("use_sim_time")
    controller_config_path = LaunchConfiguration("controller_config_path")
    controller_name = LaunchConfiguration("controller_name")
    robot_description_path = LaunchConfiguration("robot_description_path")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            robot_description_path,
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        # parameters=[robot_controllers], 
        parameters=[controller_config_path, {"use_sim_time": use_sim_time}],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
            ("/diffbot_base_controller/cmd_vel", "/cmd_vel"),
        ],
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            controller_name,
            "--param-file",
            controller_config_path,
        ],
    )

    controller_cmd_topic = PathJoinSubstitution([controller_name])
    twist_stamper_node = Node(
        package="twist_stamper",
        executable="twist_stamper",
        name="twist_stamper_node",
        output="screen",
        arguments=[
            '-r', 'cmd_vel_in:=cmd_vel',
            '-r', ['cmd_vel_out:=', controller_name, '/cmd_vel']
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    map_odom_broadcaster_node = Node(
        package="coppelia_ros2_control",
        executable="map_odom_broadcaster",
        name="map_odom_broadcaster_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )
    
    # Delay start of joint_state_broadcaster after `robot_controller`
    # TODO(anyone): This is a workaround for flaky tests. Remove when fixed.
    delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    nodes = [
        map_odom_broadcaster_node,
        twist_stamper_node,
        control_node,
        robot_state_pub_node,
        robot_controller_spawner,
        delay_joint_state_broadcaster_after_robot_controller_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)