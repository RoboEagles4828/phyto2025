from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    IncludeLaunchDescription,
    SetLaunchConfiguration,
    RegisterEventHandler,
    EmitEvent,
    LogInfo,
)
from launch.substitutions import (
    PathJoinSubstitution,
    LaunchConfiguration,
    TextSubstitution,
)
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import xacro

import os


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_ros2_control = LaunchConfiguration("use_ros2_control")
    load_controllers = LaunchConfiguration("load_controllers")
    forward_command_controllers = LaunchConfiguration("forward_command_controller")
    # namespace = LaunchConfiguration("namespace")
    hardware_plugin = LaunchConfiguration("hardware_plugin")

    # Convert robot description xacro to working xml
    pkg_phyto_description = get_package_share_directory("phyto_description")
    phyto_description_xml = xacro.process_file(
        os.path.join(pkg_phyto_description, "urdf", "robot.urdf.xacro")
    ).toxml()

    pkg_phyto_bringup = get_package_share_directory("phyto_bringup")
    phyto_controllers = os.path.join(pkg_phyto_bringup, "config", "controllers.yaml")

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": phyto_description_xml,
                # "use_sim_time": True,
                "publish_frequency": 50.0,
            }
        ],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {
                "robot_description": phyto_description_xml,
                # "use_sim_time": True,
            },
            phyto_controllers,
        ],
        output="both",
    )
    control_node_require = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=control_node,
            on_exit=[
                LogInfo(msg="Listener exited; tearing down entire system."),
                EmitEvent(event=Shutdown()),
            ],
        )
    )

    # Starts ROS2 Control Joint State Broadcastertrue"
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", ["/controller_manager"]],
    )

    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", ["/controller_manager"]],
        parameters=[
            {
                "robot_description": phyto_description_xml,
                "use_sim_time": True,
            },
            phyto_controllers,
        ],
    )

    joint_trajectory_teleop = Node(
        package="joint_trajectory_teleop",
        # namespace=namespace,
        executable="joint_trajectory_teleop",
        name="joint_trajectory_teleop",
        parameters=[{"use_sim_time": use_sim_time}],
    )


    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time", default_value="true", description="Use sim time or not"
            ),
            node_robot_state_publisher,
            control_node,
            control_node_require,
            joint_state_broadcaster_spawner,
            # joint_trajectory_controller_spawner,
            # joint_trajectory_teleop,
        ]
    )
