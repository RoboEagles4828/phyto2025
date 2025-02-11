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

import os


def generate_launch_description():
    project_root = "/workspaces/phyto2025"
    pkg_ros_gz_sim = FindPackageShare("ros_gz_sim")
    gz_launch_path = PathJoinSubstitution(
        [pkg_ros_gz_sim, "launch", "gz_sim.launch.py"]
    )
    bringup_path = get_package_share_directory("phyto_bringup")

    declare_world = DeclareLaunchArgument(
        "world",
        default_value="reefscape",
        choices=["reefscape", "empty"],
        description="World to load into Gazebo",
    )

    gazebo_layer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_path),
        launch_arguments={
            "gz_args": ["gazebo/reefscape.sdf"],
            "on_exit_shutdown": "True",
            "topic": "/robot_description"
        }.items(),
    )

    load_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        parameters=[
            {
                "world": "Reefscape",
                "file": PathJoinSubstitution(
                    [project_root, "src/phyto_description/urdf/robot.urdf"]
                ),
                # 'file': 'empty.sdf'
            }
        ],
    )

    common_layer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(bringup_path, "launch", "common.launch.py")]
        ),
        launch_arguments={},
    )

    return LaunchDescription(
        [
            common_layer,
            declare_world,
            SetLaunchConfiguration(
                name="world_file",
                value=[LaunchConfiguration("world"), TextSubstitution(text=".sdf")],
            ),
            SetEnvironmentVariable(
                "GZ_SIM_RESOURCE_PATH", PathJoinSubstitution([project_root, "src"])
            ),
            gazebo_layer,
            load_robot,
        ]
    )
