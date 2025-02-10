from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription, SetLaunchConfiguration
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

import os

def generate_launch_description():
    project_root = '/workspaces/phyto2025'
    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim')
    gz_launch_path = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])

    declare_world = DeclareLaunchArgument(
        'world',
        default_value='empty',
        choices=['frc2025', 'empty'],
        description='World to load into Gazebo'
    )


    
    gazebo_layer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_path),
        launch_arguments={
            'gz_args': ["empty.sdf"],
            'on_exit_shutdown': 'True'
        }.items(),
    )




    load_nodes = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        parameters=[{
            'world': 'empty',
            'file': PathJoinSubstitution([project_root, 'src/phyto_description/urdf/robot.urdf'])
            # 'file': 'empty.sdf'
        }]
    )


    return LaunchDescription([
        declare_world,
        SetLaunchConfiguration(name='world_file', value=[LaunchConfiguration('world'), TextSubstitution(text='.sdf')]),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', PathJoinSubstitution([project_root, 'src'])),
        gazebo_layer,
        load_nodes
    ])