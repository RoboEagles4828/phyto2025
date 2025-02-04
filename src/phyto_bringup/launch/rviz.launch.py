from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.event_handlers import OnProcessExit
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
import os

def generate_launch_description():
    bringup_pkg_path = os.path.join(get_package_share_directory('phyto_bringup'))
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    enable_rviz = LaunchConfiguration('enable_rviz')
    enable_joint_state_publisher = LaunchConfiguration('enable_joint_state_publisher')
    rviz_file = LaunchConfiguration('rviz_file')

    parse_script = os.path.join(bringup_pkg_path, 'scripts', 'parseRviz.py')
    parseRvizFile = ExecuteProcess(cmd=["python3", parse_script, rviz_file, namespace])

    # Starts Joint State Publisher GUI for rviz (conflicts with edna_debugger)
    joint_state_publisher_gui = Node (
        package='joint_state_publisher_gui',
        namespace=namespace,
        executable='joint_state_publisher_gui',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'publish_default_velocities':  True,
        }],
        condition=IfCondition(enable_joint_state_publisher),
    )

    rviz2 = Node(
        package='rviz2',
        name='rviz2',
        namespace=namespace,
        executable='rviz2',
        parameters=[{ 'use_sim_time': use_sim_time }],
        output='screen',
        arguments=[["-d"], [rviz_file]],
        condition=IfCondition(enable_rviz)
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        DeclareLaunchArgument(
            'namespace',
            default_value='default',
            description='The namespace of nodes and links'),
        DeclareLaunchArgument(
            'enable_rviz',
            default_value='true',
            description='enables rviz'),
        DeclareLaunchArgument(
            'rviz_file',
            default_value='',
            description='The config file for rviz'),
        DeclareLaunchArgument(
            'enable_joint_state_publisher',
            default_value='false',
            description='enables the joint state publisher tool'),
        parseRvizFile,
        joint_state_publisher_gui,
        rviz2
    ])