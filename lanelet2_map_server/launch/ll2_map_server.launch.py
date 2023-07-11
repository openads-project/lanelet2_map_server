from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, LogInfo

from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, FindExecutable

def generate_launch_description():
    node_name = LaunchConfiguration('node_name')

    node_name_launch_arg = DeclareLaunchArgument(
        'node_name',
        default_value='ll2_map_server'
    )

    ll2_map_server_node = Node(
            package='lanelet2_map_server',
            executable='lanelet2_map_server',
            name=node_name
    )
    
    return LaunchDescription([
        node_name_launch_arg,
        ll2_map_server_node,
        RegisterEventHandler(
            OnProcessStart(
                target_action=ll2_map_server_node
            )
        )
    ])