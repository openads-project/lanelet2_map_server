from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, LogInfo

from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, FindExecutable

def generate_launch_description():
    node_name = LaunchConfiguration('node_name')
    use_sim_time_arg = LaunchConfiguration('use_sim_time_arg')

    node_name_launch_arg = DeclareLaunchArgument(
        'node_name',
        default_value='ll2_map_server'
    )

    use_sim_time_launch_arg = DeclareLaunchArgument(
        'use_sim_time_arg',
        default_value='False'
    )

    ll2_map_server_node = Node(
            package='lanelet2_map_server',
            executable='lanelet2_map_server',
            name=node_name
    )

    sim_time_param = SetParameter(name='use_sim_time',
            value=LaunchConfiguration('use_sim_time_arg'),
            condition=LaunchConfigurationNotEquals('use_sim_time_arg', "None")
    )
    
    return LaunchDescription([
        node_name_launch_arg,
        use_sim_time_launch_arg,
        ll2_map_server_node,
        RegisterEventHandler(
            OnProcessStart(
                target_action=ll2_map_server_node,
                on_start=[
                    sim_time_param
                ]
            )
        )
    ])