from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, LogInfo

from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, FindExecutable

def generate_launch_description():
    node_name = LaunchConfiguration('node_name')
    ll2_map_filename = LaunchConfiguration('ll2_map_filename')

    node_name_launch_arg = DeclareLaunchArgument(
        'node_name',
        default_value='ll2_map_server'
    )

    ll2_map_filename_launch_arg = DeclareLaunchArgument(
        'll2_map_filename',
        default_value='/home/lutix/ws/src/mbs/Frankenberg.osm'
    )

    ll2_map_server_node = Node(
            package='lanelet2_map_server',
            executable='lanelet2_map_server',
            name=node_name
        )
    
    #To-Do: Substitute the service values with LaunchConfiguration arguments
    service_call = '"{map_filename: /home/lutix/ws/src/mbs/Frankenberg.osm, map_frame_id: map, origin_lat: 50.76838121996561, origin_lon: 6.102233877820072}"'

    configure_map = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' service call /',
            node_name,
            '/change_map_parameters ',
            'lanelet2_map_manager_ifs/srv/ChangeMapParams ',
            service_call
        ]],
        shell=True
    )

    return LaunchDescription([
        node_name_launch_arg,
        ll2_map_filename_launch_arg,
        ll2_map_server_node,
        RegisterEventHandler(
            OnProcessStart(
                target_action=ll2_map_server_node,
                on_start=[
                    LogInfo(msg='Calling service to initially set map!'),
                    configure_map
                ]
            )
        )
    ])