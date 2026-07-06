#!/usr/bin/env python3

# Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
# SPDX-License-Identifier: Apache-2.0

import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    """Generate a launch description for lanelet2_map_server."""

    remappable_topics = [
        DeclareLaunchArgument("nav_sat_fix_topic", default_value="~/gps/fix", description="NavSatFix topic for map auto-loading"),
    ]

    args = [
        DeclareLaunchArgument("name", default_value="lanelet2_map_server", description="node name"),
        DeclareLaunchArgument("namespace", default_value="", description="node namespace"),
        DeclareLaunchArgument(
            "params",
            default_value=os.path.join(get_package_share_directory("lanelet2_map_server"), "config", "params.yml"),
            description="path to parameter file",
        ),
        DeclareLaunchArgument(
            "log_level", default_value="info", description="ROS logging level (debug, info, warn, error, fatal)"
        ),
        DeclareLaunchArgument("use_sim_time", default_value="false", description="use simulation clock"),
        DeclareLaunchArgument(
            "map_filepath",
            default_value="",
            description="Path to Lanelet2 map; overrides params file if set",
        ),
        DeclareLaunchArgument(
            "origin_lat",
            default_value="",
            description="Latitude of origin of Lanelet2 map; overrides params file if set",
        ),
        DeclareLaunchArgument(
            "origin_lon",
            default_value="",
            description="Longitude of origin of Lanelet2 map; overrides params file if set",
        ),
        *remappable_topics,
    ]

    def launch_setup(context, *_args, **_kwargs):
        parameters = [LaunchConfiguration("params")]

        map_filepath = LaunchConfiguration("map_filepath").perform(context)
        if map_filepath:
            parameters.append({"map_filepath": map_filepath})

        origin_lat = LaunchConfiguration("origin_lat").perform(context)
        if origin_lat:
            parameters.append({"origin_lat": float(origin_lat)})

        origin_lon = LaunchConfiguration("origin_lon").perform(context)
        if origin_lon:
            parameters.append({"origin_lon": float(origin_lon)})

        return [
            Node(
                package="lanelet2_map_server",
                executable="lanelet2_map_server",
                namespace=LaunchConfiguration("namespace"),
                name=LaunchConfiguration("name"),
                parameters=parameters,
                arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
                remappings=[(la.default_value[0].text, LaunchConfiguration(la.name)) for la in remappable_topics],
                output="screen",
                emulate_tty=True,
            )
        ]

    return LaunchDescription(
        [
            *args,
            SetParameter("use_sim_time", LaunchConfiguration("use_sim_time")),
            OpaqueFunction(function=launch_setup),
        ]
    )
