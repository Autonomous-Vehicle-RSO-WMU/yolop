#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

# Assumes CARLA ros bridge already running.

def generate_launch_description():
    return LaunchDescription([
        Node(package='av_sim_core', executable='spawn_actors', name='spawn_actors', output='screen'),
        Node(package='av_sim_core', executable='sensor_fusion', name='sensor_fusion', output='screen'),
        Node(package='av_sim_core', executable='vision_detector', name='vision_detector', output='screen'),
        Node(package='av_sim_core', executable='stop_controller', name='stop_controller', output='screen'),
        Node(package='av_sim_core', executable='pure_pursuit', name='pure_pursuit', output='screen'),
    ])
