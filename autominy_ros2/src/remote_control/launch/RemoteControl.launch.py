#!/usr/bin/env python3
"""
ROS 2 launch para el nodo remote_control.
Convierte los remapeos y parámetros que había en el XML de ROS 1.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    remote_control = Node(
        package='remote_control',
        executable='remote_control_node',
        name='remote_control',
        output='screen',
        # Si algún día quieres depurar con gdb, descomenta la línea siguiente:
        # prefix=['gnome-terminal', '--', 'gdb', '--args'],
        parameters=[{
            'max_speed': 0.3,             # igual que en tu XML
        }],
        remappings=[
            ('speed',    '/actuators/speed_normalized'),
            ('steering', '/actuators/steering_normalized'),
        ]
    )

    return LaunchDescription([remote_control])
