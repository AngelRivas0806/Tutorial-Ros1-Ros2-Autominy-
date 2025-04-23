from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bno055_usb_stick',
            executable='bno055_usb_stick_node',
            name='bno055',
            output='screen',
            parameters=[{
                'device': '/dev/ttyACM0'  # actualizamos el puerto real
            }]
        )
    ])
