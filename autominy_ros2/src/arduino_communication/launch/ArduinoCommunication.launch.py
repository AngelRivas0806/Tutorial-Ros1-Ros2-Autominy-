from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    baud_arg = DeclareLaunchArgument(
        'baud', default_value='115200', description='Arduino baud rate'
    )
    device_arg = DeclareLaunchArgument(
        'device', default_value='/dev/ttyUSB0', description='Serial device'
    )

    arduino_node = Node(
        package='arduino_communication',
        executable='arduino_communication_node',
        name='arduino',
        output='screen',
        parameters=[
            {'baud': LaunchConfiguration('baud')},
            {'device': LaunchConfiguration('device')}
        ]
    )

    return LaunchDescription([
        baud_arg,
        device_arg,
        arduino_node,
    ])
