from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    rviz_config_path = os.path.join(
        os.path.expanduser('~'),
        'autominy_ros2_ws',
        'src',
        'lidar_pose_estimation',
        'rviz',
        'lidar_pose_view.rviz'
    )

    return LaunchDescription([
        # Nodo del RPLIDAR
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_node',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Express'  # IMPORTANTE: usa un modo soportado por tu hardware
            }],
            output='screen'
        ),

        # Nodo de estimación de pose
        Node(
            package='lidar_pose_estimation',
            executable='lidar_pose_estimation_node',
            name='lidar_pose_estimation',
            parameters=[{
                'max_dist': float(os.environ.get('ROS_LIDAR_CALIBRATION_MAX_DISTANCE', 0.25)),
                'execution_frequency': float(os.environ.get('ROS_LIDAR_CALIBRATION_EXECUTION_FREQUENCY', 1.0)),
                'cluster_tolerance': float(os.environ.get('ROS_LIDAR_CALIBRATION_CLUSTER_TOLERANCE', 0.02)),
                'min_cluster_size': int(os.environ.get('ROS_LIDAR_CALIBRATION_MIN_CLUSTER_SIZE', 10)),
                'max_cluster_size': int(os.environ.get('ROS_LIDAR_CALIBRATION_MAX_CLUSTER_SIZE', 100)),
                'max_reference_distance_deviation': float(os.environ.get('ROS_LIDAR_CALIBRATION_MAX_REFERENCE_DEVIATION', 0.01)),
                'roll': float(os.environ.get('ROS_LIDAR_CALIBRATION_ROLL', 0.0)),
                'pitch': float(os.environ.get('ROS_LIDAR_CALIBRATION_PITCH', 0.0)),
                'z': float(os.environ.get('ROS_LIDAR_CALIBRATION_Z', 0.0)),
                'p_ref_1_x': float(os.environ.get('ROS_LIDAR_CALIBRATION_P_REF_1_X', -0.035)),
                'p_ref_1_y': float(os.environ.get('ROS_LIDAR_CALIBRATION_P_REF_1_Y', -0.055)),
                'p_ref_2_x': float(os.environ.get('ROS_LIDAR_CALIBRATION_P_REF_2_X', -0.035)),
                'p_ref_2_y': float(os.environ.get('ROS_LIDAR_CALIBRATION_P_REF_2_Y', -0.055)),
            }],
            output='screen'
        ),

        # Lanzar RViz2 con configuración personalizada
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_config_path],
            output='screen'
        )
    ])
