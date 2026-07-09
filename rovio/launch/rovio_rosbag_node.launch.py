import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    rovio_share_dir = get_package_share_directory('rovio')

    rosbag_path_arg = DeclareLaunchArgument(
        'rosbag_path',
        default_value='/aas/aircraft_ws/src/vicon_room2/V2_01_easy/V2_01_easy_ros2',
        description='Absolute path to the ROS 2 bag directory to process'
    )

    return LaunchDescription([
        rosbag_path_arg,
        Node(
            package='rovio',
            executable='rovio_rosbag_loader',
            name='rovio',
            output='screen',
            parameters=[
                {'filter_config': os.path.join(rovio_share_dir, 'cfg', 'rovio.info')},
                {'camera0_config': os.path.join(rovio_share_dir, 'cfg', 'euroc_cam0.yaml')},
                {'camera1_config': os.path.join(rovio_share_dir, 'cfg', 'euroc_cam1.yaml')},
                {'rosbag_path': LaunchConfiguration('rosbag_path')},
                {'imu_topic_name': '/imu0'},
                {'cam0_topic_name': '/cam0/image_raw'},
                {'cam1_topic_name': '/cam1/image_raw'}
            ]
        )
    ])
