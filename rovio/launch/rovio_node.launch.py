import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Find the share directory of the rovio package
    rovio_share_dir = get_package_share_directory('rovio')
    imu_topic_arg = DeclareLaunchArgument('imu_topic', default_value='imu0')
    cam0_topic_arg = DeclareLaunchArgument('cam0_topic', default_value='cam0/image_raw')
    cam1_topic_arg = DeclareLaunchArgument('cam1_topic', default_value='cam1/image_raw')
    sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')

    return LaunchDescription([
        imu_topic_arg,
        cam0_topic_arg,
        cam1_topic_arg,
        Node(
            package='rovio',
            executable='rovio_node',
            name='rovio',
            output='screen',
            parameters=[
                {'filter_config': os.path.join(rovio_share_dir, 'cfg', 'rovio.info')},
                {'camera0_config': os.path.join(rovio_share_dir, 'cfg', 'euroc_cam0.yaml')},
                {'camera1_config': os.path.join(rovio_share_dir, 'cfg', 'euroc_cam1.yaml')},
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            remappings=[
                ('imu0', LaunchConfiguration('imu_topic')),
                ('cam0/image_raw', LaunchConfiguration('cam0_topic')),
                ('cam1/image_raw', LaunchConfiguration('cam1_topic'))
            ]
        )
    ])
