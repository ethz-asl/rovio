from launch import LaunchDescription
from launch_ros.actions import Node

# To use this launch file, add option '-mno-avx512f' *after* '-march=native' in rovio/CMakeLists.txt

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rovio',
            executable='rovio_node',
            name='rovio',
            output='screen',
            prefix=['valgrind --leak-check=full '] 
        )
    ])
