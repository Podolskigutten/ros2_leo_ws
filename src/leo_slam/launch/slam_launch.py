from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    # Path to your SLAM params file
    slam_params_path = os.path.join(
        get_package_share_directory('leo_slam'),
        'config',
        'mapper_params_online_async.yaml'
    )
    
    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                slam_params_path,
                {'use_sim_time': False}
            ]
        )
    ])