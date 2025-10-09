import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression
import xacro

def generate_launch_description():
    # Declare argument
    empty_world_arg = DeclareLaunchArgument(
        'empty_world',
        default_value='false',
        description='Use empty world instead of willowgarage'
    )
    empty_world_config = LaunchConfiguration('empty_world')
    
    # Conditional world path
    world_path = PythonExpression([
        "'' if '", empty_world_config, "' == 'true' else '/usr/share/gazebo-11/worlds/willowgarage.world'"
    ])

    
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('leo_gazebo'))
    xacro_file = os.path.join(pkg_path,'description','leo.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    
    # Robot state publisher
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Spawn robot entity
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-file', os.path.join(pkg_path, 'description', 'leo.xacro'),
            '-entity', 'leo',
            '-x', '0',
            '-y', '0',
            '-z', '0.5'
        ],
        output='screen'
    )
    
    # Gazebo include
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_path}.items()
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use sim time if true'),
        
        empty_world_arg,
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
    ])