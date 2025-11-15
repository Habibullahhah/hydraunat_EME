import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Get Package Paths
    pkg_loco_gazebo = get_package_share_directory('loco_gazebo')
    pkg_loco_description = get_package_share_directory('loco_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # 2. Configuration
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file_name = 'loco.world'

    world_arg = DeclareLaunchArgument(
        'world_name',
        default_value=world_file_name,
        description='Name of the world file to load'
    )

    # 3. Start Gazebo
    # FIX: We explicitly load the factory plugin using --verbose for debugging
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': PathJoinSubstitution([pkg_loco_gazebo, 'worlds', LaunchConfiguration('world_name')]),
            'extra_gazebo_args': '--verbose -s libgazebo_ros_factory.so' 
        }.items()
    )

    # 4. Process URDF (Robot Description)
    xacro_file = os.path.join(pkg_loco_description, 'urdf', 'loco.urdf.xacro')
    # We use Command to run xacro on the file
    robot_description_content = Command(['xacro ', xacro_file])

    # 5. Robot State Publisher (Publishes TF frames)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time
        }]
    )

    # 6. Spawn the Robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'loco_auv',
            '-z', '0.5' 
        ],
        output='screen'
    )

    # 7. Physics Control Node (Your Python Script)
    sim_control = Node(
        package='loco_gazebo',
        executable='sim_control_node.py',
        name='sim_control',
        output='screen'
    )

    return LaunchDescription([
        world_arg,
        gazebo,
        robot_state_publisher,
        spawn_entity,
        sim_control
    ])
