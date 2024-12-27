import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    package_name = 'my_robot_description'  # Ganti dengan nama package Anda

    # Path untuk URDF dan konfigurasi RViz
    urdf_path = os.path.join(get_package_share_directory(package_name), 'urdf', 'my_robot.urdf.xacro')
    rviz_config_path = os.path.join(get_package_share_directory(package_name), 'rviz', 'rviz_config.rviz')

    # Path untuk file world
    world_path = os.path.join(get_package_share_directory('my_robot_bringup'), 'world', 'first_world.world') 

    # Node untuk robot_state_publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_path])
        }]
    )

    # Node untuk joint_state_publisher_gui
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )

    # Node untuk RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    # Include launch file untuk robot_state_publisher
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Include launch file untuk Gazebo dengan world yang ditentukan
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_path}.items()  # Menambahkan argumen world
    )

    # Node untuk spawn robot di Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'my_bot'],
        output='screen'
    )

    # Mengembalikan LaunchDescription
    return LaunchDescription([
        rsp,
        joint_state_publisher_gui,
        gazebo,
        spawn_entity,
        rviz,
    ])
