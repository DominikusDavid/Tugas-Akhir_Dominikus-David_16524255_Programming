import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Mendeklarasikan apakah kita akan menggunakan waktu simulasi
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Mendapatkan path package dan memproses file URDF
    package_name = 'my_robot_description'  # Ganti dengan nama package Anda
    pkg_path = get_package_share_directory(package_name)  # Mendapatkan direktori package
    xacro_file = os.path.join(pkg_path, 'urdf', 'my_robot.urdf.xacro')  # Path ke file URDF
    robot_description_config = xacro.process_file(xacro_file)  # Memproses file URDF menggunakan xacro

    # Membuat node robot_state_publisher
    params = {
        'robot_description': robot_description_config.toxml(),  # Mengonversi deskripsi robot ke format XML
        'use_sim_time': use_sim_time  # Menambahkan parameter untuk waktu simulasi
    }
    node_robot_state_publisher = Node(
        package='robot_state_publisher',  # Nama package untuk robot_state_publisher
        executable='robot_state_publisher',  # Nama executable untuk node
        output='screen',  # Mengarahkan output ke layar
        parameters=[params]  # Menambahkan parameter yang telah didefinisikan
    )

    # Membuat node joint_state_publisher_gui
    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',  # Nama package untuk joint_state_publisher_gui
        executable='joint_state_publisher_gui'  # Nama executable untuk node
    )

    # Membuat node RViz
    rviz_config_path = os.path.join(pkg_path, 'rviz', 'rviz_config.rviz')  # Path ke file konfigurasi RViz
    node_rviz = Node(
        package='rviz2',  # Nama package untuk RViz
        executable='rviz2',  # Nama executable untuk node
        output='screen',  # Mengarahkan output ke layar
        arguments=['-d', rviz_config_path]  # Menambahkan argumen untuk file konfigurasi RViz
    )

    # Mengembalikan deskripsi peluncuran
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',  # Nama argumen untuk waktu simulasi
            default_value='false',  # Nilai default untuk argumen
            description='Use sim time if true'  # Deskripsi argumen
        ),
        node_robot_state_publisher,  # Menambahkan node robot_state_publisher ke deskripsi peluncuran
        node_joint_state_publisher_gui,  # Menambahkan node joint_state_publisher_gui ke deskripsi peluncuran
        node_rviz  # Menambahkan node RViz ke deskripsi peluncuran
    ])
