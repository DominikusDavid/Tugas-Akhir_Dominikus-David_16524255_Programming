import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Mengembalikan deskripsi peluncuran yang berisi node RPLIDAR
    return LaunchDescription([
        Node(
            package='rplidar_ros',  # Nama package yang berisi driver RPLIDAR
            executable='rplidar_composition',  # Nama executable untuk menjalankan RPLIDAR
            output='screen',  # Mengarahkan output ke layar
            parameters=[{
                # Parameter untuk konfigurasi RPLIDAR
                'serial_port': '/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0-port0',  # Port serial untuk RPLIDAR
                'frame_id': 'laser_frame',  # ID frame untuk data laser
                'angle_compensate': True,  # Mengaktifkan kompensasi sudut
                'scan_mode': 'Standard'  # Mode pemindaian yang digunakan
            }]
        )
    ])
