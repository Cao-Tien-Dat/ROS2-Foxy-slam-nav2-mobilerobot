import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Bao gom file launch robot_state_publisher cung cap boi package cua minh. Bat buoc phai kich hoat sim time.
    # !!! HAY DAM BAO DA DAT DUNG TEN PACKAGE !!!
    
    package_name = 'bena'  # <-- DOI TEN PACKAGE TAI DAY

    # Bao gom launch file robot_state_publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Bao gom launch file Gazebo tu package gazebo_ros
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    )

    # Khoi dong node spawn_entity tu package gazebo_ros. Ten entity khong quan trong neu chi co mot robot.
    spawn_entity = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'bena'],
        output='screen'
    )

    # Khoi dong tat ca cac thanh phan tren
    return LaunchDescription([
        rsp,           # Launch robot_state_publisher
        gazebo,        # Launch Gazebo
        spawn_entity,  # Khoi tao robot trong Gazebo
    ])
