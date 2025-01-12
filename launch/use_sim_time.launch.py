import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = 'bena'

    # Bao gồm launch file robot_state_publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Thiết lập đường dẫn chính xác cho world 'home.world'
    gazebo_world_path = os.path.join(
        get_package_share_directory('bena'), 'worlds', 'home.world'
    )

    # Bao gồm launch file gazebo với world được chỉ định
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': gazebo_world_path}.items()
    )

    # Khởi động node spawn_entity từ package gazebo_ros
    spawn_entity = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'bena'],
        output='screen'
    )

    # Xử lý đường dẫn cho params_file của SLAM Toolbox
    params_file_path = os.path.expanduser('~/ros2_ws/src/bena/config/mapper_params_online_async.yaml')

    # Thêm ExecuteProcess để gọi lệnh ros2 launch SLAM Toolbox
    slam_toolbox_launch = ExecuteProcess(
        cmd=['ros2', 'launch', 'slam_toolbox', 'online_async_launch.py', 
             f'params_file:={params_file_path}', 'use_sim_time:=true'],
        output='screen'
    )

    # Xử lý đường dẫn cho RViz2
    rviz2_config_path = os.path.expanduser('~/ros2_ws/src/bena/config/map.rviz')

    # Gọi RViz2 với file cấu hình map.rviz
    rviz2_launch = ExecuteProcess(
        cmd=['rviz2', '-d', rviz2_config_path],
        output='screen'
    )

    # Gọi lệnh ros2 launch nav2_bringup navigation_launch.py
    nav2_launch = ExecuteProcess(
        cmd=['ros2', 'launch', 'nav2_bringup', 'navigation_launch.py', 'use_sim_time:=true'],
        output='screen'
    )

    # Khởi động tất cả các thành phần trên
    return LaunchDescription([
        rsp,  
        gazebo,  
        spawn_entity,  
        slam_toolbox_launch,  
        rviz2_launch,  
        nav2_launch, 
    ])
