#!/usr/bin/env python3
"""
UGV 仿真 Launch 文件
启动: UGV 控制器 + UGV 传感器发布 + Gazebo 桥接
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    # --- 参数 ---
    use_gazebo = LaunchConfiguration('use_gazebo', default='true')
    home_lat = LaunchConfiguration('home_lat', default='30.0')
    home_lon = LaunchConfiguration('home_lon', default='120.0')

    # --- UGV 控制器节点 ---
    ugv_controller = Node(
        package='ugv_sim',
        executable='ugv_controller',
        name='ugv_controller',
        output='screen',
        parameters=[{
            'home_lat': home_lat,
            'home_lon': home_lon,
            'home_alt': 0.0,
            'max_linear_speed': 3.0,
            'max_angular_speed': 2.0,
            'update_rate': 20.0,
        }],
    )

    # --- UGV 传感器节点 ---
    ugv_sensor = Node(
        package='ugv_sim',
        executable='ugv_sensor_pub',
        name='ugv_sensor_pub',
        output='screen',
        parameters=[{
            'lidar_lines': 16,
            'lidar_range': 100.0,
            'lidar_hz': 10.0,
            'camera_width': 1280,
            'camera_height': 720,
            'camera_hz': 30.0,
            'imu_hz': 100.0,
        }],
    )

    # --- 键盘遥控节点 ---
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e',
        remappings=[('/cmd_vel', '/ugv/cmd_vel')],
    )

    # --- Gazebo 启动 ---
    gazebo = ExecuteProcess(
        condition=IfCondition(use_gazebo),
        cmd=['gz', 'sim', '-r',
             '$(find ugv_sim)/worlds/empty_with_ground.world'],
        output='screen',
    )

    # --- Gazebo-ROS2 桥接 (UGV 位姿) ---
    gz_bridge_pose = Node(
        condition=IfCondition(use_gazebo),
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ugv_gz_pose_bridge',
        arguments=[
            '/model/ugv_chassis/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose',
        ],
        output='screen',
    )

    # --- 参数声明 ---
    declare_use_gazebo = DeclareLaunchArgument(
        'use_gazebo', default_value='true',
        description='是否启动 Gazebo 仿真')
    declare_home_lat = DeclareLaunchArgument(
        'home_lat', default_value='30.0',
        description='起点纬度')
    declare_home_lon = DeclareLaunchArgument(
        'home_lon', default_value='120.0',
        description='起点经度')

    return LaunchDescription([
        declare_use_gazebo,
        declare_home_lat,
        declare_home_lon,
        gazebo,
        gz_bridge_pose,
        ugv_controller,
        ugv_sensor,
        # teleop_node,  # 需要在有 GUI 的环境中运行
    ])
