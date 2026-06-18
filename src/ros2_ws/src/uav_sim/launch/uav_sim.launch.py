#!/usr/bin/env python3
"""
UAV 仿真 Launch 文件
启动: UAV 控制器 + UAV 相机仿真 + Gazebo 桥接
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
    takeoff_height = LaunchConfiguration('takeoff_height', default='10.0')

    # --- UAV 控制器节点 ---
    uav_controller = Node(
        package='uav_sim',
        executable='uav_controller',
        name='uav_controller',
        output='screen',
        parameters=[{
            'home_lat': home_lat,
            'home_lon': home_lon,
            'home_alt': 0.0,
            'takeoff_height': takeoff_height,
            'hover_duration': 30.0,
            'update_rate': 20.0,
        }],
    )

    # --- UAV 相机仿真节点 ---
    uav_camera = Node(
        package='uav_sim',
        executable='uav_camera_sim',
        name='uav_camera_sim',
        output='screen',
        parameters=[{
            'image_width': 1920,
            'image_height': 1080,
            'capture_interval': 5.0,
            'jpeg_quality': 80,
        }],
    )

    # --- Gazebo 启动 ---
    gazebo = ExecuteProcess(
        condition=IfCondition(use_gazebo),
        cmd=['gz', 'sim', '-r',
             '$(find uav_sim)/worlds/empty_with_ground.world'],
        output='screen',
    )

    # --- Gazebo-ROS2 桥接 (UAV 位姿) ---
    gz_bridge_pose = Node(
        condition=IfCondition(use_gazebo),
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='uav_gz_pose_bridge',
        arguments=[
            '/model/m300_rtk/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose',
        ],
        output='screen',
    )

    # --- 参数声明 ---
    declare_use_gazebo = DeclareLaunchArgument(
        'use_gazebo', default_value='true',
        description='是否启动 Gazebo 仿真')
    declare_home_lat = DeclareLaunchArgument(
        'home_lat', default_value='30.0',
        description='起飞点纬度')
    declare_home_lon = DeclareLaunchArgument(
        'home_lon', default_value='120.0',
        description='起飞点经度')
    declare_takeoff_height = DeclareLaunchArgument(
        'takeoff_height', default_value='10.0',
        description='起飞高度 (m)')

    return LaunchDescription([
        declare_use_gazebo,
        declare_home_lat,
        declare_home_lon,
        declare_takeoff_height,
        gazebo,
        gz_bridge_pose,
        uav_controller,
        uav_camera,
    ])
