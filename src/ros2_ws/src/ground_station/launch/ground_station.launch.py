#!/usr/bin/env python3
"""
地面站 Launch 文件
启动: 数据接收 + Bag 录制 + 缓存管理
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # --- 参数 ---
    bag_dir = LaunchConfiguration('bag_dir', default='~/airunway_bags')
    cache_root = LaunchConfiguration('cache_root', default='~/airunway_cache')

    # --- 数据接收节点 ---
    data_receiver = Node(
        package='ground_station',
        executable='data_receiver',
        name='data_receiver',
        output='screen',
    )

    # --- Bag 录制节点 ---
    bag_recorder = Node(
        package='ground_station',
        executable='bag_recorder',
        name='bag_recorder',
        output='screen',
        parameters=[{
            'bag_dir': bag_dir,
            'max_bag_size': 1073741824,   # 1GB
            'max_bag_duration': 600,       # 10分钟
            'compression_mode': 'zstd',
        }],
    )

    # --- 缓存管理节点 ---
    cache_manager = Node(
        package='ground_station',
        executable='cache_manager',
        name='cache_manager',
        output='screen',
        parameters=[{
            'cache_root': cache_root,
            'max_cache_size_gb': 50.0,
            'retention_days': 7,
            'cleanup_interval_hours': 6,
        }],
    )

    # --- 参数声明 ---
    declare_bag_dir = DeclareLaunchArgument(
        'bag_dir', default_value='~/airunway_bags',
        description='Bag 文件存储目录')
    declare_cache_root = DeclareLaunchArgument(
        'cache_root', default_value='~/airunway_cache',
        description='缓存根目录')

    return LaunchDescription([
        declare_bag_dir,
        declare_cache_root,
        data_receiver,
        bag_recorder,
        cache_manager,
    ])
