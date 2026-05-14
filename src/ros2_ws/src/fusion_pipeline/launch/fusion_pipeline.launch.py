#!/usr/bin/env python3
"""
空地协同数据融合管道 - ROS2 Launch 文件

启动完整的五阶段数据融合节点管道：
  1. image_rectify_node     → 图像校正
  2. pointcloud_filter_node → 点云滤波降采样
  3. sfm_node               → 增量式 SfM 稀疏点云
  4. icp_registration_node  → ICP 精配准
  5. meshing_node           → 三角网格生成

使用方式:
  ros2 launch fusion_pipeline fusion_pipeline.launch.py
  ros2 launch fusion_pipeline fusion_pipeline.launch.py use_sim_time:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """生成融合管道 Launch 描述"""

    # ---- 参数文件 ----
    config_path = PathJoinSubstitution([
        FindPackageShare('fusion_pipeline'),
        'config',
        'fusion_pipeline_params.yaml'
    ])

    # ---- 启动参数声明 ----
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='是否使用仿真时间'
    )

    # ---- 节点定义 ----
    image_rectify_node = Node(
        package='fusion_pipeline',
        executable='image_rectify_node',
        name='image_rectify_node',
        output='screen',
        parameters=[config_path, {
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        remappings=[
            # 可根据实际 Topic 重映射
        ],
    )

    pointcloud_filter_node = Node(
        package='fusion_pipeline',
        executable='pointcloud_filter_node',
        name='pointcloud_filter_node',
        output='screen',
        parameters=[config_path, {
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    sfm_node = Node(
        package='fusion_pipeline',
        executable='sfm_node',
        name='sfm_node',
        output='screen',
        parameters=[config_path, {
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    icp_registration_node = Node(
        package='fusion_pipeline',
        executable='icp_registration_node',
        name='icp_registration_node',
        output='screen',
        parameters=[config_path, {
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    meshing_node = Node(
        package='fusion_pipeline',
        executable='meshing_node',
        name='meshing_node',
        output='screen',
        parameters=[config_path, {
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    return LaunchDescription([
        use_sim_time_arg,

        # 按管道顺序启动（使用 TimerAction 确保依赖顺序）
        # 第1步: 图像校正 + 点云滤波（并行启动）
        image_rectify_node,
        pointcloud_filter_node,

        # 第2步: SfM（依赖图像校正输出）
        TimerAction(
            period=2.0,
            actions=[sfm_node],
        ),

        # 第3步: ICP 配准（依赖 SfM 点云和滤波后点云）
        TimerAction(
            period=4.0,
            actions=[icp_registration_node],
        ),

        # 第4步: 网格生成（依赖配准后点云）
        TimerAction(
            period=6.0,
            actions=[meshing_node],
        ),
    ])
