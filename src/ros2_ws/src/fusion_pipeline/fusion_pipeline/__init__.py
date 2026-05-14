"""
空地协同数据融合管道 (Fusion Pipeline)

ROS2 Component 节点管道：
  image_rectify_node   → 图像校正（去畸变）
  pointcloud_filter_node → 点云滤波降采样
  sfm_node             → 增量式 SfM 稀疏点云
  icp_registration_node → ICP 精配准
  meshing_node         → 三角网格生成
"""
