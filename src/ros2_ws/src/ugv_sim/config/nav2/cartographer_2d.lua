# Cartographer 2D SLAM 参数配置 - UGV 建图
# 用于室内/室外 2D 激光 SLAM 建图

options:
  # 基本参数
  map_frame: "map"
  tracking_frame: "ugv_base_link"
  published_frame: "ugv_base_link"
  odom_frame: "ugv_odom"
  provide_odom_frame: true
  use_odometry: true
  use_nav_sat: false
  use_landmarks: false
  num_laser_scans: 1
  num_multi_echo_laser_scans: 0
  num_subdivisions_per_laser_scan: 1
  num_point_clouds: 0
  lookup_transform_timeout_sec: 0.2
  submap_publish_period_sec: 0.3
  pose_publish_period_sec: 5e-3
  publish_to_tf: 1
  publish_tracked_geometry: 0

# 局部 SLAM (前端)
local_slam:
  use_online_correlative_scan_matching: false
  real_time_correlative_scan_matcher:
    linear_search_window: 0.1
    angular_search_window: 0.349  # 20度
    translation_delta_cost_weight: 1e-1
    rotation_delta_cost_weight: 1e-1
  ceres_scan_matcher:
    occupied_space_weight: 1.0
    translation_weight: 10.0
    rotation_weight: 40.0
    ceres_solver_options:
      use_nonmonotonic_steps: true
      max_num_iterations: 10
      num_threads: 1

# 全局 SLAM (后端)
global_slam:
  optimize_every_n_nodes: 90
  use_online_correlative_scan_matching: false
  real_time_correlative_scan_matcher:
    linear_search_window: 0.15
    angular_search_window: 0.524  # 30度
    translation_delta_cost_weight: 1e-1
    rotation_delta_cost_weight: 1e-1
  ceres_scan_matcher:
    occupied_space_weight: 1.0
    translation_weight: 10.0
    rotation_weight: 40.0
    ceres_solver_options:
      use_nonmonotonic_steps: true
      max_num_iterations: 10
      num_threads: 1

# 子图配置
submaps:
  num_range_data: 90
  grid_options_2d:
    grid_type: "PROBABILITY_GRID"
    resolution: 0.05

# 闭环检测
pose_graph:
  optimize_every_n_nodes: 90
  constraint_builder:
    sampling_ratio: 0.3
    max_constraint_distance: 15.0
    min_score: 0.55
    global_localization_min_score: 0.6
    loop_closure_translation_weight: 1.1e4
    loop_closure_rotation_weight: 1e5
    log_matches: true
    ceres_scan_matcher:
      occupied_space_weight: 1.0
      translation_weight: 10.0
      rotation_weight: 40.0
      ceres_solver_options:
        use_nonmonotonic_steps: true
        max_num_iterations: 10
        num_threads: 1
    fast_correlative_scan_matcher:
      linear_search_window: 7.0
      angular_search_window: 0.524
      branch_and_bound_depth: 7
  matcher_translation_weight: 5e2
  matcher_rotation_weight: 1.6e3
  optimization_problem:
    huber_scale: 1e1
    acceleration_weight: 1e1
    rotation_weight: 4e4
    local_slam_pose_translation_weight: 1e5
    local_slam_pose_rotation_weight: 1e5
    odometry_translation_weight: 1.5e5
    odometry_rotation_weight: 6e4
    fixed_frame_pose_translation_weight: 1e1
    fixed_frame_pose_rotation_weight: 1e2
    fixed_frame_pose_use_tolerant_loss: false
    fixed_frame_pose_tolerant_loss_param_a: 1.0
    fixed_frame_pose_tolerant_loss_param_b: 1.0
    log_solver_summary: false
    ceres_solver_options:
      use_nonmonotonic_steps: false
      max_num_iterations: 50
      num_threads: 1

# Lidar 配置 (模拟16线 → 提取地面线后转2D)
trajectory_builder_2d:
  use_imu_data: true
  min_range: 0.3
  max_range: 30.0
  missing_data_ray_length: 5.0
  num_accumulated_range_data: 1
  voxel_filter_size: 0.025
  adaptive_voxel_filter:
    max_length: 0.5
    min_num_points: 200
    max_range: 50.0
  submaps:
    num_range_data: 90
  use_online_correlative_scan_matching: false
  ceres_scan_matcher:
    occupied_space_weight: 1.0
    translation_weight: 10.0
    rotation_weight: 40.0
    ceres_solver_options:
      use_nonmonotonic_steps: true
      max_num_iterations: 10
      num_threads: 1
