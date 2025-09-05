include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  map_frame = "map",
  tracking_frame = "body_link",       -- your base frame
  published_frame = "body_link",
  odom_frame = "odom",

  provide_odom_frame = true,          -- publish map->odom for RViz
  publish_frame_projected_to_2d = true,

  -- turn these ON only if the topics exist (see checks above)
  use_odometry = true,               -- set true if /odom nav_msgs/Odometry is available
  use_nav_sat = false,
  use_landmarks = false,

  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,

  -- Required by cartographer_ros:
  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,

  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.5,
  pose_publish_period_sec = 0.005,
  trajectory_publish_period_sec = 0.03,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- 2D scan-matcher & motion filter tuning for a diff-drive in indoor world
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 12.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 12.0

-- If you have an IMU topic, set this true:
TRAJECTORY_BUILDER_2D.use_imu_data = true

-- Accumulate exactly one 2D scan (good for planar lidar)
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1

-- Use real-time correlative scan matching to help when odom is rough
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.15
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = 0.35

-- Ceres scan-matcher weights (slightly stronger rotation)
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 20

-- Motion filter (don’t drop too many scans)
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.5
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.10
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = 0.10

-- Submap size: smaller = more frequent optimization, crisper loops (higher CPU)
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 60
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05  -- 5 cm

-- Pose graph / loop-closure
POSE_GRAPH = MAP_BUILDER.pose_graph
POSE_GRAPH.optimize_every_n_nodes = 90
POSE_GRAPH.constraint_builder.sampling_ratio = 0.5
POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.global_sampling_ratio = 0.003

return options


