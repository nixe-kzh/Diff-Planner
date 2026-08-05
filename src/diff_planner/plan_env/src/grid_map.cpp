#include "plan_env/grid_map.h"

void GridMap::InitMap(ros::NodeHandle &nh)
{
  node_ = nh;

  /* get parameter */
  // double x_size, y_size, z_size;
  node_.param("grid_map/pose_type", mapping_parameters_.pose_type, 1);
  node_.param("grid_map/frame_id", mapping_parameters_.frame_id, string("world"));
  node_.param("grid_map/odom_depth_timeout", mapping_parameters_.odom_depth_timeout, 1.0);

  node_.param("grid_map/resolution", mapping_parameters_.resolution, -1.0);
  node_.param("grid_map/local_update_range_x", mapping_parameters_.local_update_range3d(0), -1.0);
  node_.param("grid_map/local_update_range_y", mapping_parameters_.local_update_range3d(1), -1.0);
  node_.param("grid_map/local_update_range_z", mapping_parameters_.local_update_range3d(2), -1.0);
  node_.param("grid_map/obstacles_inflation", mapping_parameters_.obstacles_inflation, -1.0);
  node_.param("grid_map/enable_virtual_wall", mapping_parameters_.enable_virtual_wall, false);
  node_.param("grid_map/virtual_ceil", mapping_parameters_.virtual_ceil, -1.0);
  node_.param("grid_map/virtual_ground", mapping_parameters_.virtual_ground, -1.0);

  node_.param("grid_map/fx", mapping_parameters_.fx, -1.0);
  node_.param("grid_map/fy", mapping_parameters_.fy, -1.0);
  node_.param("grid_map/cx", mapping_parameters_.cx, -1.0);
  node_.param("grid_map/cy", mapping_parameters_.cy, -1.0);

  node_.param("grid_map/use_depth_filter", mapping_parameters_.use_depth_filter, true);
  node_.param("grid_map/depth_filter_tolerance", mapping_parameters_.depth_filter_tolerance, -1.0);
  node_.param("grid_map/depth_filter_mindist", mapping_parameters_.depth_filter_min_distance, 0.1);
  node_.param("grid_map/depth_filter_margin", mapping_parameters_.depth_filter_margin, -1);
  node_.param("grid_map/k_depth_scaling_factor", mapping_parameters_.depth_scaling_factor, -1.0);
  node_.param("grid_map/skip_pixel", mapping_parameters_.skip_pixel, -1);

  node_.param("grid_map/p_hit", mapping_parameters_.p_hit, 0.70);
  node_.param("grid_map/p_miss", mapping_parameters_.p_miss, 0.35);
  node_.param("grid_map/p_min", mapping_parameters_.p_min, 0.12);
  node_.param("grid_map/p_max", mapping_parameters_.p_max, 0.97);
  node_.param("grid_map/p_occ", mapping_parameters_.p_occ, 0.80);
  node_.param("grid_map/lidar_p_hit", mapping_parameters_.lidar_p_hit, 0.90);
  node_.param("grid_map/lidar_p_miss", mapping_parameters_.lidar_p_miss, 0.49);
  node_.param("grid_map/lidar_p_free", mapping_parameters_.lidar_p_free, 0.499);
  node_.param("grid_map/lidar_p_min", mapping_parameters_.lidar_p_min, 0.12);
  node_.param("grid_map/lidar_p_max", mapping_parameters_.lidar_p_max, 0.98);
  node_.param("grid_map/lidar_p_occ", mapping_parameters_.lidar_p_occ, 0.85);
  node_.param("grid_map/cloud_enable_raycast", mapping_parameters_.cloud_enable_raycast, true);
  node_.param("grid_map/fading_time", mapping_parameters_.fading_time, 1000.0);
  node_.param("grid_map/min_ray_length", mapping_parameters_.min_ray_length, 0.1);

  node_.param("grid_map/show_occ_time", mapping_parameters_.show_occ_time, false);

  node_.param("grid_map/init_x", mapping_parameters_.init_x, 0.0);
  node_.param("grid_map/init_y", mapping_parameters_.init_y, 0.0);
  node_.param("grid_map/init_z", mapping_parameters_.init_z, 0.0);

  mapping_parameters_.inf_grid = ceil((mapping_parameters_.obstacles_inflation - 1e-5) / mapping_parameters_.resolution);
  if (mapping_parameters_.inf_grid > 4)
  {
    mapping_parameters_.inf_grid = 4;
    mapping_parameters_.resolution = mapping_parameters_.obstacles_inflation / mapping_parameters_.inf_grid;
    ROS_WARN("Inflation is too big, which will cause siginificant computation! Resolution enalrged to %f automatically.", mapping_parameters_.resolution);
  }

  mapping_parameters_.resolution_inv = 1 / mapping_parameters_.resolution;
  mapping_parameters_.local_update_range3i = (mapping_parameters_.local_update_range3d * mapping_parameters_.resolution_inv).array().ceil().cast<int>();
  mapping_parameters_.local_update_range3d = mapping_parameters_.local_update_range3i.array().cast<double>() * mapping_parameters_.resolution;
  mapping_data_.ringbuffer_size3i = 2 * mapping_parameters_.local_update_range3i;
  mapping_data_.ringbuffer_inf_size3i = mapping_data_.ringbuffer_size3i + Eigen::Vector3i(2 * mapping_parameters_.inf_grid, 2 * mapping_parameters_.inf_grid, 2 * mapping_parameters_.inf_grid);

  mapping_parameters_.prob_hit_log = PLAN_ENV_LOGIT(mapping_parameters_.p_hit);//0.619
  mapping_parameters_.prob_miss_log = PLAN_ENV_LOGIT(mapping_parameters_.p_miss);//-0.619
  mapping_parameters_.clamp_min_log = PLAN_ENV_LOGIT(mapping_parameters_.p_min);//-1.9924
  mapping_parameters_.clamp_max_log = PLAN_ENV_LOGIT(mapping_parameters_.p_max);//2.1972
  mapping_parameters_.min_occupancy_log = PLAN_ENV_LOGIT(mapping_parameters_.p_occ);//1.3863
  mapping_parameters_.lidar_prob_hit_log = PLAN_ENV_LOGIT(mapping_parameters_.lidar_p_hit);
  mapping_parameters_.lidar_prob_miss_log = PLAN_ENV_LOGIT(mapping_parameters_.lidar_p_miss);
  mapping_parameters_.lidar_clamp_min_log = PLAN_ENV_LOGIT(mapping_parameters_.lidar_p_min);
  mapping_parameters_.lidar_clamp_max_log = PLAN_ENV_LOGIT(mapping_parameters_.lidar_p_max);
  mapping_parameters_.lidar_min_occupancy_log = PLAN_ENV_LOGIT(mapping_parameters_.lidar_p_occ);

  cout << "hit: " << mapping_parameters_.prob_hit_log << endl;
  cout << "miss: " << mapping_parameters_.prob_miss_log << endl;
  cout << "min log: " << mapping_parameters_.clamp_min_log << endl;
  cout << "max: " << mapping_parameters_.clamp_max_log << endl;
  cout << "thresh log: " << mapping_parameters_.min_occupancy_log << endl;
  cout << "lidar hit: " << mapping_parameters_.lidar_prob_hit_log << endl;
  cout << "lidar miss: " << mapping_parameters_.lidar_prob_miss_log << endl;
  cout << "lidar min log: " << mapping_parameters_.lidar_clamp_min_log << endl;
  cout << "lidar max: " << mapping_parameters_.lidar_clamp_max_log << endl;
  cout << "lidar thresh log: " << mapping_parameters_.lidar_min_occupancy_log << endl;

  // initialize data buffers
  Eigen::Vector3i map_voxel_num3i = 2 * mapping_parameters_.local_update_range3i;
  int buffer_size = map_voxel_num3i(0) * map_voxel_num3i(1) * map_voxel_num3i(2);
  int buffer_inf_size = (map_voxel_num3i(0) + 2 * mapping_parameters_.inf_grid) * (map_voxel_num3i(1) + 2 * mapping_parameters_.inf_grid) * (map_voxel_num3i(2) + 2 * mapping_parameters_.inf_grid);
  mapping_data_.ringbuffer_origin3i = Eigen::Vector3i(0, 0, 0);
  mapping_data_.ringbuffer_inf_origin3i = Eigen::Vector3i(0, 0, 0);

  mapping_data_.occupancy_buffer = vector<double>(buffer_size, mapping_parameters_.clamp_min_log);
  mapping_data_.occupancy_buffer_inflate = vector<uint16_t>(buffer_inf_size, 0);

  mapping_data_.count_hit_and_miss = vector<short>(buffer_size, 0);
  mapping_data_.count_hit = vector<short>(buffer_size, 0);
  mapping_data_.flag_rayend = vector<char>(buffer_size, -1);
  mapping_data_.flag_traverse = vector<char>(buffer_size, -1);
  mapping_data_.cache_voxel = vector<Eigen::Vector3i>(buffer_size, Eigen::Vector3i(0, 0, 0));

  mapping_data_.raycast_num = 0;
  mapping_data_.projected_point_count = 0;
  mapping_data_.cache_voxel_count = 0;

  mapping_data_.camera_to_body << 0.0, 0.0, 1.0, 0.0,
      -1.0, 0.0, 0.0, 0.0,
      0.0, -1.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 1.0;

  /* init callback */
  depth_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(node_, "grid_map/depth", 50));
  extrinsic_sub_ = node_.subscribe<nav_msgs::Odometry>(
      "/vins_estimator/extrinsic", 10, &GridMap::ExtrinsicCallback, this); //sub

  if (mapping_parameters_.pose_type == kPoseStamped)
  {
    pose_sub_.reset(
        new message_filters::Subscriber<geometry_msgs::PoseStamped>(node_, "grid_map/pose", 25));

    sync_image_pose_.reset(new message_filters::Synchronizer<SyncPolicyImagePose>(
        SyncPolicyImagePose(100), *depth_sub_, *pose_sub_));
    sync_image_pose_->registerCallback(boost::bind(&GridMap::DepthPoseCallback, this, _1, _2));
  }
  else if (mapping_parameters_.pose_type == kOdometry)
  {
    odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(node_, "grid_map/odom", 100, ros::TransportHints().tcpNoDelay()));

    sync_image_odom_.reset(new message_filters::Synchronizer<SyncPolicyImageOdom>(
        SyncPolicyImageOdom(100), *depth_sub_, *odom_sub_));
    sync_image_odom_->registerCallback(boost::bind(&GridMap::DepthOdometryCallback, this, _1, _2));
  }

  // use odometry and point cloud
  indep_odom_sub_ =
      node_.subscribe<nav_msgs::Odometry>("grid_map/odom", 10, &GridMap::OdometryCallback, this);
  indep_cloud_sub_ =
      node_.subscribe<sensor_msgs::PointCloud2>("grid_map/cloud", 10, &GridMap::PointCloudCallback, this);

  occ_timer_ = node_.createTimer(ros::Duration(0.032), &GridMap::UpdateOccupancyCallback, this);
  vis_timer_ = node_.createTimer(ros::Duration(0.125), &GridMap::VisualizationCallback, this);
  if (mapping_parameters_.fading_time > 0)
    fading_timer_ = node_.createTimer(ros::Duration(0.5), &GridMap::FadingCallback, this);

  map_pub_ = node_.advertise<sensor_msgs::PointCloud2>("grid_map/occupancy", 10);
  map_inf_pub_ = node_.advertise<sensor_msgs::PointCloud2>("grid_map/occupancy_inflate", 10);

  mapping_data_.occ_need_update = false;
  mapping_data_.has_first_depth = false;
  mapping_data_.has_odom = false;
  mapping_data_.last_occ_update_time.fromSec(0);

  mapping_data_.flag_have_ever_received_depth = false;
  mapping_data_.flag_depth_odom_timeout = false;
  mapping_data_.use_lidar_prob_for_update = false;
}

void GridMap::UpdateOccupancyCallback(const ros::TimerEvent & /*event*/)
{
  if (!NeedsDepthOdometryUpdate())
    return;

  /* update occupancy */
  ros::Time t1, t2, t3, t4, t5;
  t1 = ros::Time::now();

  MoveRingBuffer();
  t2 = ros::Time::now();

  ProjectDepthImage();
  t3 = ros::Time::now();

  if (mapping_data_.projected_point_count > 0)
  {
    ProcessRaycast();
    t4 = ros::Time::now();

    ClearAndInflateLocalMap();
    t5 = ros::Time::now();

    if (mapping_parameters_.show_occ_time)
    {
      cout << setprecision(7);
      cout << "t2=" << (t2 - t1).toSec() << " t3=" << (t3 - t2).toSec() << " t4=" << (t4 - t3).toSec() << " t5=" << (t5 - t4).toSec() << endl;

      static int update_count = 0;
      static double raycast_time = 0;
      static double max_raycast_time = 0;
      static double inflation_time = 0;
      static double max_inflation_time = 0;
      raycast_time += (t4 - t3).toSec();
      max_raycast_time = max(max_raycast_time, (t4 - t3).toSec());
      inflation_time += (t5 - t4).toSec();
      max_inflation_time = max(max_inflation_time, (t5 - t4).toSec());
      ++update_count;

      printf("Raycast(ms): cur t = %lf, avg t = %lf, max t = %lf\n", (t4 - t3).toSec() * 1000, raycast_time / update_count * 1000, max_raycast_time * 1000);
      printf("Infaltion(ms): cur t = %lf, avg t = %lf, max t = %lf\n", (t5 - t4).toSec() * 1000, inflation_time / update_count * 1000, max_inflation_time * 1000);
    }
  }

  mapping_data_.occ_need_update = false;
}

void GridMap::VisualizationCallback(const ros::TimerEvent & /*event*/)
{
  if (!mapping_parameters_.have_initialized)
    return;

  ros::Time t0 = ros::Time::now();
  PublishInflatedMap();
  PublishMap();
  ros::Time t1 = ros::Time::now();

  if (mapping_parameters_.show_occ_time)
  {
    printf("Visualization(ms):%f\n", (t1 - t0).toSec() * 1000);
  }
}

void GridMap::FadingCallback(const ros::TimerEvent & /*event*/)
{
  const double reduce = (mapping_parameters_.clamp_max_log - mapping_parameters_.min_occupancy_log) / (mapping_parameters_.fading_time * 2); // function called at 2Hz
  const double low_threshold = mapping_parameters_.clamp_min_log + reduce;

  ros::Time t0 = ros::Time::now();
  for (size_t i = 0; i < mapping_data_.occupancy_buffer.size(); ++i)
  {
    if (mapping_data_.occupancy_buffer[i] > low_threshold)
    {
      bool obs_flag = mapping_data_.occupancy_buffer[i] >= mapping_parameters_.min_occupancy_log;
      mapping_data_.occupancy_buffer[i] -= reduce;
      if (obs_flag && mapping_data_.occupancy_buffer[i] < mapping_parameters_.min_occupancy_log)
      {
        Eigen::Vector3i idx = BufferIndexToGlobalIndex(i);
        int inf_buf_idx = GlobalIndexToInflatedBufferIndex(idx);
         if (mapping_data_.occupancy_buffer_inflate[inf_buf_idx] > PLAN_ENV_GRID_MAP_OBSTACLE_FLAG)
         {
          ChangeInflatedBuffer(false, inf_buf_idx, idx);
         }
      }
    }
  }
  ros::Time t1 = ros::Time::now();

  if (mapping_parameters_.show_occ_time)
  {
    printf("Fading(ms):%f\n", (t1 - t0).toSec() * 1000); 
  }
}

void GridMap::DepthPoseCallback(const sensor_msgs::ImageConstPtr &img,
                                const geometry_msgs::PoseStampedConstPtr &pose)
{
  /* get depth image */
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, img->encoding);

  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, mapping_parameters_.depth_scaling_factor);
  }
  cv_ptr->image.copyTo(mapping_data_.depth_image);

  static bool first_flag = true;
  if (first_flag)
  {
    first_flag = false;
    mapping_data_.proj_points.resize(mapping_data_.depth_image.cols * mapping_data_.depth_image.rows / mapping_parameters_.skip_pixel / mapping_parameters_.skip_pixel);
  }

  // std::cout << "depth: " << mapping_data_.depth_image.cols << ", " << mapping_data_.depth_image.rows << std::endl;

  /* get pose */
  mapping_data_.camera_pos(0) = pose->pose.position.x;
  mapping_data_.camera_pos(1) = pose->pose.position.y;
  mapping_data_.camera_pos(2) = pose->pose.position.z;
  mapping_data_.camera_r_m = Eigen::Quaterniond(pose->pose.orientation.w, pose->pose.orientation.x,
                                       pose->pose.orientation.y, pose->pose.orientation.z)
                        .toRotationMatrix();

  mapping_data_.occ_need_update = true;
  mapping_data_.flag_have_ever_received_depth = true;
  mapping_data_.use_lidar_prob_for_update = false;
}

void GridMap::DepthOdometryCallback(const sensor_msgs::ImageConstPtr &img,
                                const nav_msgs::OdometryConstPtr &odom)
{

  /* get pose */
  Eigen::Quaterniond body_q = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                                 odom->pose.pose.orientation.x,
                                                 odom->pose.pose.orientation.y,
                                                 odom->pose.pose.orientation.z);
  Eigen::Matrix3d body_r_m = body_q.toRotationMatrix();
  Eigen::Matrix4d body_to_world;
  body_to_world.block<3, 3>(0, 0) = body_r_m;
  body_to_world(0, 3) = odom->pose.pose.position.x;
  body_to_world(1, 3) = odom->pose.pose.position.y;
  body_to_world(2, 3) = odom->pose.pose.position.z;
  body_to_world(3, 3) = 1.0;

  Eigen::Matrix4d camera_transform = body_to_world * mapping_data_.camera_to_body;
  mapping_data_.camera_pos(0) = camera_transform(0, 3);
  mapping_data_.camera_pos(1) = camera_transform(1, 3);
  mapping_data_.camera_pos(2) = camera_transform(2, 3);
  mapping_data_.camera_r_m = camera_transform.block<3, 3>(0, 0);

  /* get depth image */
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, mapping_parameters_.depth_scaling_factor);
  }
  cv_ptr->image.copyTo(mapping_data_.depth_image);

  static bool first_flag = true;
  if (first_flag)
  {
    first_flag = false;
    mapping_data_.proj_points.resize(mapping_data_.depth_image.cols * mapping_data_.depth_image.rows / mapping_parameters_.skip_pixel / mapping_parameters_.skip_pixel);
  }

  mapping_data_.occ_need_update = true;
  mapping_data_.flag_have_ever_received_depth = true;
  mapping_data_.use_lidar_prob_for_update = false;
}

void GridMap::OdometryCallback(const nav_msgs::OdometryConstPtr &odom)
{
  if (mapping_data_.flag_have_ever_received_depth)
  {
    indep_odom_sub_.shutdown();
    return;
  }

  /* get pose */
  Eigen::Quaterniond body_q = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                                 odom->pose.pose.orientation.x,
                                                 odom->pose.pose.orientation.y,
                                                 odom->pose.pose.orientation.z);
  Eigen::Matrix3d body_r_m = body_q.toRotationMatrix();
  Eigen::Matrix4d body_to_world;
  body_to_world.block<3, 3>(0, 0) = body_r_m;
  body_to_world(0, 3) = odom->pose.pose.position.x;
  body_to_world(1, 3) = odom->pose.pose.position.y;
  body_to_world(2, 3) = odom->pose.pose.position.z;
  body_to_world(3, 3) = 1.0;

  Eigen::Matrix4d camera_transform = body_to_world * mapping_data_.camera_to_body;
  mapping_data_.camera_pos(0) = camera_transform(0, 3);
  mapping_data_.camera_pos(1) = camera_transform(1, 3);
  mapping_data_.camera_pos(2) = camera_transform(2, 3);
  mapping_data_.camera_r_m = camera_transform.block<3, 3>(0, 0);

  mapping_data_.has_odom = true;
}

void GridMap::PointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  if (!mapping_data_.has_odom)
  {
    std::cout << "grid_map: no odom!" << std::endl;
    return;
  }

  pcl::PointCloud<pcl::PointXYZ> latest_cloud;
  pcl::fromROSMsg(*msg, latest_cloud);
  if (latest_cloud.empty())
    return;
  if (isnan(mapping_data_.camera_pos(0)) || isnan(mapping_data_.camera_pos(1)) || isnan(mapping_data_.camera_pos(2)))
    return;
  mapping_data_.projected_point_count = 0;
  if (mapping_data_.proj_points.size() < latest_cloud.size())
    mapping_data_.proj_points.resize(latest_cloud.size());

  for (const auto &pt : latest_cloud.points)
  {
    if (!std::isfinite(pt.x + pt.y + pt.z))
      continue;
    mapping_data_.proj_points[mapping_data_.projected_point_count++] = Eigen::Vector3d(pt.x + mapping_parameters_.init_x, pt.y + mapping_parameters_.init_y, pt.z + mapping_parameters_.init_z);
  }
  MoveRingBuffer();
  mapping_data_.use_lidar_prob_for_update = true;
  RaycastFromPointCloud();// TODO by glq
  ClearAndInflateLocalMap();
  mapping_data_.occ_need_update = true;
}

void GridMap::ExtrinsicCallback(const nav_msgs::OdometryConstPtr &odom)
{
  Eigen::Quaterniond camera_to_body_quaternion = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                                     odom->pose.pose.orientation.x,
                                                     odom->pose.pose.orientation.y,
                                                     odom->pose.pose.orientation.z);
  Eigen::Matrix3d camera_to_body_rotation = camera_to_body_quaternion.toRotationMatrix();
  mapping_data_.camera_to_body.block<3, 3>(0, 0) = camera_to_body_rotation;
  mapping_data_.camera_to_body(0, 3) = odom->pose.pose.position.x;
  mapping_data_.camera_to_body(1, 3) = odom->pose.pose.position.y;
  mapping_data_.camera_to_body(2, 3) = odom->pose.pose.position.z;
  mapping_data_.camera_to_body(3, 3) = 1.0;
}

void GridMap::MoveRingBuffer()
{
  if (!mapping_parameters_.have_initialized)
    InitializeMapBoundary();

  Eigen::Vector3i center_new = PositionToGlobalIndex(mapping_data_.camera_pos);
  Eigen::Vector3i ringbuffer_lowbound3i_new = center_new - mapping_parameters_.local_update_range3i;
  Eigen::Vector3d ringbuffer_lowbound3d_new = ringbuffer_lowbound3i_new.cast<double>() * mapping_parameters_.resolution;
  Eigen::Vector3i ringbuffer_upbound3i_new = center_new + mapping_parameters_.local_update_range3i;
  Eigen::Vector3d ringbuffer_upbound3d_new = ringbuffer_upbound3i_new.cast<double>() * mapping_parameters_.resolution;
  ringbuffer_upbound3i_new -= Eigen::Vector3i(1, 1, 1);

  const Eigen::Vector3i inf_grid3i(mapping_parameters_.inf_grid, mapping_parameters_.inf_grid, mapping_parameters_.inf_grid);
  const Eigen::Vector3d inf_grid3d = inf_grid3i.array().cast<double>() * mapping_parameters_.resolution;
  Eigen::Vector3i ringbuffer_inf_lowbound3i_new = ringbuffer_lowbound3i_new - inf_grid3i;
  Eigen::Vector3d ringbuffer_inf_lowbound3d_new = ringbuffer_lowbound3d_new - inf_grid3d;
  Eigen::Vector3i ringbuffer_inf_upbound3i_new = ringbuffer_upbound3i_new + inf_grid3i;
  Eigen::Vector3d ringbuffer_inf_upbound3d_new = ringbuffer_upbound3d_new + inf_grid3d;

  if (center_new(0) < mapping_data_.center_last3i(0))
    ClearBuffer(0, ringbuffer_upbound3i_new(0));
  if (center_new(0) > mapping_data_.center_last3i(0))
    ClearBuffer(1, ringbuffer_lowbound3i_new(0));
  if (center_new(1) < mapping_data_.center_last3i(1))
    ClearBuffer(2, ringbuffer_upbound3i_new(1));
  if (center_new(1) > mapping_data_.center_last3i(1))
    ClearBuffer(3, ringbuffer_lowbound3i_new(1));
  if (center_new(2) < mapping_data_.center_last3i(2))
    ClearBuffer(4, ringbuffer_upbound3i_new(2));
  if (center_new(2) > mapping_data_.center_last3i(2))
    ClearBuffer(5, ringbuffer_lowbound3i_new(2));

  for (int i = 0; i < 3; ++i)
  {
    while (mapping_data_.ringbuffer_origin3i(i) < mapping_data_.ringbuffer_lowbound3i(i))
    {
      mapping_data_.ringbuffer_origin3i(i) += mapping_data_.ringbuffer_size3i(i);
    }
    while (mapping_data_.ringbuffer_origin3i(i) > mapping_data_.ringbuffer_upbound3i(i))
    {
      mapping_data_.ringbuffer_origin3i(i) -= mapping_data_.ringbuffer_size3i(i);
    }

    while (mapping_data_.ringbuffer_inf_origin3i(i) < mapping_data_.ringbuffer_inf_lowbound3i(i))
    {
      mapping_data_.ringbuffer_inf_origin3i(i) += mapping_data_.ringbuffer_inf_size3i(i);
    }
    while (mapping_data_.ringbuffer_inf_origin3i(i) > mapping_data_.ringbuffer_inf_upbound3i(i))
    {
      mapping_data_.ringbuffer_inf_origin3i(i) -= mapping_data_.ringbuffer_inf_size3i(i);
    }
  }

  mapping_data_.center_last3i = center_new;
  mapping_data_.ringbuffer_lowbound3i = ringbuffer_lowbound3i_new;
  mapping_data_.ringbuffer_lowbound3d = ringbuffer_lowbound3d_new;
  mapping_data_.ringbuffer_upbound3i = ringbuffer_upbound3i_new;
  mapping_data_.ringbuffer_upbound3d = ringbuffer_upbound3d_new;
  mapping_data_.ringbuffer_inf_lowbound3i = ringbuffer_inf_lowbound3i_new;
  mapping_data_.ringbuffer_inf_lowbound3d = ringbuffer_inf_lowbound3d_new;
  mapping_data_.ringbuffer_inf_upbound3i = ringbuffer_inf_upbound3i_new;
  mapping_data_.ringbuffer_inf_upbound3d = ringbuffer_inf_upbound3d_new;
}

void GridMap::ProjectDepthImage()
{
  mapping_data_.projected_point_count = 0;

  uint16_t *row_ptr;
  int cols = mapping_data_.depth_image.cols;
  int rows = mapping_data_.depth_image.rows;
  int skip_pix = mapping_parameters_.skip_pixel;

  double depth;

  Eigen::Matrix3d camera_r = mapping_data_.camera_r_m;

  if (!mapping_parameters_.use_depth_filter)
  {
    for (int v = 0; v < rows; v += skip_pix)
    {
      row_ptr = mapping_data_.depth_image.ptr<uint16_t>(v);

      for (int u = 0; u < cols; u += skip_pix)
      {

        Eigen::Vector3d proj_pt;
        depth = (*row_ptr) / mapping_parameters_.depth_scaling_factor;
        row_ptr = row_ptr + mapping_parameters_.skip_pixel;

        if (depth < 0.1)
          continue;

        proj_pt(0) = (u - mapping_parameters_.cx) * depth / mapping_parameters_.fx;
        proj_pt(1) = (v - mapping_parameters_.cy) * depth / mapping_parameters_.fy;
        proj_pt(2) = depth;

        proj_pt = camera_r * proj_pt + mapping_data_.camera_pos;

        mapping_data_.proj_points[mapping_data_.projected_point_count++] = proj_pt;
      }
    }
  }
  /* use depth filter */
  else
  {

    if (!mapping_data_.has_first_depth)
      mapping_data_.has_first_depth = true;
    else
    {
      Eigen::Vector3d pt_cur, pt_world, pt_reproj;

      Eigen::Matrix3d last_camera_r_inv;
      last_camera_r_inv = mapping_data_.last_camera_r_m.inverse();
      const double inv_factor = 1.0 / mapping_parameters_.depth_scaling_factor;

      for (int v = mapping_parameters_.depth_filter_margin; v < rows - mapping_parameters_.depth_filter_margin; v += mapping_parameters_.skip_pixel)
      {
        row_ptr = mapping_data_.depth_image.ptr<uint16_t>(v) + mapping_parameters_.depth_filter_margin;

        for (int u = mapping_parameters_.depth_filter_margin; u < cols - mapping_parameters_.depth_filter_margin;
             u += mapping_parameters_.skip_pixel)
        {

          depth = (*row_ptr) * inv_factor;
          row_ptr = row_ptr + mapping_parameters_.skip_pixel;

          // filter depth
          // depth += rand_noise_(eng_);
          // if (depth > 0.01) depth += rand_noise2_(eng_);

          if (depth < mapping_parameters_.depth_filter_min_distance)
          {
            continue;
          }

          // project to world frame
          pt_cur(0) = (u - mapping_parameters_.cx) * depth / mapping_parameters_.fx;
          pt_cur(1) = (v - mapping_parameters_.cy) * depth / mapping_parameters_.fy;
          pt_cur(2) = depth;

          pt_world = camera_r * pt_cur + mapping_data_.camera_pos;

          mapping_data_.proj_points[mapping_data_.projected_point_count++] = pt_world;

          // check consistency with last image, disabled...
          if (false)
          {
            pt_reproj = last_camera_r_inv * (pt_world - mapping_data_.last_camera_pos);
            double uu = pt_reproj.x() * mapping_parameters_.fx / pt_reproj.z() + mapping_parameters_.cx;
            double vv = pt_reproj.y() * mapping_parameters_.fy / pt_reproj.z() + mapping_parameters_.cy;

            if (uu >= 0 && uu < cols && vv >= 0 && vv < rows)
            {
              if (fabs(mapping_data_.last_depth_image.at<uint16_t>((int)vv, (int)uu) * inv_factor -
                       pt_reproj.z()) < mapping_parameters_.depth_filter_tolerance)
              {
                mapping_data_.proj_points[mapping_data_.projected_point_count++] = pt_world;
              }
            }
            else
            {
              mapping_data_.proj_points[mapping_data_.projected_point_count++] = pt_world;
            }
          }
        }
      }
    }
  }

  /* maintain camera pose for consistency check */

  mapping_data_.last_camera_pos = mapping_data_.camera_pos;
  mapping_data_.last_camera_r_m = mapping_data_.camera_r_m;
  mapping_data_.last_depth_image = mapping_data_.depth_image;
}

void GridMap::RaycastFromPointCloud()
{
  if (mapping_data_.projected_point_count == 0)
    return;
  mapping_data_.cache_voxel_count = 0;
  // ros::Time t1, t2, t3;
  ros::WallTime t1, t2, t3;
  mapping_data_.raycast_num += 1;
  RayCaster raycaster;
  Eigen::Vector3d ray_pt, pt_w;
  int pts_num = 0;
  t1 = ros::WallTime::now();
  for (int i = 0; i < mapping_data_.projected_point_count; ++i)
  {
    pt_w = mapping_data_.proj_points[i];

    int vox_idx = kInvalidIndex;
    if (!IsInBuffer(pt_w))
    {
      if (mapping_parameters_.cloud_enable_raycast)
      {
        pt_w = ClosestPointInMap(pt_w, mapping_data_.camera_pos);
        vox_idx = SetCachedOccupancy(pt_w, 0);
        pts_num++;
      }
    }
    else
    {
      vox_idx = SetCachedOccupancy(pt_w, 1);
      pts_num++;
    }

    if (!mapping_parameters_.cloud_enable_raycast)
      continue;

    if (vox_idx != kInvalidIndex)
    {
      if (mapping_data_.flag_rayend[vox_idx] == mapping_data_.raycast_num)
        continue;
      mapping_data_.flag_rayend[vox_idx] = mapping_data_.raycast_num;
    }
    raycaster.SetInput(pt_w / mapping_parameters_.resolution, mapping_data_.camera_pos / mapping_parameters_.resolution);
    while (raycaster.Step(ray_pt))
    {
      Eigen::Vector3d tmp = (ray_pt + Eigen::Vector3d(0.5, 0.5, 0.5)) * mapping_parameters_.resolution;
      pts_num++;
      int idx = SetCachedOccupancy(tmp, 0);
      if (idx != kInvalidIndex)
      {
        if (mapping_data_.flag_traverse[idx] == mapping_data_.raycast_num)
          break;
        mapping_data_.flag_traverse[idx] = mapping_data_.raycast_num;
      }
    }
  }
  t2 = ros::WallTime::now();
  for (int i = 0; i < mapping_data_.cache_voxel_count; ++i)
  {
    int buf_id = GlobalIndexToBufferIndex(mapping_data_.cache_voxel[i]);
    double log_update =
        (mapping_data_.count_hit[buf_id] > 0)
            ? mapping_parameters_.lidar_prob_hit_log
            : mapping_parameters_.lidar_prob_miss_log;

    mapping_data_.count_hit[buf_id] = mapping_data_.count_hit_and_miss[buf_id] = 0;

    if (log_update >= 0 && mapping_data_.occupancy_buffer[buf_id] >= mapping_parameters_.lidar_clamp_max_log)
      continue;
    if (log_update <= 0 && mapping_data_.occupancy_buffer[buf_id] <= mapping_parameters_.lidar_clamp_min_log)
      continue;

    mapping_data_.occupancy_buffer[buf_id] =
        std::min(std::max(mapping_data_.occupancy_buffer[buf_id] + log_update, mapping_parameters_.lidar_clamp_min_log),
                 mapping_parameters_.lidar_clamp_max_log);
  }
  t3 = ros::WallTime::now();
    if (mapping_parameters_.show_occ_time)
  {
    ROS_WARN("Raycast time: t2-t1=%f, t3-t2=%f, pts_num=%d", (t2 - t1).toSec(), (t3 - t2).toSec(), pts_num);
  }
}

void GridMap::ProcessRaycast()
{
  mapping_data_.cache_voxel_count = 0;

  ros::Time t1, t2, t3;

  mapping_data_.raycast_num += 1;

  RayCaster raycaster;
  Eigen::Vector3d ray_pt, pt_w;

  int pts_num = 0;
  t1 = ros::Time::now();
  for (int i = 0; i < mapping_data_.projected_point_count; ++i)
  {
    int vox_idx;
    pt_w = mapping_data_.proj_points[i];

    // set flag for projected point

    if (!IsInBuffer(pt_w))
    {
      pt_w = ClosestPointInMap(pt_w, mapping_data_.camera_pos);
      pts_num++;
      vox_idx = SetCachedOccupancy(pt_w, 0);
    }
    else
    {
      pts_num++;
      vox_idx = SetCachedOccupancy(pt_w, 1);
    }

    // raycasting between camera center and point

    if (vox_idx != kInvalidIndex)
    {
      if (mapping_data_.flag_rayend[vox_idx] == mapping_data_.raycast_num)
      {
        continue;
      }
      else
      {
        mapping_data_.flag_rayend[vox_idx] = mapping_data_.raycast_num;
      }
    }

    raycaster.SetInput(pt_w / mapping_parameters_.resolution, mapping_data_.camera_pos / mapping_parameters_.resolution);

    while (raycaster.Step(ray_pt))
    {
      Eigen::Vector3d tmp = (ray_pt + Eigen::Vector3d(0.5, 0.5, 0.5)) * mapping_parameters_.resolution;

      pts_num++;
      vox_idx = SetCachedOccupancy(tmp, 0);

      if (vox_idx != kInvalidIndex)
      {
        if (mapping_data_.flag_traverse[vox_idx] == mapping_data_.raycast_num)
        {
          break;
        }
        else
        {
          mapping_data_.flag_traverse[vox_idx] = mapping_data_.raycast_num;
        }
      }
    }
  }

  t2 = ros::Time::now();

  for (int i = 0; i < mapping_data_.cache_voxel_count; ++i)
  {

    int idx_ctns = GlobalIndexToBufferIndex(mapping_data_.cache_voxel[i]);

    double log_odds_update =
        mapping_data_.count_hit[idx_ctns] >= mapping_data_.count_hit_and_miss[idx_ctns] - mapping_data_.count_hit[idx_ctns] ? mapping_parameters_.prob_hit_log : mapping_parameters_.prob_miss_log;

    mapping_data_.count_hit[idx_ctns] = mapping_data_.count_hit_and_miss[idx_ctns] = 0;

    if (log_odds_update >= 0 && mapping_data_.occupancy_buffer[idx_ctns] >= mapping_parameters_.clamp_max_log)
    {
      continue;
    }
    else if (log_odds_update <= 0 && mapping_data_.occupancy_buffer[idx_ctns] <= mapping_parameters_.clamp_min_log)
    {
      continue;
    }

    mapping_data_.occupancy_buffer[idx_ctns] =
        std::min(std::max(mapping_data_.occupancy_buffer[idx_ctns] + log_odds_update, mapping_parameters_.clamp_min_log),
                 mapping_parameters_.clamp_max_log);
  }

  t3 = ros::Time::now();

  if (mapping_parameters_.show_occ_time)
  {
    ROS_WARN("Raycast time: t2-t1=%f, t3-t2=%f, pts_num=%d", (t2 - t1).toSec(), (t3 - t2).toSec(), pts_num);
  }
}

void GridMap::ClearAndInflateLocalMap()
{
  const double occupancy_threshold =
      mapping_data_.use_lidar_prob_for_update ? mapping_parameters_.lidar_min_occupancy_log : mapping_parameters_.min_occupancy_log;
  for (int i = 0; i < mapping_data_.cache_voxel_count; ++i)
  {
    Eigen::Vector3i idx = mapping_data_.cache_voxel[i];
    int buf_id = GlobalIndexToBufferIndex(idx);
    int inf_buf_id = GlobalIndexToInflatedBufferIndex(idx);

    if (mapping_data_.occupancy_buffer_inflate[inf_buf_id] < PLAN_ENV_GRID_MAP_OBSTACLE_FLAG && mapping_data_.occupancy_buffer[buf_id] >= occupancy_threshold)
    {
      ChangeInflatedBuffer(true, inf_buf_id, idx);
    }

    if (mapping_data_.occupancy_buffer_inflate[inf_buf_id] >= PLAN_ENV_GRID_MAP_OBSTACLE_FLAG && mapping_data_.occupancy_buffer[buf_id] < occupancy_threshold)
    {
      ChangeInflatedBuffer(false, inf_buf_id, idx);
    }
  }
}

void GridMap::InitializeMapBoundary()
{
  mapping_parameters_.have_initialized = true;

  mapping_data_.center_last3i = PositionToGlobalIndex(mapping_data_.camera_pos);

  mapping_data_.ringbuffer_lowbound3i = mapping_data_.center_last3i - mapping_parameters_.local_update_range3i;
  mapping_data_.ringbuffer_lowbound3d = mapping_data_.ringbuffer_lowbound3i.cast<double>() * mapping_parameters_.resolution;
  mapping_data_.ringbuffer_upbound3i = mapping_data_.center_last3i + mapping_parameters_.local_update_range3i;
  mapping_data_.ringbuffer_upbound3d = mapping_data_.ringbuffer_upbound3i.cast<double>() * mapping_parameters_.resolution;
  mapping_data_.ringbuffer_upbound3i -= Eigen::Vector3i(1, 1, 1);

  const Eigen::Vector3i inf_grid3i(mapping_parameters_.inf_grid, mapping_parameters_.inf_grid, mapping_parameters_.inf_grid);
  const Eigen::Vector3d inf_grid3d = inf_grid3i.array().cast<double>() * mapping_parameters_.resolution;
  mapping_data_.ringbuffer_inf_lowbound3i = mapping_data_.ringbuffer_lowbound3i - inf_grid3i;
  mapping_data_.ringbuffer_inf_lowbound3d = mapping_data_.ringbuffer_lowbound3d - inf_grid3d;
  mapping_data_.ringbuffer_inf_upbound3i = mapping_data_.ringbuffer_upbound3i + inf_grid3i;
  mapping_data_.ringbuffer_inf_upbound3d = mapping_data_.ringbuffer_upbound3d + inf_grid3d;

  // cout << "md_.ringbuffer_lowbound3i_=" << mapping_data_.ringbuffer_lowbound3i.transpose() << " md_.ringbuffer_lowbound3d_=" << mapping_data_.ringbuffer_lowbound3d.transpose() << " md_.ringbuffer_upbound3i_=" << mapping_data_.ringbuffer_upbound3i.transpose() << " md_.ringbuffer_upbound3d_=" << mapping_data_.ringbuffer_upbound3d.transpose() << endl;

  for (int i = 0; i < 3; ++i)
  {
    while (mapping_data_.ringbuffer_origin3i(i) < mapping_data_.ringbuffer_lowbound3i(i))
    {
      mapping_data_.ringbuffer_origin3i(i) += mapping_data_.ringbuffer_size3i(i);
    }
    while (mapping_data_.ringbuffer_origin3i(i) > mapping_data_.ringbuffer_upbound3i(i))
    {
      mapping_data_.ringbuffer_origin3i(i) -= mapping_data_.ringbuffer_size3i(i);
    }

    while (mapping_data_.ringbuffer_inf_origin3i(i) < mapping_data_.ringbuffer_inf_lowbound3i(i))
    {
      mapping_data_.ringbuffer_inf_origin3i(i) += mapping_data_.ringbuffer_inf_size3i(i);
    }
    while (mapping_data_.ringbuffer_inf_origin3i(i) > mapping_data_.ringbuffer_inf_upbound3i(i))
    {
      mapping_data_.ringbuffer_inf_origin3i(i) -= mapping_data_.ringbuffer_inf_size3i(i);
    }
  }

#if PLAN_ENV_GRID_MAP_NEW_PLATFORM_TEST
  TestIndexingCost();
#endif
}

void GridMap::ClearBuffer(char case_index, int bound)
{
  for (int x = (case_index == 0 ? bound : mapping_data_.ringbuffer_lowbound3i(0)); x <= (case_index == 1 ? bound : mapping_data_.ringbuffer_upbound3i(0)); ++x)
    for (int y = (case_index == 2 ? bound : mapping_data_.ringbuffer_lowbound3i(1)); y <= (case_index == 3 ? bound : mapping_data_.ringbuffer_upbound3i(1)); ++y)
      for (int z = (case_index == 4 ? bound : mapping_data_.ringbuffer_lowbound3i(2)); z <= (case_index == 5 ? bound : mapping_data_.ringbuffer_upbound3i(2)); ++z)
      {
        Eigen::Vector3i id_global(x, y, z);
        int id_buf = GlobalIndexToBufferIndex(id_global);
        int id_buf_inf = GlobalIndexToInflatedBufferIndex(id_global);
        Eigen::Vector3i id_global_inf_clr((case_index == 0 ? x + mapping_parameters_.inf_grid : (case_index == 1 ? x - mapping_parameters_.inf_grid : x)),
                                          (case_index == 2 ? y + mapping_parameters_.inf_grid : (case_index == 3 ? y - mapping_parameters_.inf_grid : y)),
                                          (case_index == 4 ? z + mapping_parameters_.inf_grid : (case_index == 5 ? z - mapping_parameters_.inf_grid : z)));
        // int id_buf_inf_clr = GlobalIndexToInflatedBufferIndex(id_global_inf_clr);

        // mapping_data_.occupancy_buffer_inflate[id_buf_inf_clr] = 0;
        mapping_data_.count_hit[id_buf] = 0;
        mapping_data_.count_hit_and_miss[id_buf] = 0;
        mapping_data_.flag_traverse[id_buf] = mapping_data_.raycast_num;
        mapping_data_.flag_rayend[id_buf] = mapping_data_.raycast_num;
        mapping_data_.occupancy_buffer[id_buf] = mapping_parameters_.clamp_min_log;

        if (mapping_data_.occupancy_buffer_inflate[id_buf_inf] > PLAN_ENV_GRID_MAP_OBSTACLE_FLAG)
        {
          ChangeInflatedBuffer(false, id_buf_inf, id_global);
        }
      }

#if PLAN_ENV_GRID_MAP_NEW_PLATFORM_TEST
  for (int x = (case_index == 0 ? bound : mapping_data_.ringbuffer_lowbound3i(0)); x <= (case_index == 1 ? bound : mapping_data_.ringbuffer_upbound3i(0)); ++x)
    for (int y = (case_index == 2 ? bound : mapping_data_.ringbuffer_lowbound3i(1)); y <= (case_index == 3 ? bound : mapping_data_.ringbuffer_upbound3i(1)); ++y)
      for (int z = (case_index == 4 ? bound : mapping_data_.ringbuffer_lowbound3i(2)); z <= (case_index == 5 ? bound : mapping_data_.ringbuffer_upbound3i(2)); ++z)
      {
        Eigen::Vector3i id_global_inf_clr((case_index == 0 ? x + mapping_parameters_.inf_grid : (case_index == 1 ? x - mapping_parameters_.inf_grid : x)),
                                          (case_index == 2 ? y + mapping_parameters_.inf_grid : (case_index == 3 ? y - mapping_parameters_.inf_grid : y)),
                                          (case_index == 4 ? z + mapping_parameters_.inf_grid : (case_index == 5 ? z - mapping_parameters_.inf_grid : z)));
        int id_buf_inf_clr = GlobalIndexToInflatedBufferIndex(id_global_inf_clr);
        if (mapping_data_.occupancy_buffer_inflate[id_buf_inf_clr] != 0)
        {
          ROS_ERROR("Here should be 0!!! md_.occupancy_buffer_inflate_[id_buf_inf_clr]=%d", mapping_data_.occupancy_buffer_inflate[id_buf_inf_clr]);
        }
      }
#endif
}

Eigen::Vector3d GridMap::ClosestPointInMap(const Eigen::Vector3d &pt, const Eigen::Vector3d &camera_pt)
{
  Eigen::Vector3d diff = pt - camera_pt;
  Eigen::Vector3d max_tc = mapping_data_.ringbuffer_upbound3d - camera_pt;
  Eigen::Vector3d min_tc = mapping_data_.ringbuffer_lowbound3d - camera_pt;

  double min_t = 1000000;

  for (int i = 0; i < 3; ++i)
  {
    if (fabs(diff[i]) > 0)
    {

      double t1 = max_tc[i] / diff[i];
      if (t1 > 0 && t1 < min_t)
        min_t = t1;

      double t2 = min_tc[i] / diff[i];
      if (t2 > 0 && t2 < min_t)
        min_t = t2;
    }
  }

  return camera_pt + (min_t - 1e-3) * diff;
}

bool GridMap::NeedsDepthOdometryUpdate()
{
  if (mapping_data_.last_occ_update_time.toSec() < 1.0)
  {
    mapping_data_.last_occ_update_time = ros::Time::now();
  }
  if (!mapping_data_.occ_need_update)
  {
    if (mapping_data_.flag_have_ever_received_depth && (ros::Time::now() - mapping_data_.last_occ_update_time).toSec() > mapping_parameters_.odom_depth_timeout)
    {
      ROS_ERROR("odom or depth lost! ros::Time::now()=%f, md_.last_occ_update_time_=%f, mp_.odom_depth_timeout_=%f",
                ros::Time::now().toSec(), mapping_data_.last_occ_update_time.toSec(), mapping_parameters_.odom_depth_timeout);
      mapping_data_.flag_depth_odom_timeout = true;
    }
    return false;
  }
  mapping_data_.last_occ_update_time = ros::Time::now();

  return true;
}

void GridMap::PublishMap()
{

  if (map_pub_.getNumSubscribers() <= 0)
    return;

  Eigen::Vector3d heading = (mapping_data_.camera_r_m * mapping_data_.camera_to_body.block<3, 3>(0, 0).transpose()).block<3, 1>(0, 0);
  pcl::PointCloud<pcl::PointXYZ> cloud;
  double lbz = mapping_parameters_.enable_virtual_wall ? max(mapping_data_.ringbuffer_lowbound3d(2), mapping_parameters_.virtual_ground) : mapping_data_.ringbuffer_lowbound3d(2);
  double ubz = mapping_parameters_.enable_virtual_wall ? min(mapping_data_.ringbuffer_upbound3d(2), mapping_parameters_.virtual_ceil) : mapping_data_.ringbuffer_upbound3d(2);
  if (mapping_data_.ringbuffer_upbound3d(0) - mapping_data_.ringbuffer_lowbound3d(0) > mapping_parameters_.resolution && (mapping_data_.ringbuffer_upbound3d(1) - mapping_data_.ringbuffer_lowbound3d(1)) > mapping_parameters_.resolution && (ubz - lbz) > mapping_parameters_.resolution)
    for (double xd = mapping_data_.ringbuffer_lowbound3d(0) + mapping_parameters_.resolution / 2; xd <= mapping_data_.ringbuffer_upbound3d(0); xd += mapping_parameters_.resolution)
      for (double yd = mapping_data_.ringbuffer_lowbound3d(1) + mapping_parameters_.resolution / 2; yd <= mapping_data_.ringbuffer_upbound3d(1); yd += mapping_parameters_.resolution)
        for (double zd = lbz + mapping_parameters_.resolution / 2; zd <= ubz; zd += mapping_parameters_.resolution)
        {
          Eigen::Vector3d relative_dir = (Eigen::Vector3d(xd, yd, zd) - mapping_data_.camera_pos);
          if (heading.dot(relative_dir.normalized()) > 0.5)
          {
            if (mapping_data_.occupancy_buffer[GlobalIndexToBufferIndex(PositionToGlobalIndex(Eigen::Vector3d(xd, yd, zd)))] >= mapping_parameters_.min_occupancy_log)
              cloud.push_back(pcl::PointXYZ(xd, yd, zd));
          }
        }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mapping_parameters_.frame_id;
  sensor_msgs::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  map_pub_.publish(cloud_msg);
}

void GridMap::PublishInflatedMap()
{

  if (map_inf_pub_.getNumSubscribers() <= 0)
    return;

  Eigen::Vector3d heading = (mapping_data_.camera_r_m * mapping_data_.camera_to_body.block<3, 3>(0, 0).transpose()).block<3, 1>(0, 0);
  pcl::PointCloud<pcl::PointXYZ> cloud;
  double lbz = mapping_parameters_.enable_virtual_wall ? max(mapping_data_.ringbuffer_inf_lowbound3d(2), mapping_parameters_.virtual_ground) : mapping_data_.ringbuffer_inf_lowbound3d(2);
  double ubz = mapping_parameters_.enable_virtual_wall ? min(mapping_data_.ringbuffer_inf_upbound3d(2), mapping_parameters_.virtual_ceil) : mapping_data_.ringbuffer_inf_upbound3d(2);
  if (mapping_data_.ringbuffer_inf_upbound3d(0) - mapping_data_.ringbuffer_inf_lowbound3d(0) > mapping_parameters_.resolution &&
      (mapping_data_.ringbuffer_inf_upbound3d(1) - mapping_data_.ringbuffer_inf_lowbound3d(1)) > mapping_parameters_.resolution && (ubz - lbz) > mapping_parameters_.resolution)
    for (double xd = mapping_data_.ringbuffer_inf_lowbound3d(0) + mapping_parameters_.resolution / 2; xd < mapping_data_.ringbuffer_inf_upbound3d(0); xd += mapping_parameters_.resolution)
      for (double yd = mapping_data_.ringbuffer_inf_lowbound3d(1) + mapping_parameters_.resolution / 2; yd < mapping_data_.ringbuffer_inf_upbound3d(1); yd += mapping_parameters_.resolution)
        for (double zd = lbz + mapping_parameters_.resolution / 2; zd < ubz; zd += mapping_parameters_.resolution)
        {
          Eigen::Vector3d relative_dir = (Eigen::Vector3d(xd, yd, zd) - mapping_data_.camera_pos);
          if (heading.dot(relative_dir.normalized()) > 0.0)
          {
            if (mapping_data_.occupancy_buffer_inflate[GlobalIndexToInflatedBufferIndex(PositionToGlobalIndex(Eigen::Vector3d(xd, yd, zd)))])
              cloud.push_back(pcl::PointXYZ(xd, yd, zd));
          }
        }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mapping_parameters_.frame_id;
  sensor_msgs::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  map_inf_pub_.publish(cloud_msg);
}

void GridMap::TestIndexingCost()
{
  if (!mapping_parameters_.have_initialized)
    return;

  ros::Time t0 = ros::Time::now();
  double a = 0;
  int b = 0;
  for (int i = 0; i < 10; ++i)
    for (int x = mapping_data_.ringbuffer_lowbound3i(0); x <= mapping_data_.ringbuffer_upbound3i(0); ++x)
      for (int y = mapping_data_.ringbuffer_lowbound3i(1); y <= mapping_data_.ringbuffer_upbound3i(1); ++y)
        for (int z = mapping_data_.ringbuffer_lowbound3i(2); z <= mapping_data_.ringbuffer_upbound3i(2); ++z)
        {
          b += x + y + z;
        }
  ros::Time t1 = ros::Time::now();

  for (int i = 0; i < 10; ++i)
    for (int x = mapping_data_.ringbuffer_lowbound3i(0); x <= mapping_data_.ringbuffer_upbound3i(0); ++x)
      for (int y = mapping_data_.ringbuffer_lowbound3i(1); y <= mapping_data_.ringbuffer_upbound3i(1); ++y)
        for (int z = mapping_data_.ringbuffer_lowbound3i(2); z <= mapping_data_.ringbuffer_upbound3i(2); ++z)
        {
          int id_buf_inf_clr = GlobalIndexToInflatedBufferIndex(Eigen::Vector3i(x, y, z)); // 8396us = 7970
          b += id_buf_inf_clr;
          b += x + y + z;
        }
  ros::Time t2 = ros::Time::now();

  for (int i = 0; i < 10; ++i)
    for (int x = mapping_data_.ringbuffer_lowbound3i(0); x <= mapping_data_.ringbuffer_upbound3i(0); ++x)
      for (int y = mapping_data_.ringbuffer_lowbound3i(1); y <= mapping_data_.ringbuffer_upbound3i(1); ++y)
        for (int z = mapping_data_.ringbuffer_lowbound3i(2); z <= mapping_data_.ringbuffer_upbound3i(2); ++z)
        {
          Eigen::Vector3d pos = GlobalIndexToPosition(Eigen::Vector3i(x, y, z)); // 6553us = 6127
          a += pos.sum();
          b += x + y + z;
        }
  ros::Time t3 = ros::Time::now();

  for (int i = 0; i < 10; ++i)
    for (double xd = mapping_data_.ringbuffer_lowbound3d(0); xd <= mapping_data_.ringbuffer_upbound3d(0); xd += mapping_parameters_.resolution)
      for (double yd = mapping_data_.ringbuffer_lowbound3d(1); yd <= mapping_data_.ringbuffer_upbound3d(1); yd += mapping_parameters_.resolution)
        for (double zd = mapping_data_.ringbuffer_lowbound3d(2); zd <= mapping_data_.ringbuffer_upbound3d(2); zd += mapping_parameters_.resolution)
        {
          a += xd + yd + zd;
        }
  ros::Time t4 = ros::Time::now();

  for (int i = 0; i < 10; ++i)
    for (double xd = mapping_data_.ringbuffer_lowbound3d(0); xd <= mapping_data_.ringbuffer_upbound3d(0); xd += mapping_parameters_.resolution)
      for (double yd = mapping_data_.ringbuffer_lowbound3d(1); yd <= mapping_data_.ringbuffer_upbound3d(1); yd += mapping_parameters_.resolution)
        for (double zd = mapping_data_.ringbuffer_lowbound3d(2); zd <= mapping_data_.ringbuffer_upbound3d(2); zd += mapping_parameters_.resolution)
        {
          Eigen::Vector3i idx = PositionToGlobalIndex(Eigen::Vector3d(xd, yd, zd)); // 7088us = 478us
          a += xd + yd + zd;
          b += idx.sum();
        }
  ros::Time t5 = ros::Time::now();

  for (int i = 0; i < 10; ++i)
    for (double xd = mapping_data_.ringbuffer_lowbound3d(0); xd <= mapping_data_.ringbuffer_upbound3d(0); xd += mapping_parameters_.resolution)
      for (double yd = mapping_data_.ringbuffer_lowbound3d(1); yd <= mapping_data_.ringbuffer_upbound3d(1); yd += mapping_parameters_.resolution)
        for (double zd = mapping_data_.ringbuffer_lowbound3d(2); zd <= mapping_data_.ringbuffer_upbound3d(2); zd += mapping_parameters_.resolution)
        {
          int id_buf_inf_clr = GlobalIndexToInflatedBufferIndex(PositionToGlobalIndex(Eigen::Vector3d(xd, yd, zd)));
          a += xd + yd + zd;
          b += id_buf_inf_clr;
        }
  ros::Time t6 = ros::Time::now();

  for (int i = 0; i < 10; ++i)
    for (size_t i = 0; i < mapping_data_.occupancy_buffer.size(); ++i)
    {
      b += i;
    }
  ros::Time t7 = ros::Time::now();

  for (int i = 0; i < 10; ++i)
    for (size_t i = 0; i < mapping_data_.occupancy_buffer.size(); ++i)
    {
      Eigen::Vector3i idx = BufferIndexToGlobalIndex(i); // 36939
      b += i;
      b += idx.sum();
    }
  ros::Time t8 = ros::Time::now();

  int n = mapping_data_.occupancy_buffer.size() * 10;

  cout << "a=" << a << " b=" << b << endl;
  printf("iter=%d, t1-t0=%f, t2-t1=%f, t3-t2=%f, t4-t3=%f, t5-t4=%f, t6-t5=%f, t7-t6=%f, t8-t7=%f\n", n, (t1 - t0).toSec(), (t2 - t1).toSec(), (t3 - t2).toSec(), (t4 - t3).toSec(), (t5 - t4).toSec(), (t6 - t5).toSec(), (t7 - t6).toSec(), (t8 - t7).toSec());
  printf("globalIdx2InfBufIdx():%fns(1.88), globalIdx2Pos():%fns(0.70), pos2GlobalIdx():%fns(1.11), globalIdx2InfBufIdx(pos2GlobalIdx()):%fns(3.56), BufIdx2GlobalIdx():%fns(10.05)\n",
         ((t2 - t1) - (t1 - t0)).toSec() * 1e9 / n, ((t3 - t2) - (t1 - t0)).toSec() * 1e9 / n, ((t5 - t4) - (t4 - t3)).toSec() * 1e9 / n, ((t6 - t5) - (t4 - t3)).toSec() * 1e9 / n, ((t8 - t7) - (t7 - t6)).toSec() * 1e9 / n);
}
