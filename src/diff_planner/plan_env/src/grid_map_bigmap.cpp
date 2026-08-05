#include "plan_env/grid_map_bigmap.h"

// #define current_img_ mapping_data_.depth_image[image_cnt_ & 1]
// #define last_img_ mapping_data_.depth_image[!(image_cnt_ & 1)]

void GridMap::InitMap(ros::NodeHandle &nh)
{
  node_ = nh;

  /* get parameter */
  double x_size, y_size, z_size;
  node_.param("grid_map/resolution", mapping_parameters_.resolution, -1.0);
  node_.param("grid_map/map_size_x", x_size, -1.0);
  node_.param("grid_map/map_size_y", y_size, -1.0);
  node_.param("grid_map/map_size_z", z_size, -1.0);
  node_.param("grid_map/local_update_range_x", mapping_parameters_.local_update_range(0), -1.0);
  node_.param("grid_map/local_update_range_y", mapping_parameters_.local_update_range(1), -1.0);
  node_.param("grid_map/local_update_range_z", mapping_parameters_.local_update_range(2), -1.0);
  node_.param("grid_map/obstacles_inflation", mapping_parameters_.obstacles_inflation, -1.0);

  node_.param("grid_map/fx", mapping_parameters_.fx, -1.0);
  node_.param("grid_map/fy", mapping_parameters_.fy, -1.0);
  node_.param("grid_map/cx", mapping_parameters_.cx, -1.0);
  node_.param("grid_map/cy", mapping_parameters_.cy, -1.0);

  node_.param("grid_map/use_depth_filter", mapping_parameters_.use_depth_filter, true);
  node_.param("grid_map/depth_filter_tolerance", mapping_parameters_.depth_filter_tolerance, -1.0);
  node_.param("grid_map/depth_filter_maxdist", mapping_parameters_.depth_filter_maxdist, -1.0);
  node_.param("grid_map/depth_filter_mindist", mapping_parameters_.depth_filter_mindist, -1.0);
  node_.param("grid_map/depth_filter_margin", mapping_parameters_.depth_filter_margin, -1);
  node_.param("grid_map/k_depth_scaling_factor", mapping_parameters_.depth_scaling_factor, -1.0);
  node_.param("grid_map/skip_pixel", mapping_parameters_.skip_pixel, -1);

  node_.param("grid_map/p_hit", mapping_parameters_.p_hit, 0.70);
  node_.param("grid_map/p_miss", mapping_parameters_.p_miss, 0.35);
  node_.param("grid_map/p_min", mapping_parameters_.p_min, 0.12);
  node_.param("grid_map/p_max", mapping_parameters_.p_max, 0.97);
  node_.param("grid_map/p_occ", mapping_parameters_.p_occ, 0.80);
  node_.param("grid_map/fading_time", mapping_parameters_.fading_time, 1000.0);
  node_.param("grid_map/min_ray_length", mapping_parameters_.min_ray_length, -0.1);
  node_.param("grid_map/max_ray_length", mapping_parameters_.max_ray_length, -0.1);

  node_.param("grid_map/visualization_truncate_height", mapping_parameters_.visualization_truncate_height, -0.1);

  node_.param("grid_map/show_occ_time", mapping_parameters_.show_occ_time, false);
  node_.param("grid_map/pose_type", mapping_parameters_.pose_type, 1);

  node_.param("grid_map/frame_id", mapping_parameters_.frame_id, string("world"));
  node_.param("grid_map/local_map_margin", mapping_parameters_.local_map_margin, 1);
  node_.param("grid_map/ground_height", mapping_parameters_.ground_height, 0.0);

  node_.param("grid_map/odom_depth_timeout", mapping_parameters_.odom_depth_timeout, 1.0);

  mapping_parameters_.resolution_inv = 1 / mapping_parameters_.resolution;
  mapping_parameters_.map_origin = Eigen::Vector3d(-x_size / 2.0, -y_size / 2.0, mapping_parameters_.ground_height);
  mapping_parameters_.map_size = Eigen::Vector3d(x_size, y_size, z_size);

  mapping_parameters_.prob_hit_log = PLAN_ENV_BIG_MAP_LOGIT(mapping_parameters_.p_hit);
  mapping_parameters_.prob_miss_log = PLAN_ENV_BIG_MAP_LOGIT(mapping_parameters_.p_miss);
  mapping_parameters_.clamp_min_log = PLAN_ENV_BIG_MAP_LOGIT(mapping_parameters_.p_min);
  mapping_parameters_.clamp_max_log = PLAN_ENV_BIG_MAP_LOGIT(mapping_parameters_.p_max);
  mapping_parameters_.min_occupancy_log = PLAN_ENV_BIG_MAP_LOGIT(mapping_parameters_.p_occ);
  mapping_parameters_.unknown_flag = 0.01;

  cout << "hit: " << mapping_parameters_.prob_hit_log << endl;
  cout << "miss: " << mapping_parameters_.prob_miss_log << endl;
  cout << "min log: " << mapping_parameters_.clamp_min_log << endl;
  cout << "max: " << mapping_parameters_.clamp_max_log << endl;
  cout << "thresh log: " << mapping_parameters_.min_occupancy_log << endl;

  for (int i = 0; i < 3; ++i)
    mapping_parameters_.map_voxel_num(i) = ceil(mapping_parameters_.map_size(i) / mapping_parameters_.resolution);

  mapping_parameters_.map_min_boundary = mapping_parameters_.map_origin;
  mapping_parameters_.map_max_boundary = mapping_parameters_.map_origin + mapping_parameters_.map_size;

  // initialize data buffers

  int buffer_size = mapping_parameters_.map_voxel_num(0) * mapping_parameters_.map_voxel_num(1) * mapping_parameters_.map_voxel_num(2);

  mapping_data_.occupancy_buffer = vector<double>(buffer_size, mapping_parameters_.clamp_min_log - mapping_parameters_.unknown_flag);
  mapping_data_.occupancy_buffer_inflate = vector<char>(buffer_size, 0);

  mapping_data_.count_hit_and_miss = vector<short>(buffer_size, 0);
  mapping_data_.count_hit = vector<short>(buffer_size, 0);
  mapping_data_.flag_rayend = vector<char>(buffer_size, -1);
  mapping_data_.flag_traverse = vector<char>(buffer_size, -1);

  mapping_data_.raycast_num = 0;

  mapping_data_.projected_points.resize(640 * 480 / mapping_parameters_.skip_pixel / mapping_parameters_.skip_pixel);
  mapping_data_.projected_point_count = 0;

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
  indep_cloud_sub_ =
      node_.subscribe<sensor_msgs::PointCloud2>("grid_map/cloud", 10, &GridMap::PointCloudCallback, this);
  indep_odom_sub_ =
      node_.subscribe<nav_msgs::Odometry>("grid_map/odom", 10, &GridMap::OdometryCallback, this);

  occ_timer_ = node_.createTimer(ros::Duration(0.05), &GridMap::UpdateOccupancyCallback, this);
  vis_timer_ = node_.createTimer(ros::Duration(0.05), &GridMap::VisualizationCallback, this);
  fading_timer_ = node_.createTimer(ros::Duration(0.5), &GridMap::FadingCallback, this);

  map_pub_ = node_.advertise<sensor_msgs::PointCloud2>("grid_map/occupancy", 10);
  map_inf_pub_ = node_.advertise<sensor_msgs::PointCloud2>("grid_map/occupancy_inflate", 10);

  mapping_data_.occ_need_update = false;
  mapping_data_.local_updated = false;
  mapping_data_.has_first_depth = false;
  mapping_data_.has_odom = false;
  mapping_data_.has_cloud = false;
  mapping_data_.image_count = 0;
  mapping_data_.last_occ_update_time.fromSec(0);

  mapping_data_.fuse_time = 0.0;
  mapping_data_.update_num = 0;
  mapping_data_.max_fuse_time = 0.0;

  mapping_data_.flag_depth_odom_timeout = false;
  mapping_data_.flag_use_depth_fusion = false;

  // rand_noise_ = uniform_real_distribution<double>(-0.2, 0.2);
  // rand_noise2_ = normal_distribution<double>(0, 0.2);
  // random_device rd;
  // eng_ = default_random_engine(rd());
}

void GridMap::ResetBuffer()
{
  Eigen::Vector3d min_pos = mapping_parameters_.map_min_boundary;
  Eigen::Vector3d max_pos = mapping_parameters_.map_max_boundary;

  ResetBuffer(min_pos, max_pos);

  mapping_data_.local_bound_min = Eigen::Vector3i::Zero();
  mapping_data_.local_bound_max = mapping_parameters_.map_voxel_num - Eigen::Vector3i::Ones();
}

void GridMap::ResetBuffer(Eigen::Vector3d min_pos, Eigen::Vector3d max_pos)
{

  Eigen::Vector3i min_id, max_id;
  PositionToIndex(min_pos, min_id);
  PositionToIndex(max_pos, max_id);

  BoundIndex(min_id);
  BoundIndex(max_id);

  /* reset occ and dist buffer */
  for (int x = min_id(0); x <= max_id(0); ++x)
    for (int y = min_id(1); y <= max_id(1); ++y)
      for (int z = min_id(2); z <= max_id(2); ++z)
      {
        mapping_data_.occupancy_buffer_inflate[ToAddress(x, y, z)] = 0;
      }
}

int GridMap::SetCachedOccupancy(Eigen::Vector3d pos, int occ)
{
  if (occ != 1 && occ != 0)
    return kInvalidIndex;

  Eigen::Vector3i id;
  PositionToIndex(pos, id);
  int idx_ctns = ToAddress(id);

  mapping_data_.count_hit_and_miss[idx_ctns] += 1;

  if (mapping_data_.count_hit_and_miss[idx_ctns] == 1)
  {
    mapping_data_.cache_voxel.push(id);
  }

  if (occ == 1)
    mapping_data_.count_hit[idx_ctns] += 1;

  return idx_ctns;
}

void GridMap::ProjectDepthImage()
{
  // mapping_data_.projected_points.clear();
  mapping_data_.projected_point_count = 0;

  uint16_t *row_ptr;
  // int cols = current_img_.cols, rows = current_img_.rows;
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
        depth = (*row_ptr++) / mapping_parameters_.depth_scaling_factor;
        proj_pt(0) = (u - mapping_parameters_.cx) * depth / mapping_parameters_.fx;
        proj_pt(1) = (v - mapping_parameters_.cy) * depth / mapping_parameters_.fy;
        proj_pt(2) = depth;

        proj_pt = camera_r * proj_pt + mapping_data_.camera_pos;

        if (u == 320 && v == 240)
          std::cout << "depth: " << depth << std::endl;
        mapping_data_.projected_points[mapping_data_.projected_point_count++] = proj_pt;
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

          if (*row_ptr == 0)
          {
            depth = mapping_parameters_.max_ray_length + 0.1;
          }
          else if (depth < mapping_parameters_.depth_filter_mindist)
          {
            continue;
          }
          else if (depth > mapping_parameters_.depth_filter_maxdist)
          {
            depth = mapping_parameters_.max_ray_length + 0.1;
          }

          // project to world frame
          pt_cur(0) = (u - mapping_parameters_.cx) * depth / mapping_parameters_.fx;
          pt_cur(1) = (v - mapping_parameters_.cy) * depth / mapping_parameters_.fy;
          pt_cur(2) = depth;

          pt_world = camera_r * pt_cur + mapping_data_.camera_pos;
          // if (!IsInMap(pt_world)) {
          //   pt_world = ClosestPointInMap(pt_world, mapping_data_.camera_pos);
          // }

          mapping_data_.projected_points[mapping_data_.projected_point_count++] = pt_world;

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
                mapping_data_.projected_points[mapping_data_.projected_point_count++] = pt_world;
              }
            }
            else
            {
              mapping_data_.projected_points[mapping_data_.projected_point_count++] = pt_world;
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

void GridMap::ProcessRaycast()
{
  // if (mapping_data_.projected_points.size() == 0)
  if (mapping_data_.projected_point_count == 0)
    return;

  ros::WallTime t1, t2, t3;

  mapping_data_.raycast_num += 1;

  int vox_idx;
  double length;

  // bounding box of updated region
  double min_x = mapping_parameters_.map_max_boundary(0);
  double min_y = mapping_parameters_.map_max_boundary(1);
  double min_z = mapping_parameters_.map_max_boundary(2);

  double max_x = mapping_parameters_.map_min_boundary(0);
  double max_y = mapping_parameters_.map_min_boundary(1);
  double max_z = mapping_parameters_.map_min_boundary(2);

  RayCaster raycaster;
  Eigen::Vector3d half = Eigen::Vector3d(0.5, 0.5, 0.5);
  Eigen::Vector3d ray_pt, pt_w;

  int pts_num = 0;
  t1 = ros::WallTime::now();
  for (int i = 0; i < mapping_data_.projected_point_count; ++i)
  {
    pt_w = mapping_data_.projected_points[i];

    // set flag for projected point

    if (!IsInMap(pt_w))
    {
      pt_w = ClosestPointInMap(pt_w, mapping_data_.camera_pos);

      length = (pt_w - mapping_data_.camera_pos).norm();
      if (length > mapping_parameters_.max_ray_length)
      {
        pt_w = (pt_w - mapping_data_.camera_pos) / length * mapping_parameters_.max_ray_length + mapping_data_.camera_pos;
      }
      vox_idx = SetCachedOccupancy(pt_w, 0);
    }
    else
    {
      length = (pt_w - mapping_data_.camera_pos).norm();

      if (length > mapping_parameters_.max_ray_length)
      {
        pt_w = (pt_w - mapping_data_.camera_pos) / length * mapping_parameters_.max_ray_length + mapping_data_.camera_pos;
        vox_idx = SetCachedOccupancy(pt_w, 0);
      }
      else
      {
        vox_idx = SetCachedOccupancy(pt_w, 1);
      }
    }

    max_x = max(max_x, pt_w(0));
    max_y = max(max_y, pt_w(1));
    max_z = max(max_z, pt_w(2));

    min_x = min(min_x, pt_w(0));
    min_y = min(min_y, pt_w(1));
    min_z = min(min_z, pt_w(2));

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
      Eigen::Vector3d tmp = (ray_pt + half) * mapping_parameters_.resolution;
      length = (tmp - mapping_data_.camera_pos).norm();

      if (length < mapping_parameters_.min_ray_length)
        break;

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

  t2 = ros::WallTime::now();

  min_x = min(min_x, mapping_data_.camera_pos(0));
  min_y = min(min_y, mapping_data_.camera_pos(1));
  min_z = min(min_z, mapping_data_.camera_pos(2));

  max_x = max(max_x, mapping_data_.camera_pos(0));
  max_y = max(max_y, mapping_data_.camera_pos(1));
  max_z = max(max_z, mapping_data_.camera_pos(2));
  max_z = max(max_z, mapping_parameters_.ground_height);

  PositionToIndex(Eigen::Vector3d(max_x, max_y, max_z), mapping_data_.local_bound_max);
  PositionToIndex(Eigen::Vector3d(min_x, min_y, min_z), mapping_data_.local_bound_min);
  BoundIndex(mapping_data_.local_bound_min);
  BoundIndex(mapping_data_.local_bound_max);

  mapping_data_.local_updated = true;

  // update occupancy cached in queue
  Eigen::Vector3d local_range_min = mapping_data_.camera_pos - mapping_parameters_.local_update_range;
  Eigen::Vector3d local_range_max = mapping_data_.camera_pos + mapping_parameters_.local_update_range;

  Eigen::Vector3i min_id, max_id;
  PositionToIndex(local_range_min, min_id);
  PositionToIndex(local_range_max, max_id);
  BoundIndex(min_id);
  BoundIndex(max_id);

  std::cout << "cache all: " << mapping_data_.cache_voxel.size() << std::endl;

  while (!mapping_data_.cache_voxel.empty())
  {

    Eigen::Vector3i idx = mapping_data_.cache_voxel.front();
    int idx_ctns = ToAddress(idx);
    mapping_data_.cache_voxel.pop();

    double log_odds_update =
        mapping_data_.count_hit[idx_ctns] >= mapping_data_.count_hit_and_miss[idx_ctns] - mapping_data_.count_hit[idx_ctns] ? mapping_parameters_.prob_hit_log : mapping_parameters_.prob_miss_log;

    mapping_data_.count_hit[idx_ctns] = mapping_data_.count_hit_and_miss[idx_ctns] = 0;

    if (log_odds_update >= 0 && mapping_data_.occupancy_buffer[idx_ctns] >= mapping_parameters_.clamp_max_log)
    {
      continue;
    }
    else if (log_odds_update <= 0 && mapping_data_.occupancy_buffer[idx_ctns] <= mapping_parameters_.clamp_min_log)
    {
      // mapping_data_.occupancy_buffer[idx_ctns] = mapping_parameters_.clamp_min_log;
      continue;
    }

    // bool in_local = idx(0) >= min_id(0) && idx(0) <= max_id(0) && idx(1) >= min_id(1) &&
    //                 idx(1) <= max_id(1) && idx(2) >= min_id(2) && idx(2) <= max_id(2);
    // if (!in_local)
    // {
    //   mapping_data_.occupancy_buffer[idx_ctns] = mapping_parameters_.clamp_min_log;
    // }

    mapping_data_.occupancy_buffer[idx_ctns] =
        std::min(std::max(mapping_data_.occupancy_buffer[idx_ctns] + log_odds_update, mapping_parameters_.clamp_min_log),
                 mapping_parameters_.clamp_max_log);
  }

  t3 = ros::WallTime::now();

  if ( mapping_parameters_.show_occ_time )
  {
    ROS_WARN("Raycast time: t2-t1=%f, t3-t2=%f, pts_num=%d", (t2-t1).toSec(), (t3-t2).toSec(), pts_num);
  }
}

Eigen::Vector3d GridMap::ClosestPointInMap(const Eigen::Vector3d &pt, const Eigen::Vector3d &camera_pt)
{
  Eigen::Vector3d diff = pt - camera_pt;
  Eigen::Vector3d max_tc = mapping_parameters_.map_max_boundary - camera_pt;
  Eigen::Vector3d min_tc = mapping_parameters_.map_min_boundary - camera_pt;

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

void GridMap::ClearAndInflateLocalMap()
{
  /*clear outside local*/
  const int vec_margin = 5;
  // Eigen::Vector3i min_vec_margin = min_vec - Eigen::Vector3i(vec_margin,
  // vec_margin, vec_margin); Eigen::Vector3i max_vec_margin = max_vec +
  // Eigen::Vector3i(vec_margin, vec_margin, vec_margin);

  Eigen::Vector3i min_cut = mapping_data_.local_bound_min -
                            Eigen::Vector3i(mapping_parameters_.local_map_margin, mapping_parameters_.local_map_margin, mapping_parameters_.local_map_margin);
  Eigen::Vector3i max_cut = mapping_data_.local_bound_max +
                            Eigen::Vector3i(mapping_parameters_.local_map_margin, mapping_parameters_.local_map_margin, mapping_parameters_.local_map_margin);
  BoundIndex(min_cut);
  BoundIndex(max_cut);

  Eigen::Vector3i min_cut_m = min_cut - Eigen::Vector3i(vec_margin, vec_margin, vec_margin);
  Eigen::Vector3i max_cut_m = max_cut + Eigen::Vector3i(vec_margin, vec_margin, vec_margin);
  BoundIndex(min_cut_m);
  BoundIndex(max_cut_m);

  // clear data outside the local range

  for (int x = min_cut_m(0); x <= max_cut_m(0); ++x)
    for (int y = min_cut_m(1); y <= max_cut_m(1); ++y)
    {

      for (int z = min_cut_m(2); z < min_cut(2); ++z)
      {
        int idx = ToAddress(x, y, z);
        mapping_data_.occupancy_buffer[idx] = mapping_parameters_.clamp_min_log - mapping_parameters_.unknown_flag;
      }

      for (int z = max_cut(2) + 1; z <= max_cut_m(2); ++z)
      {
        int idx = ToAddress(x, y, z);
        mapping_data_.occupancy_buffer[idx] = mapping_parameters_.clamp_min_log - mapping_parameters_.unknown_flag;
      }
    }

  for (int z = min_cut_m(2); z <= max_cut_m(2); ++z)
    for (int x = min_cut_m(0); x <= max_cut_m(0); ++x)
    {

      for (int y = min_cut_m(1); y < min_cut(1); ++y)
      {
        int idx = ToAddress(x, y, z);
        mapping_data_.occupancy_buffer[idx] = mapping_parameters_.clamp_min_log - mapping_parameters_.unknown_flag;
      }

      for (int y = max_cut(1) + 1; y <= max_cut_m(1); ++y)
      {
        int idx = ToAddress(x, y, z);
        mapping_data_.occupancy_buffer[idx] = mapping_parameters_.clamp_min_log - mapping_parameters_.unknown_flag;
      }
    }

  for (int y = min_cut_m(1); y <= max_cut_m(1); ++y)
    for (int z = min_cut_m(2); z <= max_cut_m(2); ++z)
    {

      for (int x = min_cut_m(0); x < min_cut(0); ++x)
      {
        int idx = ToAddress(x, y, z);
        mapping_data_.occupancy_buffer[idx] = mapping_parameters_.clamp_min_log - mapping_parameters_.unknown_flag;
      }

      for (int x = max_cut(0) + 1; x <= max_cut_m(0); ++x)
      {
        int idx = ToAddress(x, y, z);
        mapping_data_.occupancy_buffer[idx] = mapping_parameters_.clamp_min_log - mapping_parameters_.unknown_flag;
      }
    }

  // inflate occupied voxels to compensate robot size

  int inf_step = ceil((mapping_parameters_.obstacles_inflation - 0.001) / mapping_parameters_.resolution);
  if (inf_step > 4)
  {
    ROS_ERROR("Inflation is too big, which will cause siginificant computation! Reduce inflation or enlarge resolution.");
  }
  vector<Eigen::Vector3i> inf_pts(pow(2 * inf_step + 1, 3));
  // inf_pts.resize(4 * inf_step + 3);
  Eigen::Vector3i inf_pt;

  // clear outdated data
  for (int x = mapping_data_.local_bound_min(0); x <= mapping_data_.local_bound_max(0); ++x)
    for (int y = mapping_data_.local_bound_min(1); y <= mapping_data_.local_bound_max(1); ++y)
      for (int z = mapping_data_.local_bound_min(2); z <= mapping_data_.local_bound_max(2); ++z)
      {
        mapping_data_.occupancy_buffer_inflate[ToAddress(x, y, z)] = 0;
      }

  // inflate obstacles
  for (int x = mapping_data_.local_bound_min(0); x <= mapping_data_.local_bound_max(0); ++x)
    for (int y = mapping_data_.local_bound_min(1); y <= mapping_data_.local_bound_max(1); ++y)
      for (int z = mapping_data_.local_bound_min(2); z <= mapping_data_.local_bound_max(2); ++z)
      {

        if (mapping_data_.occupancy_buffer[ToAddress(x, y, z)] > mapping_parameters_.min_occupancy_log)
        {
          InflatePoint(Eigen::Vector3i(x, y, z), inf_step, inf_pts);

          for (int k = 0; k < (int)inf_pts.size(); ++k)
          {
            inf_pt = inf_pts[k];
            int idx_inf = ToAddress(inf_pt);
            if (idx_inf < 0 ||
                idx_inf >= mapping_parameters_.map_voxel_num(0) * mapping_parameters_.map_voxel_num(1) * mapping_parameters_.map_voxel_num(2))
            {
              continue;
            }
            mapping_data_.occupancy_buffer_inflate[idx_inf] = 1;
          }
        }
      }
}

void GridMap::VisualizationCallback(const ros::TimerEvent & /*event*/)
{
  PublishInflatedMap(true);
  PublishMap();
}

void GridMap::FadingCallback(const ros::TimerEvent & /*event*/)
{
  Eigen::Vector3d local_range_min = mapping_data_.camera_pos - mapping_parameters_.local_update_range;
  Eigen::Vector3d local_range_max = mapping_data_.camera_pos + mapping_parameters_.local_update_range;

  Eigen::Vector3i min_id, max_id;
  PositionToIndex(local_range_min, min_id);
  PositionToIndex(local_range_max, max_id);
  BoundIndex(min_id);
  BoundIndex(max_id);

  const double reduce = (mapping_parameters_.clamp_max_log - mapping_parameters_.min_occupancy_log) / (mapping_parameters_.fading_time * 2); // function called at 2Hz
  const double low_thres = mapping_parameters_.clamp_min_log + reduce;

  for (int x = min_id(0); x <= max_id(0); ++x)
    for (int y = min_id(1); y <= max_id(1); ++y)
      for (int z = min_id(2); z <= max_id(2); ++z)
      {
        int address = ToAddress(x, y, z);
        if (mapping_data_.occupancy_buffer[address] > low_thres)
        {
          mapping_data_.occupancy_buffer[address] -= reduce;
        }
      }
}

void GridMap::UpdateOccupancyCallback(const ros::TimerEvent & /*event*/)
{
  if (mapping_data_.last_occ_update_time.toSec() < 1.0)
    mapping_data_.last_occ_update_time = ros::Time::now();

  if (!mapping_data_.occ_need_update)
  {
    if (mapping_data_.flag_use_depth_fusion && (ros::Time::now() - mapping_data_.last_occ_update_time).toSec() > mapping_parameters_.odom_depth_timeout)
    {
      ROS_ERROR("odom or depth lost! ros::Time::now()=%f, md_.last_occ_update_time_=%f, mp_.odom_depth_timeout_=%f",
                ros::Time::now().toSec(), mapping_data_.last_occ_update_time.toSec(), mapping_parameters_.odom_depth_timeout);
      mapping_data_.flag_depth_odom_timeout = true;
    }
    return;
  }
  mapping_data_.last_occ_update_time = ros::Time::now();

  /* update occupancy */
  ros::Time t1, t2, t3, t4;
  t1 = ros::Time::now();

  ProjectDepthImage();
  t2 = ros::Time::now();
  ProcessRaycast();
  t3 = ros::Time::now();

  if (mapping_data_.local_updated)
    ClearAndInflateLocalMap();

  t4 = ros::Time::now();

  if (mapping_parameters_.show_occ_time)
  {
    cout << setprecision(7);
    cout << "t2=" << (t2 - t1).toSec() << " t3=" << (t3 - t2).toSec() << " t4=" << (t4 - t3).toSec() << endl;

    mapping_data_.fuse_time += (t3 - t2).toSec();
    mapping_data_.max_fuse_time = max(mapping_data_.max_fuse_time, (t3 - t2).toSec());
    mapping_data_.update_num += 1;

    ROS_WARN("Fusion: cur t = %lf, avg t = %lf, max t = %lf", (t3 - t2).toSec(),
             mapping_data_.fuse_time / mapping_data_.update_num, mapping_data_.max_fuse_time);
  }

  mapping_data_.occ_need_update = false;
  mapping_data_.local_updated = false;
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

  // std::cout << "depth: " << mapping_data_.depth_image.cols << ", " << mapping_data_.depth_image.rows << std::endl;

  /* get pose */
  mapping_data_.camera_pos(0) = pose->pose.position.x;
  mapping_data_.camera_pos(1) = pose->pose.position.y;
  mapping_data_.camera_pos(2) = pose->pose.position.z;
  mapping_data_.camera_r_m = Eigen::Quaterniond(pose->pose.orientation.w, pose->pose.orientation.x,
                                       pose->pose.orientation.y, pose->pose.orientation.z)
                        .toRotationMatrix();
  if (IsInMap(mapping_data_.camera_pos))
  {
    mapping_data_.has_odom = true;
    mapping_data_.occ_need_update = true;
  }
  else
  {
    mapping_data_.occ_need_update = false;
  }

  mapping_data_.flag_use_depth_fusion = true;
}

void GridMap::OdometryCallback(const nav_msgs::OdometryConstPtr &odom)
{
  if (mapping_data_.has_first_depth)
    return;

  mapping_data_.camera_pos(0) = odom->pose.pose.position.x;
  mapping_data_.camera_pos(1) = odom->pose.pose.position.y;
  mapping_data_.camera_pos(2) = odom->pose.pose.position.z;

  mapping_data_.has_odom = true;
}

void GridMap::PointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &img)
{

  pcl::PointCloud<pcl::PointXYZ> latest_cloud;
  pcl::fromROSMsg(*img, latest_cloud);

  mapping_data_.has_cloud = true;

  if (!mapping_data_.has_odom)
  {
    std::cout << "no odom!" << std::endl;
    return;
  }

  if (latest_cloud.points.size() == 0)
    return;

  if (isnan(mapping_data_.camera_pos(0)) || isnan(mapping_data_.camera_pos(1)) || isnan(mapping_data_.camera_pos(2)))
    return;

  this->ResetBuffer(mapping_data_.camera_pos - mapping_parameters_.local_update_range,
                    mapping_data_.camera_pos + mapping_parameters_.local_update_range);

  pcl::PointXYZ pt;
  Eigen::Vector3d p3d, p3d_inf;

  int inf_step = ceil((mapping_parameters_.obstacles_inflation - 0.001) / mapping_parameters_.resolution);
  if (inf_step > 4)
  {
    ROS_ERROR("Inflation is too big, which will cause siginificant computation! Reduce inflation or enlarge resolution.");
  }
  int inf_step_z = 1;

  double max_x, max_y, max_z, min_x, min_y, min_z;

  min_x = mapping_parameters_.map_max_boundary(0);
  min_y = mapping_parameters_.map_max_boundary(1);
  min_z = mapping_parameters_.map_max_boundary(2);

  max_x = mapping_parameters_.map_min_boundary(0);
  max_y = mapping_parameters_.map_min_boundary(1);
  max_z = mapping_parameters_.map_min_boundary(2);

  for (size_t i = 0; i < latest_cloud.points.size(); ++i)
  {
    pt = latest_cloud.points[i];
    p3d(0) = pt.x, p3d(1) = pt.y, p3d(2) = pt.z;

    /* point inside update range */
    Eigen::Vector3d devi = p3d - mapping_data_.camera_pos;
    Eigen::Vector3i inf_pt;

    if (fabs(devi(0)) < mapping_parameters_.local_update_range(0) && fabs(devi(1)) < mapping_parameters_.local_update_range(1) &&
        fabs(devi(2)) < mapping_parameters_.local_update_range(2))
    {

      /* inflate the point */
      for (int x = -inf_step; x <= inf_step; ++x)
        for (int y = -inf_step; y <= inf_step; ++y)
          for (int z = -inf_step_z; z <= inf_step_z; ++z)
          {

            p3d_inf(0) = pt.x + x * mapping_parameters_.resolution;
            p3d_inf(1) = pt.y + y * mapping_parameters_.resolution;
            p3d_inf(2) = pt.z + z * mapping_parameters_.resolution;

            max_x = max(max_x, p3d_inf(0));
            max_y = max(max_y, p3d_inf(1));
            max_z = max(max_z, p3d_inf(2));

            min_x = min(min_x, p3d_inf(0));
            min_y = min(min_y, p3d_inf(1));
            min_z = min(min_z, p3d_inf(2));

            PositionToIndex(p3d_inf, inf_pt);

            if (!IsInMap(inf_pt))
              continue;

            int idx_inf = ToAddress(inf_pt);

            mapping_data_.occupancy_buffer_inflate[idx_inf] = 1;
          }
    }
  }

  min_x = min(min_x, mapping_data_.camera_pos(0));
  min_y = min(min_y, mapping_data_.camera_pos(1));
  min_z = min(min_z, mapping_data_.camera_pos(2));

  max_x = max(max_x, mapping_data_.camera_pos(0));
  max_y = max(max_y, mapping_data_.camera_pos(1));
  max_z = max(max_z, mapping_data_.camera_pos(2));

  max_z = max(max_z, mapping_parameters_.ground_height);

  PositionToIndex(Eigen::Vector3d(max_x, max_y, max_z), mapping_data_.local_bound_max);
  PositionToIndex(Eigen::Vector3d(min_x, min_y, min_z), mapping_data_.local_bound_min);

  BoundIndex(mapping_data_.local_bound_min);
  BoundIndex(mapping_data_.local_bound_max);
}

void GridMap::PublishMap()
{

  if (map_pub_.getNumSubscribers() <= 0)
    return;

  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = mapping_data_.local_bound_min;
  Eigen::Vector3i max_cut = mapping_data_.local_bound_max;

  int lmm = mapping_parameters_.local_map_margin / 2;
  min_cut -= Eigen::Vector3i(lmm, lmm, lmm);
  max_cut += Eigen::Vector3i(lmm, lmm, lmm);

  BoundIndex(min_cut);
  BoundIndex(max_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z)
      {
        if (mapping_data_.occupancy_buffer[ToAddress(x, y, z)] < mapping_parameters_.min_occupancy_log)
          continue;

        Eigen::Vector3d pos;
        IndexToPosition(Eigen::Vector3i(x, y, z), pos);
        if (pos(2) > mapping_parameters_.visualization_truncate_height)
          continue;

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.push_back(pt);
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mapping_parameters_.frame_id;
  sensor_msgs::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  map_pub_.publish(cloud_msg);
}

void GridMap::PublishInflatedMap(bool all_info)
{

  if (map_inf_pub_.getNumSubscribers() <= 0)
    return;

  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = mapping_data_.local_bound_min;
  Eigen::Vector3i max_cut = mapping_data_.local_bound_max;

  if (all_info)
  {
    int lmm = mapping_parameters_.local_map_margin;
    min_cut -= Eigen::Vector3i(lmm, lmm, lmm);
    max_cut += Eigen::Vector3i(lmm, lmm, lmm);
  }

  BoundIndex(min_cut);
  BoundIndex(max_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z)
      {
        if (mapping_data_.occupancy_buffer_inflate[ToAddress(x, y, z)] == 0)
          continue;

        Eigen::Vector3d pos;
        IndexToPosition(Eigen::Vector3i(x, y, z), pos);
        if (pos(2) > mapping_parameters_.visualization_truncate_height)
          continue;

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.push_back(pt);
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mapping_parameters_.frame_id;
  sensor_msgs::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  map_inf_pub_.publish(cloud_msg);

  // ROS_INFO("pub map");
}

bool GridMap::IsOdometryValid() { return mapping_data_.has_odom; }

bool GridMap::HasDepthObservation() { return mapping_data_.has_first_depth; }

Eigen::Vector3d GridMap::GetOrigin() { return mapping_parameters_.map_origin; }

// int GridMap::GetVoxelCount() {
//   return mapping_parameters_.map_voxel_num[0] * mapping_parameters_.map_voxel_num[1] * mapping_parameters_.map_voxel_num[2];
// }

void GridMap::GetRegion(Eigen::Vector3d &ori, Eigen::Vector3d &size)
{
  ori = mapping_parameters_.map_origin, size = mapping_parameters_.map_size;
}

void GridMap::ExtrinsicCallback(const nav_msgs::OdometryConstPtr &odom)
{
  Eigen::Quaterniond cam2body_q = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                                     odom->pose.pose.orientation.x,
                                                     odom->pose.pose.orientation.y,
                                                     odom->pose.pose.orientation.z);
  Eigen::Matrix3d cam2body_r_m = cam2body_q.toRotationMatrix();
  mapping_data_.camera_to_body.block<3, 3>(0, 0) = cam2body_r_m;
  mapping_data_.camera_to_body(0, 3) = odom->pose.pose.position.x;
  mapping_data_.camera_to_body(1, 3) = odom->pose.pose.position.y;
  mapping_data_.camera_to_body(2, 3) = odom->pose.pose.position.z;
  mapping_data_.camera_to_body(3, 3) = 1.0;
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
  Eigen::Matrix4d body2world;
  body2world.block<3, 3>(0, 0) = body_r_m;
  body2world(0, 3) = odom->pose.pose.position.x;
  body2world(1, 3) = odom->pose.pose.position.y;
  body2world(2, 3) = odom->pose.pose.position.z;
  body2world(3, 3) = 1.0;

  Eigen::Matrix4d camera_transform = body2world * mapping_data_.camera_to_body;
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

  mapping_data_.occ_need_update = true;
  mapping_data_.flag_use_depth_fusion = true;
}
