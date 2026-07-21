#ifndef DIFF_PLANNER_PLAN_ENV_INCLUDE_PLAN_ENV_GRID_MAP_H_
#define DIFF_PLANNER_PLAN_ENV_INCLUDE_PLAN_ENV_GRID_MAP_H_

#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <random>
#include <nav_msgs/Odometry.h>
#include <queue>
#include <ros/ros.h>
#include <tuple>
#include <visualization_msgs/Marker.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

#include <plan_env/raycast.h>

#define PLAN_ENV_LOGIT(x) (log((x) / (1 - (x))))
#define PLAN_ENV_GRID_MAP_OBSTACLE_FLAG 32767
#define PLAN_ENV_GRID_MAP_NEW_PLATFORM_TEST false

using namespace std;

// constant parameters

struct MappingParameters
{
  bool have_initialized = false;

  /* map properties */
  Eigen::Vector3d local_update_range3d;
  Eigen::Vector3i local_update_range3i;
  double resolution, resolution_inv;
  double obstacles_inflation;
  int inf_grid;
  string frame_id;
  int pose_type;
  bool enable_virtual_wall;
  double virtual_ceil, virtual_ground;
  double init_x, init_y, init_z;

  /* camera parameters */
  double cx, cy, fx, fy;

  /* time out */
  double odom_depth_timeout;

  /* depth image projection filtering */
  bool use_depth_filter;
  double depth_filter_min_distance, depth_filter_tolerance;
  int depth_filter_margin;
  double depth_scaling_factor;
  int skip_pixel;

  /* raycasting */
  double p_hit, p_miss, p_min, p_max, p_occ;                                           // occupancy probability (depth)
  double prob_hit_log, prob_miss_log, clamp_min_log, clamp_max_log, min_occupancy_log; // PLAN_ENV_LOGIT of occupancy probability (depth)
  double lidar_p_hit, lidar_p_miss, lidar_p_free, lidar_p_min, lidar_p_max, lidar_p_occ; // occupancy probability (cloud)
  double lidar_prob_hit_log, lidar_prob_miss_log, lidar_clamp_min_log,
      lidar_clamp_max_log, lidar_min_occupancy_log; // PLAN_ENV_LOGIT of occupancy probability (cloud)
  bool cloud_enable_raycast;
  double min_ray_length;                                                                   // range of doing raycasting
  double fading_time;

  /* visualization and computation time display */
  bool show_occ_time;
};

// intermediate mapping data for fusion

struct MappingData
{
  Eigen::Vector3i center_last3i;
  // Eigen::Vector3d ringbuffer_origin3d;
  Eigen::Vector3i ringbuffer_origin3i;
  // Eigen::Vector3d ringbuffer_division3d;
  // Eigen::Vector3i ringbuffer_division3i;
  Eigen::Vector3d ringbuffer_lowbound3d;
  Eigen::Vector3i ringbuffer_lowbound3i;
  Eigen::Vector3d ringbuffer_upbound3d;
  Eigen::Vector3i ringbuffer_upbound3i;
  // Eigen::Vector3d ringbuffer_size3d;
  Eigen::Vector3i ringbuffer_size3i;
  Eigen::Vector3i ringbuffer_inf_origin3i;
  Eigen::Vector3d ringbuffer_inf_lowbound3d;
  Eigen::Vector3i ringbuffer_inf_lowbound3i;
  Eigen::Vector3d ringbuffer_inf_upbound3d;
  Eigen::Vector3i ringbuffer_inf_upbound3i;
  Eigen::Vector3i ringbuffer_inf_size3i;

  // main map data, occupancy of each voxel

  std::vector<double> occupancy_buffer;
  std::vector<uint16_t> occupancy_buffer_inflate;

  // camera position and pose data

  Eigen::Vector3d camera_pos, last_camera_pos;
  Eigen::Matrix3d camera_r_m, last_camera_r_m;
  Eigen::Matrix4d camera_to_body;

  // depth image data

  cv::Mat depth_image, last_depth_image;

  // flags of map state

  bool occ_need_update, local_updated;
  bool has_first_depth;
  bool has_odom;
  bool use_lidar_prob_for_update;

  // odom_depth_timeout
  ros::Time last_occ_update_time;
  bool flag_depth_odom_timeout;
  bool flag_have_ever_received_depth;

  // depth image projected point cloud

  vector<Eigen::Vector3d> proj_points;
  int projected_point_count;

  // flag buffers for speeding up raycasting

  vector<short> count_hit, count_hit_and_miss;
  vector<char> flag_traverse, flag_rayend;
  char raycast_num;

  vector<Eigen::Vector3i> cache_voxel;
  int cache_voxel_count;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class GridMap
{
public:
  GridMap() {}
  ~GridMap() {}

  void InitMap(ros::NodeHandle &nh);
  inline int GetOccupancy(Eigen::Vector3d pos);
  inline int GetInflatedOccupancy(Eigen::Vector3d pos);
  inline double GetResolution();
  bool GetOdometryDepthTimeout() { return mapping_data_.flag_depth_odom_timeout; }

  typedef std::shared_ptr<GridMap> Ptr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  MappingParameters mapping_parameters_;
  MappingData mapping_data_;

  enum
  {
    kPoseStamped = 1,
    kOdometry = 2,
    kInvalidIndex = -10000
  };

  inline Eigen::Vector3d GlobalIndexToPosition(const Eigen::Vector3i &id);  // 1.69ns
  inline Eigen::Vector3i PositionToGlobalIndex(const Eigen::Vector3d &pos); // 0.13ns
  inline int GlobalIndexToBufferIndex(const Eigen::Vector3i &id);           // 2.2ns
  inline int GlobalIndexToInflatedBufferIndex(const Eigen::Vector3i &id);        // 2.2ns
  inline Eigen::Vector3i BufferIndexToGlobalIndex(size_t address);          // 10.18ns
  inline Eigen::Vector3i InflatedBufferIndexToGlobalIndex(size_t address);       // 10.18ns
  inline bool IsInBuffer(const Eigen::Vector3d &pos);
  inline bool IsInBuffer(const Eigen::Vector3i &idx);
  inline bool IsInInflatedBuffer(const Eigen::Vector3d &pos);
  inline bool IsInInflatedBuffer(const Eigen::Vector3i &idx);

  void PublishMap();
  void PublishInflatedMap();

  // get depth image and camera pose
  void DepthPoseCallback(const sensor_msgs::ImageConstPtr &img,
                         const geometry_msgs::PoseStampedConstPtr &pose);
  void ExtrinsicCallback(const nav_msgs::OdometryConstPtr &odom);
  void DepthOdometryCallback(const sensor_msgs::ImageConstPtr &img, const nav_msgs::OdometryConstPtr &odom);
  void PointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &img);
  void OdometryCallback(const nav_msgs::OdometryConstPtr &odom);

  // update occupancy by raycasting
  void UpdateOccupancyCallback(const ros::TimerEvent & /*event*/);
  void VisualizationCallback(const ros::TimerEvent & /*event*/);
  void FadingCallback(const ros::TimerEvent & /*event*/);

  void ClearBuffer(char case_index, int bound);

  // main update process
  void MoveRingBuffer();
  void ProjectDepthImage();
  void ProcessRaycast();
  void RaycastFromPointCloud();   // 专门为点云做的 raycast
  void ClearAndInflateLocalMap();

  inline void ChangeInflatedBuffer(const bool dir, const int inf_buf_idx, const Eigen::Vector3i global_idx);
  inline int SetCachedOccupancy(Eigen::Vector3d pos, int occ);
  Eigen::Vector3d ClosestPointInMap(const Eigen::Vector3d &pt, const Eigen::Vector3d &camera_pt);
  void TestIndexingCost();
  bool NeedsDepthOdometryUpdate();
  void InitializeMapBoundary();

  // typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image,
  // nav_msgs::Odometry> SyncPolicyImageOdom; typedef
  // message_filters::sync_policies::ExactTime<sensor_msgs::Image,
  // geometry_msgs::PoseStamped> SyncPolicyImagePose;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry>
      SyncPolicyImageOdom;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, geometry_msgs::PoseStamped>
      SyncPolicyImagePose;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImagePose>> SynchronizerImagePose;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImageOdom>> SynchronizerImageOdom;

  ros::NodeHandle node_;
  shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> depth_sub_;
  shared_ptr<message_filters::Subscriber<geometry_msgs::PoseStamped>> pose_sub_;
  shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub_;
  SynchronizerImagePose sync_image_pose_;
  SynchronizerImageOdom sync_image_odom_;

  ros::Subscriber indep_cloud_sub_, indep_odom_sub_, extrinsic_sub_;
  ros::Publisher map_pub_, map_inf_pub_;
  ros::Timer occ_timer_, vis_timer_, fading_timer_;

  //
  uniform_real_distribution<double> rand_noise_;
  normal_distribution<double> rand_noise2_;
  default_random_engine eng_;
};

/* ============================== definition of inline function
 * ============================== */

inline int GridMap::SetCachedOccupancy(Eigen::Vector3d pos, int occ)
{
  if (occ != 1 && occ != 0)
    return kInvalidIndex;

  Eigen::Vector3i id = PositionToGlobalIndex(pos);
  int idx_ctns = GlobalIndexToBufferIndex(id);

  mapping_data_.count_hit_and_miss[idx_ctns] += 1;

  if (mapping_data_.count_hit_and_miss[idx_ctns] == 1)
  {
    mapping_data_.cache_voxel[mapping_data_.cache_voxel_count++] = id;
  }

  if (occ == 1)
    mapping_data_.count_hit[idx_ctns] += 1;

  return idx_ctns;
}

inline void GridMap::ChangeInflatedBuffer(const bool dir, const int inf_buf_idx, const Eigen::Vector3i global_idx)
{
  int inf_grid = mapping_parameters_.inf_grid;
  if (dir)
    mapping_data_.occupancy_buffer_inflate[inf_buf_idx] += PLAN_ENV_GRID_MAP_OBSTACLE_FLAG;
  else
    mapping_data_.occupancy_buffer_inflate[inf_buf_idx] -= PLAN_ENV_GRID_MAP_OBSTACLE_FLAG;

  for (int x_inf = -inf_grid; x_inf <= inf_grid; ++x_inf)
    for (int y_inf = -inf_grid; y_inf <= inf_grid; ++y_inf)
      for (int z_inf = -inf_grid; z_inf <= inf_grid; ++z_inf)
      {
        Eigen::Vector3i id_inf(global_idx + Eigen::Vector3i(x_inf, y_inf, z_inf));
#if PLAN_ENV_GRID_MAP_NEW_PLATFORM_TEST
        if (IsInInflatedBuffer(id_inf))
        {
          int id_inf_buf = GlobalIndexToInflatedBufferIndex(id_inf);
          if (dir)
            ++mapping_data_.occupancy_buffer_inflate[id_inf_buf];
          else
          {
            --mapping_data_.occupancy_buffer_inflate[id_inf_buf];
            if (mapping_data_.occupancy_buffer_inflate[id_inf_buf] > 65000) // An error case
            {
              t1 = ros::Time::now();
              ROS_ERROR("A negtive value of nearby obstacle number! reset the map.");
              fill(mapping_data_.occupancy_buffer.begin(), mapping_data_.occupancy_buffer.end(), mapping_parameters_.clamp_min_log);
              fill(mapping_data_.occupancy_buffer_inflate.begin(), mapping_data_.occupancy_buffer_inflate.end(), 0L);
              t2 = ros::Time::now();
              ROS_WARN("reset the map time: t2-t1=%f", (t2 - t1).toSec());
            }
          }

          if (mapping_data_.occupancy_buffer_inflate[id_inf_buf] > 60000)
          {
            cout << "2 occ=" << mapping_data_.occupancy_buffer_inflate[id_inf_buf] << " id_inf_buf=" << id_inf_buf << " id_inf=" << id_inf.transpose() << " pos=" << GlobalIndexToPosition(id_inf).transpose() << endl;
          }
        }
        else
        {
          cout << "id_inf=" << id_inf.transpose() << " md_.ringbuffer_inf_upbound3i_=" << mapping_data_.ringbuffer_inf_upbound3i.transpose() << " md_.ringbuffer_upbound3i_=" << mapping_data_.ringbuffer_upbound3i.transpose() << endl;
          ROS_ERROR("isInInfBuf return false 1");
        }

#else
        int id_inf_buf = GlobalIndexToInflatedBufferIndex(id_inf);
        if (dir)
          ++mapping_data_.occupancy_buffer_inflate[id_inf_buf];
        else
        {
          --mapping_data_.occupancy_buffer_inflate[id_inf_buf];
          if (mapping_data_.occupancy_buffer_inflate[id_inf_buf] > 65000) // An error case
          {
            ros::Time t1, t2;
            t1 = ros::Time::now();
            ROS_ERROR("A negtive value of nearby obstacle number! reset the map.");
            fill(mapping_data_.occupancy_buffer.begin(), mapping_data_.occupancy_buffer.end(), mapping_parameters_.clamp_min_log);
            fill(mapping_data_.occupancy_buffer_inflate.begin(), mapping_data_.occupancy_buffer_inflate.end(), 0L);
            t2 = ros::Time::now();
            ROS_WARN("reset the map time: t2-t1=%f", (t2 - t1).toSec());
          }
        }
#endif  // DIFF_PLANNER_PLAN_ENV_INCLUDE_PLAN_ENV_GRID_MAP_H_
      }
}

inline int GridMap::GlobalIndexToBufferIndex(const Eigen::Vector3i &id)
{
  int x_buffer = (id(0) - mapping_data_.ringbuffer_origin3i(0)) % mapping_data_.ringbuffer_size3i(0);
  int y_buffer = (id(1) - mapping_data_.ringbuffer_origin3i(1)) % mapping_data_.ringbuffer_size3i(1);
  int z_buffer = (id(2) - mapping_data_.ringbuffer_origin3i(2)) % mapping_data_.ringbuffer_size3i(2);
  if (x_buffer < 0)
    x_buffer += mapping_data_.ringbuffer_size3i(0);
  if (y_buffer < 0)
    y_buffer += mapping_data_.ringbuffer_size3i(1);
  if (z_buffer < 0)
    z_buffer += mapping_data_.ringbuffer_size3i(2);

  return mapping_data_.ringbuffer_size3i(0) * mapping_data_.ringbuffer_size3i(1) * z_buffer + mapping_data_.ringbuffer_size3i(0) * y_buffer + x_buffer;
}

inline int GridMap::GlobalIndexToInflatedBufferIndex(const Eigen::Vector3i &id)
{
  int x_buffer = (id(0) - mapping_data_.ringbuffer_inf_origin3i(0)) % mapping_data_.ringbuffer_inf_size3i(0);
  int y_buffer = (id(1) - mapping_data_.ringbuffer_inf_origin3i(1)) % mapping_data_.ringbuffer_inf_size3i(1);
  int z_buffer = (id(2) - mapping_data_.ringbuffer_inf_origin3i(2)) % mapping_data_.ringbuffer_inf_size3i(2);
  if (x_buffer < 0)
    x_buffer += mapping_data_.ringbuffer_inf_size3i(0);
  if (y_buffer < 0)
    y_buffer += mapping_data_.ringbuffer_inf_size3i(1);
  if (z_buffer < 0)
    z_buffer += mapping_data_.ringbuffer_inf_size3i(2);

  return mapping_data_.ringbuffer_inf_size3i(0) * mapping_data_.ringbuffer_inf_size3i(1) * z_buffer + mapping_data_.ringbuffer_inf_size3i(0) * y_buffer + x_buffer;
}

inline Eigen::Vector3i GridMap::BufferIndexToGlobalIndex(size_t address)
{

  const int ringbuffer_xysize = mapping_data_.ringbuffer_size3i(0) * mapping_data_.ringbuffer_size3i(1);
  int zid_in_buffer = address / ringbuffer_xysize;
  address %= ringbuffer_xysize;
  int yid_in_buffer = address / mapping_data_.ringbuffer_size3i(0);
  int xid_in_buffer = address % mapping_data_.ringbuffer_size3i(0);

  int xid_global = xid_in_buffer + mapping_data_.ringbuffer_origin3i(0);
  if (xid_global > mapping_data_.ringbuffer_upbound3i(0))
    xid_global -= mapping_data_.ringbuffer_size3i(0);
  int yid_global = yid_in_buffer + mapping_data_.ringbuffer_origin3i(1);
  if (yid_global > mapping_data_.ringbuffer_upbound3i(1))
    yid_global -= mapping_data_.ringbuffer_size3i(1);
  int zid_global = zid_in_buffer + mapping_data_.ringbuffer_origin3i(2);
  if (zid_global > mapping_data_.ringbuffer_upbound3i(2))
    zid_global -= mapping_data_.ringbuffer_size3i(2);

  return Eigen::Vector3i(xid_global, yid_global, zid_global);
}

inline Eigen::Vector3i GridMap::InflatedBufferIndexToGlobalIndex(size_t address)
{

  const int ringbuffer_xysize = mapping_data_.ringbuffer_inf_size3i(0) * mapping_data_.ringbuffer_inf_size3i(1);
  int zid_in_buffer = address / ringbuffer_xysize;
  address %= ringbuffer_xysize;
  int yid_in_buffer = address / mapping_data_.ringbuffer_inf_size3i(0);
  int xid_in_buffer = address % mapping_data_.ringbuffer_inf_size3i(0);

  int xid_global = xid_in_buffer + mapping_data_.ringbuffer_inf_origin3i(0);
  if (xid_global > mapping_data_.ringbuffer_inf_upbound3i(0))
    xid_global -= mapping_data_.ringbuffer_inf_size3i(0);
  int yid_global = yid_in_buffer + mapping_data_.ringbuffer_inf_origin3i(1);
  if (yid_global > mapping_data_.ringbuffer_inf_upbound3i(1))
    yid_global -= mapping_data_.ringbuffer_inf_size3i(1);
  int zid_global = zid_in_buffer + mapping_data_.ringbuffer_inf_origin3i(2);
  if (zid_global > mapping_data_.ringbuffer_inf_upbound3i(2))
    zid_global -= mapping_data_.ringbuffer_inf_size3i(2);

  return Eigen::Vector3i(xid_global, yid_global, zid_global);
}

inline int GridMap::GetOccupancy(Eigen::Vector3d pos)
{
  if (mapping_parameters_.enable_virtual_wall && (pos(2) >= mapping_parameters_.virtual_ceil || pos(2) <= mapping_parameters_.virtual_ground))
    return -1;

  if (!IsInBuffer(pos))
    return 0;

  return mapping_data_.occupancy_buffer[GlobalIndexToBufferIndex(PositionToGlobalIndex(pos))] > mapping_parameters_.min_occupancy_log ? 1 : 0;
}

inline int GridMap::GetInflatedOccupancy(Eigen::Vector3d pos)
{
  if (mapping_parameters_.enable_virtual_wall && (pos(2) >= mapping_parameters_.virtual_ceil || pos(2) <= mapping_parameters_.virtual_ground))
    return -1;
  
  if (!IsInInflatedBuffer(pos))
    return 0;

  return int(mapping_data_.occupancy_buffer_inflate[GlobalIndexToInflatedBufferIndex(PositionToGlobalIndex(pos))]);
}

inline bool GridMap::IsInBuffer(const Eigen::Vector3d &pos)
{
  if (pos(0) < mapping_data_.ringbuffer_lowbound3d(0) || pos(1) < mapping_data_.ringbuffer_lowbound3d(1) || pos(2) < mapping_data_.ringbuffer_lowbound3d(2))
  {
    return false;
  }
  if (pos(0) > mapping_data_.ringbuffer_upbound3d(0) || pos(1) > mapping_data_.ringbuffer_upbound3d(1) || pos(2) > mapping_data_.ringbuffer_upbound3d(2))
  {
    return false;
  }
  return true;
}

inline bool GridMap::IsInBuffer(const Eigen::Vector3i &idx)
{
  if (idx(0) < mapping_data_.ringbuffer_lowbound3i(0) || idx(1) < mapping_data_.ringbuffer_lowbound3i(1) || idx(2) < mapping_data_.ringbuffer_lowbound3i(2))
  {
    return false;
  }
  if (idx(0) > mapping_data_.ringbuffer_upbound3i(0) || idx(1) > mapping_data_.ringbuffer_upbound3i(1) || idx(2) > mapping_data_.ringbuffer_upbound3i(2))
  {
    return false;
  }
  return true;
}

inline bool GridMap::IsInInflatedBuffer(const Eigen::Vector3d &pos)
{
  if (pos(0) < mapping_data_.ringbuffer_inf_lowbound3d(0) || pos(1) < mapping_data_.ringbuffer_inf_lowbound3d(1) || pos(2) < mapping_data_.ringbuffer_inf_lowbound3d(2))
  {
    return false;
  }
  if (pos(0) > mapping_data_.ringbuffer_inf_upbound3d(0) || pos(1) > mapping_data_.ringbuffer_inf_upbound3d(1) || pos(2) > mapping_data_.ringbuffer_inf_upbound3d(2))
  {
    return false;
  }
  return true;
}

inline bool GridMap::IsInInflatedBuffer(const Eigen::Vector3i &idx)
{
  if (idx(0) < mapping_data_.ringbuffer_inf_lowbound3i(0) || idx(1) < mapping_data_.ringbuffer_inf_lowbound3i(1) || idx(2) < mapping_data_.ringbuffer_inf_lowbound3i(2))
  {
    return false;
  }
  if (idx(0) > mapping_data_.ringbuffer_inf_upbound3i(0) || idx(1) > mapping_data_.ringbuffer_inf_upbound3i(1) || idx(2) > mapping_data_.ringbuffer_inf_upbound3i(2))
  {
    return false;
  }
  return true;
}

inline Eigen::Vector3d GridMap::GlobalIndexToPosition(const Eigen::Vector3i &id) // t ~ 0us
{
  return Eigen::Vector3d((id(0) + 0.5) * mapping_parameters_.resolution, (id(1) + 0.5) * mapping_parameters_.resolution, (id(2) + 0.5) * mapping_parameters_.resolution);
}

inline Eigen::Vector3i GridMap::PositionToGlobalIndex(const Eigen::Vector3d &pos)
{
  return (pos * mapping_parameters_.resolution_inv).array().floor().cast<int>(); // more than twice faster than std::floor()
}

inline double GridMap::GetResolution() { return mapping_parameters_.resolution; }

#endif
