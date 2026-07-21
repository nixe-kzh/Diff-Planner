#ifndef DIFF_PLANNER_PLAN_ENV_INCLUDE_PLAN_ENV_GRID_MAP_BIGMAP_H_
#define DIFF_PLANNER_PLAN_ENV_INCLUDE_PLAN_ENV_GRID_MAP_BIGMAP_H_

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

#define PLAN_ENV_BIG_MAP_LOGIT(x) (log((x) / (1 - (x))))

using namespace std;

// voxel hashing
template <typename T>
struct MatrixHash : std::unary_function<T, size_t>
{
  std::size_t operator()(const T &matrix) const
  {
    size_t seed = 0;
    for (size_t i = 0; i < matrix.size(); ++i)
    {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};

// constant parameters

struct MappingParameters
{

  /* map properties */
  Eigen::Vector3d map_origin, map_size;
  Eigen::Vector3d map_min_boundary, map_max_boundary; // map range in pos
  Eigen::Vector3i map_voxel_num;                       // map range in index
  Eigen::Vector3d local_update_range;
  double resolution, resolution_inv;
  double obstacles_inflation;
  string frame_id;
  int pose_type;

  /* camera parameters */
  double cx, cy, fx, fy;

  /* time out */
  double odom_depth_timeout;

  /* depth image projection filtering */
  double depth_filter_maxdist, depth_filter_mindist, depth_filter_tolerance;
  int depth_filter_margin;
  bool use_depth_filter;
  double depth_scaling_factor;
  int skip_pixel;

  /* raycasting */
  double p_hit, p_miss, p_min, p_max, p_occ; // occupancy probability
  double prob_hit_log, prob_miss_log, clamp_min_log, clamp_max_log,
      min_occupancy_log;                  // PLAN_ENV_BIG_MAP_LOGIT of occupancy probability
  double min_ray_length, max_ray_length; // range of doing raycasting
  double fading_time;

  /* local map update and clear */
  int local_map_margin;

  /* visualization and computation time display */
  double visualization_truncate_height, ground_height;
  bool show_occ_time;

  /* active mapping */
  double unknown_flag;
};

// intermediate mapping data for fusion

struct MappingData
{
  // main map data, occupancy of each voxel and Euclidean distance

  std::vector<double> occupancy_buffer;
  std::vector<char> occupancy_buffer_inflate;

  // camera position and pose data

  Eigen::Vector3d camera_pos, last_camera_pos;
  Eigen::Matrix3d camera_r_m, last_camera_r_m;
  Eigen::Matrix4d camera_to_body;

  // depth image data

  cv::Mat depth_image, last_depth_image;
  int image_count;

  // flags of map state

  bool occ_need_update, local_updated;
  bool has_first_depth;
  bool has_odom, has_cloud;

  // odom_depth_timeout
  ros::Time last_occ_update_time;
  bool flag_depth_odom_timeout;
  bool flag_use_depth_fusion;

  // depth image projected point cloud

  vector<Eigen::Vector3d> projected_points;
  int projected_point_count;

  // flag buffers for speeding up raycasting

  vector<short> count_hit, count_hit_and_miss;
  vector<char> flag_traverse, flag_rayend;
  char raycast_num;
  queue<Eigen::Vector3i> cache_voxel;

  // range of updating grid

  Eigen::Vector3i local_bound_min, local_bound_max;

  // computation time

  double fuse_time, max_fuse_time;
  int update_num;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class GridMap
{
public:
  GridMap() {}
  ~GridMap() {}

  enum
  {
    kPoseStamped = 1,
    kOdometry = 2,
    kInvalidIndex = -10000
  };

  // occupancy map management
  void ResetBuffer();
  void ResetBuffer(Eigen::Vector3d min, Eigen::Vector3d max);

  inline void PositionToIndex(const Eigen::Vector3d &pos, Eigen::Vector3i &id);
  inline void IndexToPosition(const Eigen::Vector3i &id, Eigen::Vector3d &pos);
  inline int ToAddress(const Eigen::Vector3i &id);
  inline int ToAddress(int &x, int &y, int &z);
  inline bool IsInMap(const Eigen::Vector3d &pos);
  inline bool IsInMap(const Eigen::Vector3i &idx);

  inline void SetOccupancy(Eigen::Vector3d pos, double occ = 1);
  inline void SetOccupied(Eigen::Vector3d pos);
  inline int GetOccupancy(Eigen::Vector3d pos);
  inline int GetOccupancy(Eigen::Vector3i id);
  inline int GetInflatedOccupancy(Eigen::Vector3d pos);
  inline int GetLessInflatedOccupancy(Eigen::Vector3d pos);

  inline void BoundIndex(Eigen::Vector3i &id);
  inline bool IsUnknown(const Eigen::Vector3i &id);
  inline bool IsUnknown(const Eigen::Vector3d &pos);
  inline bool IsKnownFree(const Eigen::Vector3i &id);
  inline bool IsKnownOccupied(const Eigen::Vector3i &id);

  void InitMap(ros::NodeHandle &nh);

  void PublishMap();
  void PublishInflatedMap(bool all_info = false);

  void PublishDepth();

  bool HasDepthObservation();
  bool IsOdometryValid();
  void GetRegion(Eigen::Vector3d &ori, Eigen::Vector3d &size);
  inline double GetResolution();
  Eigen::Vector3d GetOrigin();
  int GetVoxelCount();
  bool GetOdometryDepthTimeout() { return mapping_data_.flag_depth_odom_timeout; }

  using Ptr = std::shared_ptr<GridMap>;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  MappingParameters mapping_parameters_;
  MappingData mapping_data_;

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

  // main update process
  void ProjectDepthImage();
  void ProcessRaycast();
  void ClearAndInflateLocalMap();

  inline void InflatePoint(const Eigen::Vector3i &pt, int step, vector<Eigen::Vector3i> &pts);
  int SetCachedOccupancy(Eigen::Vector3d pos, int occ);
  Eigen::Vector3d ClosestPointInMap(const Eigen::Vector3d &pt, const Eigen::Vector3d &camera_pt);

  // typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image,
  // nav_msgs::Odometry> SyncPolicyImageOdom; typedef
  // message_filters::sync_policies::ExactTime<sensor_msgs::Image,
  // geometry_msgs::PoseStamped> SyncPolicyImagePose;
  using SyncPolicyImageOdom =
      message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                       nav_msgs::Odometry>;
  using SyncPolicyImagePose =
      message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                       geometry_msgs::PoseStamped>;
  using SynchronizerImagePose =
      std::shared_ptr<message_filters::Synchronizer<SyncPolicyImagePose>>;
  using SynchronizerImageOdom =
      std::shared_ptr<message_filters::Synchronizer<SyncPolicyImageOdom>>;

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

inline int GridMap::ToAddress(const Eigen::Vector3i &id)
{
  return id(0) * mapping_parameters_.map_voxel_num(1) * mapping_parameters_.map_voxel_num(2) + id(1) * mapping_parameters_.map_voxel_num(2) + id(2);
}

inline int GridMap::ToAddress(int &x, int &y, int &z)
{
  return x * mapping_parameters_.map_voxel_num(1) * mapping_parameters_.map_voxel_num(2) + y * mapping_parameters_.map_voxel_num(2) + z;
}

inline void GridMap::BoundIndex(Eigen::Vector3i &id)
{
  Eigen::Vector3i id1;
  id1(0) = max(min(id(0), mapping_parameters_.map_voxel_num(0) - 1), 0);
  id1(1) = max(min(id(1), mapping_parameters_.map_voxel_num(1) - 1), 0);
  id1(2) = max(min(id(2), mapping_parameters_.map_voxel_num(2) - 1), 0);
  id = id1;
}

inline bool GridMap::IsUnknown(const Eigen::Vector3i &id)
{
  Eigen::Vector3i id1 = id;
  BoundIndex(id1);
  return mapping_data_.occupancy_buffer[ToAddress(id1)] < mapping_parameters_.clamp_min_log - 1e-3;
}

inline bool GridMap::IsUnknown(const Eigen::Vector3d &pos)
{
  Eigen::Vector3i idc;
  PositionToIndex(pos, idc);
  return IsUnknown(idc);
}

inline bool GridMap::IsKnownFree(const Eigen::Vector3i &id)
{
  Eigen::Vector3i id1 = id;
  BoundIndex(id1);
  int adr = ToAddress(id1);

  // return mapping_data_.occupancy_buffer[adr] >= mapping_parameters_.clamp_min_log &&
  //     mapping_data_.occupancy_buffer[adr] < mapping_parameters_.min_occupancy_log;
  return mapping_data_.occupancy_buffer[adr] >= mapping_parameters_.clamp_min_log && mapping_data_.occupancy_buffer_inflate[adr] == 0;
}

inline bool GridMap::IsKnownOccupied(const Eigen::Vector3i &id)
{
  Eigen::Vector3i id1 = id;
  BoundIndex(id1);
  int adr = ToAddress(id1);

  return mapping_data_.occupancy_buffer_inflate[adr] == 1;
}

inline void GridMap::SetOccupied(Eigen::Vector3d pos)
{
  if (!IsInMap(pos))
    return;

  Eigen::Vector3i id;
  PositionToIndex(pos, id);

  mapping_data_.occupancy_buffer_inflate[id(0) * mapping_parameters_.map_voxel_num(1) * mapping_parameters_.map_voxel_num(2) +
                                id(1) * mapping_parameters_.map_voxel_num(2) + id(2)] = 1;
}

inline void GridMap::SetOccupancy(Eigen::Vector3d pos, double occ)
{
  if (occ != 1 && occ != 0)
  {
    cout << "occ value error!" << endl;
    return;
  }

  if (!IsInMap(pos))
    return;

  Eigen::Vector3i id;
  PositionToIndex(pos, id);

  mapping_data_.occupancy_buffer[ToAddress(id)] = occ;
}

inline int GridMap::GetOccupancy(Eigen::Vector3d pos)
{
  if (!IsInMap(pos))
    return -1;

  Eigen::Vector3i id;
  PositionToIndex(pos, id);

  return mapping_data_.occupancy_buffer[ToAddress(id)] > mapping_parameters_.min_occupancy_log ? 1 : 0;
}

inline int GridMap::GetInflatedOccupancy(Eigen::Vector3d pos)
{
  if (!IsInMap(pos))
    return -1;

  Eigen::Vector3i id;
  PositionToIndex(pos, id);

  return int(mapping_data_.occupancy_buffer_inflate[ToAddress(id)]);
}

inline int GridMap::GetLessInflatedOccupancy(Eigen::Vector3d pos)
{
  const double res = mapping_parameters_.resolution;
  for ( double x = -res; x < res + 1e-5; x+= res )
    for ( double y = -res; y < res + 1e-5; y+= res )
      for ( double z = -res; z < res + 1e-5; z+= res )
      {
        if ( ! GetInflatedOccupancy(pos += Eigen::Vector3d(x, y, z)) )
        {
          return false;
        }
      }

  return true;
}

inline int GridMap::GetOccupancy(Eigen::Vector3i id)
{
  if (id(0) < 0 || id(0) >= mapping_parameters_.map_voxel_num(0) || id(1) < 0 || id(1) >= mapping_parameters_.map_voxel_num(1) ||
      id(2) < 0 || id(2) >= mapping_parameters_.map_voxel_num(2))
    return -1;

  return mapping_data_.occupancy_buffer[ToAddress(id)] > mapping_parameters_.min_occupancy_log ? 1 : 0;
}

inline bool GridMap::IsInMap(const Eigen::Vector3d &pos)
{
  if (pos(0) < mapping_parameters_.map_min_boundary(0) + 1e-4 || pos(1) < mapping_parameters_.map_min_boundary(1) + 1e-4 ||
      pos(2) < mapping_parameters_.map_min_boundary(2) + 1e-4)
  {
    // cout << "less than min range!" << endl;
    return false;
  }
  if (pos(0) > mapping_parameters_.map_max_boundary(0) - 1e-4 || pos(1) > mapping_parameters_.map_max_boundary(1) - 1e-4 ||
      pos(2) > mapping_parameters_.map_max_boundary(2) - 1e-4)
  {
    return false;
  }
  return true;
}

inline bool GridMap::IsInMap(const Eigen::Vector3i &idx)
{
  if (idx(0) < 0 || idx(1) < 0 || idx(2) < 0)
  {
    return false;
  }
  if (idx(0) > mapping_parameters_.map_voxel_num(0) - 1 || idx(1) > mapping_parameters_.map_voxel_num(1) - 1 ||
      idx(2) > mapping_parameters_.map_voxel_num(2) - 1)
  {
    return false;
  }
  return true;
}

inline void GridMap::PositionToIndex(const Eigen::Vector3d &pos, Eigen::Vector3i &id)
{
  for (int i = 0; i < 3; ++i)
    id(i) = floor((pos(i) - mapping_parameters_.map_origin(i)) * mapping_parameters_.resolution_inv);
}

inline void GridMap::IndexToPosition(const Eigen::Vector3i &id, Eigen::Vector3d &pos)
{
  for (int i = 0; i < 3; ++i)
    pos(i) = (id(i) + 0.5) * mapping_parameters_.resolution + mapping_parameters_.map_origin(i);
}

inline void GridMap::InflatePoint(const Eigen::Vector3i &pt, int step, vector<Eigen::Vector3i> &pts)
{
  int num = 0;

  /* ---------- + shape inflate ---------- */
  // for (int x = -step; x <= step; ++x)
  // {
  //   if (x == 0)
  //     continue;
  //   pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1), pt(2));
  // }
  // for (int y = -step; y <= step; ++y)
  // {
  //   if (y == 0)
  //     continue;
  //   pts[num++] = Eigen::Vector3i(pt(0), pt(1) + y, pt(2));
  // }
  // for (int z = -1; z <= 1; ++z)
  // {
  //   pts[num++] = Eigen::Vector3i(pt(0), pt(1), pt(2) + z);
  // }

  /* ---------- all inflate ---------- */
  for (int x = -step; x <= step; ++x)
    for (int y = -step; y <= step; ++y)
      for (int z = -step; z <= step; ++z)
      {
        pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1) + y, pt(2) + z);
      }
}

inline double GridMap::GetResolution() { return mapping_parameters_.resolution; }

#endif  // DIFF_PLANNER_PLAN_ENV_INCLUDE_PLAN_ENV_GRID_MAP_BIGMAP_H_
