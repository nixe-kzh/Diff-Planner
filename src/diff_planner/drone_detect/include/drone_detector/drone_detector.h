#ifndef DIFF_PLANNER_DRONE_DETECT_INCLUDE_DRONE_DETECTOR_DRONE_DETECTOR_H_
#define DIFF_PLANNER_DRONE_DETECT_INCLUDE_DRONE_DETECTOR_DRONE_DETECTOR_H_

#include <iostream>
#include <vector>
// ROS
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
// synchronize topic
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_srvs/Trigger.h>

//include opencv and eigen
#include <Eigen/Eigen>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <cv_bridge/cv_bridge.h>

namespace detect {

constexpr int kMaxDroneCount = 3;

/*!
 * Main class for the node to handle the ROS interfacing.
 */
class DroneDetector
{
 public:
  /*!
   * Constructor.
   * @param node_handle the ROS node handle.
   */
  DroneDetector(ros::NodeHandle& node_handle);

  /*!
   * Destructor.
   */
  virtual ~DroneDetector();

  void Test();
 private:
  void ReadParameters();

  // inline functions
  double SquaredDistance(const Eigen::Vector3d &p1,
                         const Eigen::Vector3d &p2);
  double SquaredDistance(const Eigen::Vector4d &p1,
                         const Eigen::Vector4d &p2);
  Eigen::Vector4d DepthToPosition(int u, int v, float depth);
  Eigen::Vector4d DepthToPosition(const Eigen::Vector2i &pixel, float depth);
  Eigen::Vector2i PositionToDepthPixel(
      const Eigen::Vector4d &position_in_camera);
  bool IsInSensorRange(const Eigen::Vector2i &pixel);

  bool CountPixels(int drone_id, Eigen::Vector2i &detected_pixel,
                   Eigen::Vector4d &detected_position_camera);
  void Detect(int drone_id, Eigen::Vector2i &detected_pixel);
  
  // subscribe callback function
  void DepthColorCameraPoseCallback(
      const sensor_msgs::ImageConstPtr& depth_image,
      const sensor_msgs::ImageConstPtr& color_image,
      const geometry_msgs::PoseStampedConstPtr& camera_pose);

  void DepthCameraPoseCallback(
      const sensor_msgs::ImageConstPtr& depth_image,
      const geometry_msgs::PoseStampedConstPtr& camera_pose);
  
  void OdometryCallback(const nav_msgs::Odometry& odometry);
  void DepthImageCallback(const sensor_msgs::ImageConstPtr& depth_image);

  void DroneOdometryCallback(const nav_msgs::Odometry& odometry, int drone_id);

  void Drone0OdometryCallback(const nav_msgs::Odometry& odometry);
  void Drone1OdometryCallback(const nav_msgs::Odometry& odometry);
  void Drone2OdometryCallback(const nav_msgs::Odometry& odometry);
  void OtherDroneOdometryCallback(const nav_msgs::Odometry& odometry);
  
  //! ROS node handle.
  ros::NodeHandle& nh_;

  //! ROS topic subscriber.
  // depth, colordepth, camera_pos subscriber
  using SyncPolicyDepthColorImagePose =
      message_filters::sync_policies::ApproximateTime<
          sensor_msgs::Image, sensor_msgs::Image,
          geometry_msgs::PoseStamped>;
  using SynchronizerDepthColorImagePose =
      std::shared_ptr<message_filters::Synchronizer<
          SyncPolicyDepthColorImagePose>>;
  using SyncPolicyDepthImagePose =
      message_filters::sync_policies::ApproximateTime<
          sensor_msgs::Image, geometry_msgs::PoseStamped>;
  using SynchronizerDepthImagePose =
      std::shared_ptr<message_filters::Synchronizer<SyncPolicyDepthImagePose>>;
  
  // std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> depth_img_sub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>>
      color_depth_image_sub_;
  std::shared_ptr<message_filters::Subscriber<geometry_msgs::PoseStamped>>
      camera_pose_sub_;

  SynchronizerDepthColorImagePose depth_color_image_pose_synchronizer_;
  SynchronizerDepthImagePose depth_image_pose_synchronizer_;
  // other drones subscriber
  ros::Subscriber drone0_odometry_sub_;
  ros::Subscriber drone1_odometry_sub_;
  ros::Subscriber drone2_odometry_sub_;
  ros::Subscriber other_drone_odometry_sub_;
  std::string drone1_odometry_topic_;
  std::string drone2_odometry_topic_;

  ros::Subscriber odometry_sub_;
  ros::Subscriber depth_image_sub_;
  bool has_odometry_;
  nav_msgs::Odometry odometry_;
  // ROS topic publisher
  // new_depth_img: erase the detected drones
  // new_colordepth_img: for debug
  ros::Publisher new_depth_image_pub_;
  ros::Publisher debug_depth_image_pub_;

  // parameters
  //camera param
  int image_width_;
  int image_height_;
  double fx_;
  double fy_;
  double cx_;
  double cy_;

  double max_pose_error_;
  double max_squared_pose_error_;
  double drone_width_;
  double drone_height_;
  double pixel_ratio_;
  int pixel_threshold_;

  // for debug
  bool debug_flag_;
  int debug_detection_result_[kMaxDroneCount];
  std::stringstream debug_image_text_[kMaxDroneCount];
  ros::Time debug_start_time_;
  ros::Time debug_end_time_;

  ros::Publisher debug_info_pub_;
  ros::Publisher drone_pose_error_pub_[kMaxDroneCount];

  int my_id_;
  cv::Mat depth_image_;
  cv::Mat color_image_;

  Eigen::Matrix4d camera_to_body_;
  Eigen::Matrix4d camera_to_world_;
  Eigen::Quaterniond camera_to_world_quaternion_;
  Eigen::Vector4d my_pose_world_;
  Eigen::Quaterniond my_attitude_world_;
  Eigen::Vector4d my_last_pose_world_;
  ros::Time my_last_odom_stamp_ = ros::TIME_MAX;
  ros::Time my_last_camera_stamp_ = ros::TIME_MAX;

  Eigen::Matrix4d drone_to_world_[kMaxDroneCount];
  Eigen::Vector4d drone_pose_world_[kMaxDroneCount];
  Eigen::Quaterniond drone_attitude_world_[kMaxDroneCount];
  Eigen::Vector4d drone_pose_camera_[kMaxDroneCount];
  Eigen::Vector2i drone_reference_pixel_[kMaxDroneCount];

  std::vector<Eigen::Vector2i> hit_pixels_[kMaxDroneCount];
  int valid_pixel_count_[kMaxDroneCount];
  
  bool in_depth_[kMaxDroneCount] = {false};
  cv::Point search_box_upper_left_[kMaxDroneCount];
  cv::Point search_box_lower_right_[kMaxDroneCount];
  cv::Point bounding_box_upper_left_[kMaxDroneCount];
  cv::Point bounding_box_lower_right_[kMaxDroneCount];
};

}  // namespace detect

#endif  // DIFF_PLANNER_DRONE_DETECT_INCLUDE_DRONE_DETECTOR_DRONE_DETECTOR_H_
