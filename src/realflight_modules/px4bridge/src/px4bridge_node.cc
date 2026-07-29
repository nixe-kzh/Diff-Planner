#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <cmath>
#include <cstdint>
#include <exception>
#include <stdexcept>

namespace px4bridge {
namespace {

constexpr double kDefaultVisionPoseRateHz = 50.0;
constexpr double kDefaultOdometryTimeoutSec = 0.2;
constexpr double kMinimumQuaternionNormSquared = 1e-12;

}  // namespace

// 将 LIO 的 nav_msgs/Odometry 限频转换为 MAVROS 接收的 PoseStamped。

class Px4Bridge {
 public:
  Px4Bridge() : private_node_handle_("~") {
    LoadParameters();

    // launch 默认将私有话题 ~vision_pose 重映射到/mavros/vision_pose/pose。
    vision_pose_publisher_ =
        private_node_handle_.advertise<geometry_msgs::PoseStamped>("vision_pose", 10);

    odometry_subscriber_ = 
        private_node_handle_.subscribe("odom", 1, &Px4Bridge::OdometryCallback,
                                       this, ros::TransportHints().tcpNoDelay());

    // 定时器定义最大发送频率；只有收到新的 odometry 才会发布。
    publish_timer_ = 
        private_node_handle_.createTimer(ros::Duration(1.0 / vision_pose_rate_hz_),
                                         &Px4Bridge::PublishVisionPose, this);

    ROS_INFO_STREAM("PX4 bridge started: odom=" << odometry_subscriber_.getTopic()
                                                << ", vision_pose="
                                                << vision_pose_publisher_.getTopic()
                                                << ", rate=" << vision_pose_rate_hz_ << " Hz"
                                                << ", timeout=" << odometry_timeout_sec_ << " s");
  }

 private:
  void LoadParameters() {
    private_node_handle_.param("vision_pose_rate_hz", vision_pose_rate_hz_,
                               kDefaultVisionPoseRateHz);
    private_node_handle_.param("odometry_timeout_sec", odometry_timeout_sec_,
                               kDefaultOdometryTimeoutSec);

    if (!std::isfinite(vision_pose_rate_hz_) || vision_pose_rate_hz_ <= 0.0) {
      throw std::invalid_argument(
          "Parameter '~vision_pose_rate_hz' must be finite and greater than "
          "zero.");
    }
    if (!std::isfinite(odometry_timeout_sec_) || odometry_timeout_sec_ <= 0.0) {
      throw std::invalid_argument(
          "Parameter '~odometry_timeout_sec' must be finite and greater than "
          "zero.");
    }
  }

  void OdometryCallback(const nav_msgs::Odometry::ConstPtr& odometry) {
    // 输入应表示 ROS 约定下的机体位姿
    // 世界系为 ENU，机体系为 FLU
    // 若 LIO 输出雷达位姿，必须先应用雷达到机体的外参
    if (!IsPoseFinite(odometry->pose.pose)) {
      ROS_WARN_THROTTLE(1.0, "Discarding LIO odometry with non-finite pose data.");
      return;
    }

    const geometry_msgs::Quaternion& orientation = odometry->pose.pose.orientation;
    const double quaternion_norm_squared =
        orientation.x * orientation.x + orientation.y * orientation.y +
        orientation.z * orientation.z + orientation.w * orientation.w;
    if (quaternion_norm_squared < kMinimumQuaternionNormSquared) {
      ROS_WARN_THROTTLE(1.0, "Discarding LIO odometry with an invalid orientation.");
      return;
    }

    latest_vision_pose_.header = odometry->header;
    latest_vision_pose_.pose = odometry->pose.pose;

    // PX4 会校验姿态四元数，这里先归一化以消除数值积分产生的小误差。
    const double inverse_quaternion_norm = 1.0 / std::sqrt(quaternion_norm_squared);
    latest_vision_pose_.pose.orientation.x *= inverse_quaternion_norm;
    latest_vision_pose_.pose.orientation.y *= inverse_quaternion_norm;
    latest_vision_pose_.pose.orientation.z *= inverse_quaternion_norm;
    latest_vision_pose_.pose.orientation.w *= inverse_quaternion_norm;

    latest_odometry_receive_time_ = ros::Time::now();

    // 优先保留 LIO 的测量时间戳，MAVROS 会用它生成 MAVLink usec，并丢弃与上一帧时间戳完全相同的消息
    if (latest_vision_pose_.header.stamp.isZero()) {
      latest_vision_pose_.header.stamp = latest_odometry_receive_time_;
    }
    ++latest_odometry_sequence_;

  }

  void PublishVisionPose(const ros::TimerEvent&) {
    if (latest_odometry_sequence_ == 0) {
      ROS_WARN_THROTTLE(5.0, "Waiting for LIO odometry.");
      return;
    }

    const double odometry_age_sec = (ros::Time::now() - latest_odometry_receive_time_).toSec();
    if (odometry_age_sec > odometry_timeout_sec_) {
      ROS_WARN_THROTTLE(1.0, "LIO odometry timed out (age: %.3f s); vision pose paused.",
                        odometry_age_sec);
      return;
    }

    // 不重复发布同一帧，避免 MAVROS 因时间戳相同再次丢弃消息
    if (latest_odometry_sequence_ == published_odometry_sequence_) {
      return;
    }

    vision_pose_publisher_.publish(latest_vision_pose_);
    published_odometry_sequence_ = latest_odometry_sequence_;
  }

  static bool IsPoseFinite(const geometry_msgs::Pose& pose) {
    return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
           std::isfinite(pose.position.z) && std::isfinite(pose.orientation.x) &&
           std::isfinite(pose.orientation.y) && std::isfinite(pose.orientation.z) &&
           std::isfinite(pose.orientation.w);
  }

  ros::NodeHandle private_node_handle_;
  ros::Publisher vision_pose_publisher_;
  ros::Subscriber odometry_subscriber_;
  ros::Timer publish_timer_;

  geometry_msgs::PoseStamped latest_vision_pose_;
  ros::Time latest_odometry_receive_time_;
  std::uint64_t latest_odometry_sequence_ = 0;
  std::uint64_t published_odometry_sequence_ = 0;

  double vision_pose_rate_hz_ = kDefaultVisionPoseRateHz;
  double odometry_timeout_sec_ = kDefaultOdometryTimeoutSec;
};

}  // namespace px4bridge

int main(int argc, char** argv) {
  ros::init(argc, argv, "px4bridge");

  try {
    px4bridge::Px4Bridge bridge;
    ros::spin();
  } catch (const std::exception& exception) {
    ROS_FATAL_STREAM("Failed to start PX4 bridge: " << exception.what());
    return 1;
  }

  return 0;
}
