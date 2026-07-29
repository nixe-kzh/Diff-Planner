#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <cmath>
#include <cstdint>
#include <exception>
#include <mutex>
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

    geometry_msgs::PoseStamped vision_pose;
    vision_pose.header = odometry->header;
    vision_pose.pose = odometry->pose.pose;

    // PX4 会校验姿态四元数，这里先归一化以消除数值积分产生的小误差。
    const double inverse_quaternion_norm = 1.0 / std::sqrt(quaternion_norm_squared);
    vision_pose.pose.orientation.x *= inverse_quaternion_norm;
    vision_pose.pose.orientation.y *= inverse_quaternion_norm;
    vision_pose.pose.orientation.z *= inverse_quaternion_norm;
    vision_pose.pose.orientation.w *= inverse_quaternion_norm;

    // 优先保留 LIO 的测量时间戳。时间戳为零时才使用本机 ROS 时间。
    if (vision_pose.header.stamp.isZero()) {
      vision_pose.header.stamp = ros::Time::now();
    }

    const ros::SteadyTime receive_time = ros::SteadyTime::now();
    std::lock_guard<std::mutex> lock(state_mutex_);

    // MAVROS 会丢弃重复时间戳；主动检查可以直接暴露上游重复发布旧帧的问题。
    if (latest_odometry_sequence_ != 0 &&
        vision_pose.header.stamp <= latest_vision_pose_.header.stamp) {
      ROS_WARN_THROTTLE(
          1.0,
          "Discarding LIO odometry with a repeated or decreasing timestamp.");
      return;
    }

    latest_vision_pose_ = vision_pose;
    latest_odometry_receive_time_ = receive_time;
    ++latest_odometry_sequence_;
  }

  void PublishVisionPose(const ros::TimerEvent&) {
    geometry_msgs::PoseStamped vision_pose;

    {
      std::lock_guard<std::mutex> lock(state_mutex_);

      if (latest_odometry_sequence_ == 0) {
        ROS_WARN_THROTTLE(5.0, "Waiting for LIO odometry.");
        return;
      }

      // 单调时钟不受 NTP、系统校时和 ROS 仿真时间跳变影响。
      const double odometry_age_sec =
          (ros::SteadyTime::now() - latest_odometry_receive_time_).toSec();
      if (odometry_age_sec > odometry_timeout_sec_) {
        ROS_WARN_THROTTLE(
            1.0,
            "LIO odometry timed out (age: %.3f s); vision pose paused.",
            odometry_age_sec);
        return;
      }

      // 不重复发布同一帧，避免 MAVROS 因时间戳相同再次丢弃消息。
      if (latest_odometry_sequence_ == published_odometry_sequence_) {
        return;
      }

      vision_pose = latest_vision_pose_;
      published_odometry_sequence_ = latest_odometry_sequence_;
    }

    // 在锁外发布，避免 ROS 通信操作阻塞 odometry 回调。
    vision_pose_publisher_.publish(vision_pose);
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

  std::mutex state_mutex_;
  geometry_msgs::PoseStamped latest_vision_pose_;
  ros::SteadyTime latest_odometry_receive_time_;
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
    // odometry 与发布定时器分线程执行，避免高负载时互相阻塞回调。
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();
  } catch (const std::exception& exception) {
    ROS_FATAL_STREAM("Failed to start PX4 bridge: " << exception.what());
    return 1;
  }

  return 0;
}
