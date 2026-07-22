#ifndef PX4CTRL_INPUT_DATA_H_
#define PX4CTRL_INPUT_DATA_H_

#include <ros/ros.h>
#include <Eigen/Dense>

#include <sensor_msgs/Imu.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/TakeoffLand.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <sensor_msgs/BatteryState.h>
#include <uav_utils/utils.h>
#include <px4ctrl/parameters.h>

namespace px4ctrl {

class RcData {
  public:
    double mode_;
    double gear_;
    double reboot_cmd_;
    double last_mode_;
    double last_gear_;
    double last_reboot_cmd_;
    bool have_init_last_mode_{false};
    bool have_init_last_gear_{false};
    bool have_init_last_reboot_cmd_{false};
    double ch_[4];

    mavros_msgs::RCIn msg_;
    ros::Time rcv_stamp_;

    bool is_command_mode_;
    bool enter_command_mode_;
    bool is_hover_mode_;
    bool enter_hover_mode_;
    bool toggle_reboot_;

    bool takeoff_land_triggered_;
    bool takeoff_land_trigger_enabled_;
    int takeoff_land_trigger_channel_;
    int takeoff_land_trigger_threshold_;
    bool have_init_takeoff_land_switch_;
    bool last_takeoff_land_switch_high_;

    static constexpr double kGearShiftValue = 0.75;
    static constexpr double kApiModeThresholdValue = 0.75;
    static constexpr double kRebootThresholdValue = 0.5;
    static constexpr double kDeadZone = 0.25;

    RcData();
    void CheckValidity();
    bool CheckCentered();
    void ConfigureTakeoffLandTrigger(bool enabled, int channel, int threshold);
    void Feed(mavros_msgs::RCInConstPtr message);
    bool IsReceived(const ros::Time &now_time);
};

class OdometryData {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d p_;
    Eigen::Vector3d v_;
    Eigen::Quaterniond q_;
    Eigen::Vector3d w_;

    nav_msgs::Odometry msg_;
    ros::Time rcv_stamp_;
    bool recv_new_msg_;

    OdometryData();
    void Feed(nav_msgs::OdometryConstPtr message);
};

class ImuData {
  public:
    Eigen::Quaterniond q_;
    Eigen::Vector3d w_;
    Eigen::Vector3d a_;

    sensor_msgs::Imu msg_;
    ros::Time rcv_stamp_;

    ImuData();
    void Feed(sensor_msgs::ImuConstPtr message);
};

class StateData {
  public:
    mavros_msgs::State current_state_;
    mavros_msgs::State state_before_offboard_;

    StateData();
    void Feed(mavros_msgs::StateConstPtr message);
};

class ExtendedStateData {
  public:
    mavros_msgs::ExtendedState current_extended_state_;

    ExtendedStateData();
    void Feed(mavros_msgs::ExtendedStateConstPtr message);
};

class CommandData {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d p_;
    Eigen::Vector3d v_;
    Eigen::Vector3d a_;
    Eigen::Vector3d j_;
    double yaw_;
    double yaw_rate_;

    quadrotor_msgs::PositionCommand msg_;
    ros::Time rcv_stamp_;

    CommandData();
    void Feed(quadrotor_msgs::PositionCommandConstPtr message);
};

class BatteryData {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double volt_{0.0};
    double percentage_{0.0};

    sensor_msgs::BatteryState msg_;
    ros::Time rcv_stamp_;

    BatteryData();
    void Feed(sensor_msgs::BatteryStateConstPtr message);
};

class TakeoffLandData {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    bool triggered_{false};
    uint8_t takeoff_land_cmd_; // see TakeoffLand.msg_ for its defination

    quadrotor_msgs::TakeoffLand msg_;
    ros::Time rcv_stamp_;

    TakeoffLandData();
    void Feed(quadrotor_msgs::TakeoffLandConstPtr message);
};

}  // namespace px4ctrl

#endif
