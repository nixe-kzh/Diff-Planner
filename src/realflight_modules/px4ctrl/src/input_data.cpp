#include <px4ctrl/input_data.h>
#include <rosfmt/rosfmt.h>

namespace px4ctrl {

RcData::RcData() {
    rcv_stamp_ = ros::Time(0);

    last_mode_ = -1.0;
    last_gear_ = -1.0;
    last_reboot_cmd_ = -1.0;

    // Parameter initilation is very important in RC-Free usage!
    is_hover_mode_ = true;
    enter_hover_mode_ = false;
    is_command_mode_ = true;
    enter_command_mode_ = false;
    toggle_reboot_ = false;
    takeoff_land_triggered_ = false;
    takeoff_land_trigger_enabled_ = false;
    takeoff_land_trigger_channel_ = 9;
    takeoff_land_trigger_threshold_ = 1750;
    have_init_takeoff_land_switch_ = false;
    last_takeoff_land_switch_high_ = false;
    for (int i = 0; i < 4; ++i) {
        ch_[i] = 0.0;
    }
}

void RcData::ConfigureTakeoffLandTrigger(bool enabled, int channel, int threshold) {
    takeoff_land_trigger_enabled_ = enabled;
    takeoff_land_trigger_channel_ = channel - 1; // ROS parameter uses human-readable, one-based channel numbers.
    takeoff_land_trigger_threshold_ = threshold;
    takeoff_land_triggered_ = false;
    have_init_takeoff_land_switch_ = false;
    last_takeoff_land_switch_high_ = false;
}

void RcData::Feed(mavros_msgs::RCInConstPtr message) {
    msg_ = *message;
    if (msg_.channels.size() < 8) {
        ROSFMT_ERROR_THROTTLE(1.0, "Insufficient RC channels: {} received, 8 required", msg_.channels.size());
        return;
    }

    rcv_stamp_ = ros::Time::now();

    for (int i = 0; i < 4; i++) {
        ch_[i] = ((double)msg_.channels[i] - 1500.0) / 500.0;
        if (ch_[i] > kDeadZone)
            ch_[i] = (ch_[i] - kDeadZone) / (1 - kDeadZone);
        else if (ch_[i] < -kDeadZone)
            ch_[i] = (ch_[i] + kDeadZone) / (1 - kDeadZone);
        else
            ch_[i] = 0.0;
    }

    mode_ = ((double)msg_.channels[4] - 1000.0) / 1000.0;
    gear_ = ((double)msg_.channels[5] - 1000.0) / 1000.0;
    reboot_cmd_ = ((double)msg_.channels[7] - 1000.0) / 1000.0;

    CheckValidity();

    if (!have_init_last_mode_) {
        have_init_last_mode_ = true;
        last_mode_ = mode_;
    }
    if (!have_init_last_gear_) {
        have_init_last_gear_ = true;
        last_gear_ = gear_;
    }
    if (!have_init_last_reboot_cmd_) {
        have_init_last_reboot_cmd_ = true;
        last_reboot_cmd_ = reboot_cmd_;
    }

    // 1
    if (last_mode_ < kApiModeThresholdValue && mode_ > kApiModeThresholdValue)
        enter_hover_mode_ = true;
    else
        enter_hover_mode_ = false;

    if (mode_ > kApiModeThresholdValue)
        is_hover_mode_ = true;
    else
        is_hover_mode_ = false;

    // 2
    if (is_hover_mode_) {
        if (last_gear_ < kGearShiftValue && gear_ > kGearShiftValue)
            enter_command_mode_ = true;
        else if (gear_ < kGearShiftValue)
            enter_command_mode_ = false;

        if (gear_ > kGearShiftValue)
            is_command_mode_ = true;
        else
            is_command_mode_ = false;
    }

    // 3
    if (!is_hover_mode_ && !is_command_mode_) {
        if (last_reboot_cmd_ < kRebootThresholdValue && reboot_cmd_ > kRebootThresholdValue)
            toggle_reboot_ = true;
        else
            toggle_reboot_ = false;
    } else {
        toggle_reboot_ = false;
    }

    if (takeoff_land_trigger_enabled_) {
        if (takeoff_land_trigger_channel_ >= static_cast<int>(msg_.channels.size())) {
            ROSFMT_ERROR_THROTTLE(1.0,
                                  "RC trigger channel {} unavailable; {} channels received",
                                  takeoff_land_trigger_channel_ + 1, msg_.channels.size());
            have_init_takeoff_land_switch_ = false;
        } else {
            const bool switch_high = msg_.channels[takeoff_land_trigger_channel_] > takeoff_land_trigger_threshold_;
            if (!have_init_takeoff_land_switch_) {
                have_init_takeoff_land_switch_ = true;
            } else if (!last_takeoff_land_switch_high_ && switch_high) {
                takeoff_land_triggered_ = true;
                ROSFMT_INFO("RC takeoff/land triggered on channel {}",
                            takeoff_land_trigger_channel_ + 1);
            }
            last_takeoff_land_switch_high_ = switch_high;
        }
    }

    last_mode_ = mode_;
    last_gear_ = gear_;
    last_reboot_cmd_ = reboot_cmd_;
}

void RcData::CheckValidity() {
    if (mode_ >= -1.1 && mode_ <= 1.1 && gear_ >= -1.1 && gear_ <= 1.1 && reboot_cmd_ >= -1.1 && reboot_cmd_ <= 1.1) {
        // pass
    } else {
        ROSFMT_ERROR("Invalid RC data: mode={:.2f}, gear={:.2f}, reboot={:.2f}",
                     mode_, gear_, reboot_cmd_);
    }
}

bool RcData::CheckCentered() {
    bool centered = fabs(ch_[0]) < 1e-5 && fabs(ch_[1]) < 1e-5 && fabs(ch_[2]) < 1e-5 && fabs(ch_[3]) < 1e-5;
    return centered;
}

OdometryData::OdometryData() {
    rcv_stamp_ = ros::Time(0);
    q_.setIdentity();
    recv_new_msg_ = false;
};

void OdometryData::Feed(nav_msgs::OdometryConstPtr message) {
    ros::Time now = ros::Time::now();

    msg_ = *message;
    rcv_stamp_ = now;
    recv_new_msg_ = true;

    uav_utils::extract_odometry(message, p_, v_, q_, w_);

// #define PX4CTRL_VELOCITY_IN_BODY
#ifdef PX4CTRL_VELOCITY_IN_BODY /* Set to 1 if the velocity in odom topic is relative to current body frame, not to world frame.*/
    Eigen::Quaternion<double> w_rb_q(msg_.pose.pose.orientation.w, msg_.pose.pose.orientation.x, msg_.pose.pose.orientation.y, msg_.pose.pose.orientation.z);
    Eigen::Matrix3d w_rb = w_rb_q.matrix();
    v_ = w_rb * v_;

    static int count = 0;
    if (count++ % 500 == 0)
        ROSFMT_WARN("Odometry velocity interpreted in body frame");
#endif

    // check the frequency
    static int one_min_count = 9999;
    static ros::Time last_clear_count_time = ros::Time(0.0);
    if ( (now - last_clear_count_time).toSec() > 1.0 ) {
        if ( one_min_count < 100 ) {
            ROSFMT_WARN("Odometry rate below 100 Hz");
        }
        one_min_count = 0;
        last_clear_count_time = now;
    }
    one_min_count ++;
}

ImuData::ImuData() {
    rcv_stamp_ = ros::Time(0);
}

void ImuData::Feed(sensor_msgs::ImuConstPtr message) {
    ros::Time now = ros::Time::now();

    msg_ = *message;
    rcv_stamp_ = now;

    w_(0) = msg_.angular_velocity.x;
    w_(1) = msg_.angular_velocity.y;
    w_(2) = msg_.angular_velocity.z;

    a_(0) = msg_.linear_acceleration.x;
    a_(1) = msg_.linear_acceleration.y;
    a_(2) = msg_.linear_acceleration.z;

    q_.x() = msg_.orientation.x;
    q_.y() = msg_.orientation.y;
    q_.z() = msg_.orientation.z;
    q_.w() = msg_.orientation.w;

    // check the frequency
    static int one_min_count = 9999;
    static ros::Time last_clear_count_time = ros::Time(0.0);
    if ( (now - last_clear_count_time).toSec() > 1.0 ) {
        if ( one_min_count < 100 ) {
            ROSFMT_WARN("IMU rate below 100 Hz");
        }
        one_min_count = 0;
        last_clear_count_time = now;
    }
    one_min_count ++;
}

StateData::StateData() {
}

void StateData::Feed(mavros_msgs::StateConstPtr message) {

    current_state_ = *message;
}

ExtendedStateData::ExtendedStateData() {
}

void ExtendedStateData::Feed(mavros_msgs::ExtendedStateConstPtr message) {
    current_extended_state_ = *message;
}

CommandData::CommandData() {
    rcv_stamp_ = ros::Time(0);
}

void CommandData::Feed(quadrotor_msgs::PositionCommandConstPtr message) {

    msg_ = *message;
    rcv_stamp_ = ros::Time::now();

    p_(0) = msg_.position.x;
    p_(1) = msg_.position.y;
    p_(2) = msg_.position.z;

    v_(0) = msg_.velocity.x;
    v_(1) = msg_.velocity.y;
    v_(2) = msg_.velocity.z;

    a_(0) = msg_.acceleration.x;
    a_(1) = msg_.acceleration.y;
    a_(2) = msg_.acceleration.z;

    j_(0) = msg_.jerk.x;
    j_(1) = msg_.jerk.y;
    j_(2) = msg_.jerk.z;

    // std::cout << "j1=" << j.transpose() << std::endl;

    yaw_ = uav_utils::normalize_angle(msg_.yaw);
    yaw_rate_ = msg_.yaw_dot;
}

BatteryData::BatteryData() {
    rcv_stamp_ = ros::Time(0);
}

void BatteryData::Feed(sensor_msgs::BatteryStateConstPtr message) {

    msg_ = *message;
    rcv_stamp_ = ros::Time::now();

    double voltage = 0;
    for (size_t i = 0; i < message->cell_voltage.size(); ++i) {
        voltage += message->cell_voltage[i];
    }
    volt_ = 0.8 * volt_ + 0.2 * voltage; // Naive LPF, cell_voltage has a higher frequency

    // volt_ = 0.8 * volt_ + 0.2 * message->voltage; // Naive LPF
    percentage_ = message->percentage;

    static ros::Time last_print_t = ros::Time(0);
    if (percentage_ > 0.05) {
        if ((rcv_stamp_ - last_print_t).toSec() > 10) {
            ROSFMT_INFO("Battery: {:.3f} V, {:.1f}%", volt_, percentage_ * 100.0);
            last_print_t = rcv_stamp_;
        }
    } else {
        if ((rcv_stamp_ - last_print_t).toSec() > 1) {
            ROSFMT_ERROR("Battery critical: {:.3f} V, {:.1f}%", volt_, percentage_ * 100.0);
            last_print_t = rcv_stamp_;
        }
    }
}

TakeoffLandData::TakeoffLandData() {
    rcv_stamp_ = ros::Time(0);
}

void TakeoffLandData::Feed(quadrotor_msgs::TakeoffLandConstPtr message) {

    msg_ = *message;
    rcv_stamp_ = ros::Time::now();

    triggered_ = true;
    takeoff_land_cmd_ = message->takeoff_land_cmd;
}

}  // namespace px4ctrl
