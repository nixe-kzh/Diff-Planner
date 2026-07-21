#include <px4ctrl/parameters.h>

namespace px4ctrl {

Parameters::Parameters() {
}

void Parameters::ConfigFromRosHandle(const ros::NodeHandle &nh) {
    ReadEssentialParam(nh, "gain/kp0", gain_.kp0);
    ReadEssentialParam(nh, "gain/kp1", gain_.kp1);
    ReadEssentialParam(nh, "gain/kp2", gain_.kp2);
    ReadEssentialParam(nh, "gain/kv0", gain_.kv0);
    ReadEssentialParam(nh, "gain/kv1", gain_.kv1);
    ReadEssentialParam(nh, "gain/kv2", gain_.kv2);
    ReadEssentialParam(nh, "gain/kvi0", gain_.kvi0);
    ReadEssentialParam(nh, "gain/kvi1", gain_.kvi1);
    ReadEssentialParam(nh, "gain/kvi2", gain_.kvi2);
    ReadEssentialParam(nh, "gain/kvd0", gain_.kvd0);
    ReadEssentialParam(nh, "gain/kvd1", gain_.kvd1);
    ReadEssentialParam(nh, "gain/kvd2", gain_.kvd2);
    ReadEssentialParam(nh, "gain/k_ang_r", gain_.k_ang_r);
    ReadEssentialParam(nh, "gain/k_ang_p", gain_.k_ang_p);
    ReadEssentialParam(nh, "gain/k_ang_y", gain_.k_ang_y);

    ReadEssentialParam(nh, "rotor_drag/x", rotor_drag_.x);
    ReadEssentialParam(nh, "rotor_drag/y", rotor_drag_.y);
    ReadEssentialParam(nh, "rotor_drag/z", rotor_drag_.z);
    ReadEssentialParam(nh, "rotor_drag/k_thrust_horz", rotor_drag_.k_thrust_horz);

    ReadEssentialParam(nh, "msg_timeout/odom", message_timeout_.odom);
    ReadEssentialParam(nh, "msg_timeout/rc", message_timeout_.rc);
    ReadEssentialParam(nh, "msg_timeout/cmd", message_timeout_.cmd);
    ReadEssentialParam(nh, "msg_timeout/imu", message_timeout_.imu);
    ReadEssentialParam(nh, "msg_timeout/bat", message_timeout_.bat);

    ReadEssentialParam(nh, "pose_solver", pose_solver_);
    ReadEssentialParam(nh, "mass", mass_);
    ReadEssentialParam(nh, "gra", gra_);
    ReadEssentialParam(nh, "ctrl_freq_max", ctrl_freq_max_);
    ReadEssentialParam(nh, "max_manual_vel", max_manual_vel_);
    ReadEssentialParam(nh, "max_angle", max_angle_);
    ReadEssentialParam(nh, "low_voltage", low_voltage_);

    ReadEssentialParam(nh, "rc_reverse/roll", rc_reverse_.roll);
    ReadEssentialParam(nh, "rc_reverse/pitch", rc_reverse_.pitch);
    ReadEssentialParam(nh, "rc_reverse/yaw", rc_reverse_.yaw);
    ReadEssentialParam(nh, "rc_reverse/throttle", rc_reverse_.throttle);

    ReadEssentialParam(nh, "auto_takeoff_land/enable", takeoff_land_.enable);
    ReadEssentialParam(nh, "auto_takeoff_land/enable_auto_arm", takeoff_land_.enable_auto_arm);
    ReadEssentialParam(nh, "auto_takeoff_land/no_rc", takeoff_land_.no_rc);
    ReadEssentialParam(nh, "auto_takeoff_land/takeoff_height", takeoff_land_.height);
    ReadEssentialParam(nh, "auto_takeoff_land/takeoff_land_speed", takeoff_land_.speed);
    nh.param("auto_takeoff_land/rc_trigger/enabled", takeoff_land_.enable_rc_trigger, false);
    nh.param("auto_takeoff_land/rc_trigger/channel", takeoff_land_.rc_trigger_channel, 10);
    nh.param("auto_takeoff_land/rc_trigger/threshold", takeoff_land_.rc_trigger_threshold, 1750);

    ReadEssentialParam(nh, "thrust_model/print_value", thrust_mapping_.print_val);
    ReadEssentialParam(nh, "thrust_model/k1", thrust_mapping_.k1);
    ReadEssentialParam(nh, "thrust_model/k2", thrust_mapping_.k2);
    ReadEssentialParam(nh, "thrust_model/k3", thrust_mapping_.k3);
    ReadEssentialParam(nh, "thrust_model/accurate_thrust_model", thrust_mapping_.accurate_thrust_model);
    ReadEssentialParam(nh, "thrust_model/hover_percentage", thrust_mapping_.hover_percentage);
    ReadEssentialParam(nh, "thrust_model/noisy_imu", thrust_mapping_.noisy_imu);


    max_angle_ /= (180.0 / M_PI);

    if ( takeoff_land_.enable_auto_arm && !takeoff_land_.enable ) {
        takeoff_land_.enable_auto_arm = false;
        ROSFMT_ERROR("Auto-arm requires auto takeoff/land; disabling auto-arm");
    }
    if (takeoff_land_.rc_trigger_channel < 1 || takeoff_land_.rc_trigger_channel > 18) {
        takeoff_land_.enable_rc_trigger = false;
        ROSFMT_ERROR("Invalid RC trigger channel: {}; disabling trigger", takeoff_land_.rc_trigger_channel);
    }
    if (takeoff_land_.rc_trigger_threshold < 800 || takeoff_land_.rc_trigger_threshold > 2200) {
        takeoff_land_.enable_rc_trigger = false;
        ROSFMT_ERROR("Invalid RC trigger threshold: {}; disabling trigger", takeoff_land_.rc_trigger_threshold);
    }

    if ( thrust_mapping_.print_val ) {
        ROSFMT_WARN("Thrust model value logging is enabled");
    }
};
}  // namespace px4ctrl
