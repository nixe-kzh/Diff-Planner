#ifndef PX4CTRL_PARAMETERS_H_
#define PX4CTRL_PARAMETERS_H_

#include <ros/ros.h>
#include <rosfmt/rosfmt.h>

namespace px4ctrl {

class Parameters {
  public:
    struct Gain {
        double kp0, kp1, kp2;
        double kv0, kv1, kv2;
        double kvi0, kvi1, kvi2;
        double kvd0, kvd1, kvd2;
        double k_ang_r, k_ang_p, k_ang_y;
    };

    struct RotorDrag {
        double x, y, z;
        double k_thrust_horz;
    };

    struct MessageTimeout {
        double odom;
        double rc;
        double cmd;
        double imu;
        double bat;
    };

    struct ThrustMapping {
        bool print_val;
        double k1;
        double k2;
        double k3;
        bool accurate_thrust_model;
        double hover_percentage;
        bool noisy_imu;
    };

    struct RcReverse {
        bool roll;
        bool pitch;
        bool yaw;
        bool throttle;
    };

    struct AutoTakeoffLand {
        bool enable;
        bool enable_auto_arm;
        bool no_rc;
        double height;
        double speed;
        bool enable_rc_trigger;
        int rc_trigger_channel;
        int rc_trigger_threshold;
    };

    Gain gain_;
    RotorDrag rotor_drag_;
    MessageTimeout message_timeout_;
    RcReverse rc_reverse_;
    ThrustMapping thrust_mapping_;
    AutoTakeoffLand takeoff_land_;

    int pose_solver_;
    double mass_;
    double gra_;
    double max_angle_;
    double ctrl_freq_max_;
    double max_manual_vel_;
    double low_voltage_;

    // bool print_dbg;

    Parameters();
    void ConfigFromRosHandle(const ros::NodeHandle &nh);
    void ConfigFullThrust(double hov);

  private:
    template <typename TName, typename TVal>
    void ReadEssentialParam(const ros::NodeHandle &nh, const TName &name, TVal &val) {
        if (nh.getParam(name, val)) {
            // pass
        } else {
            ROSFMT_ERROR("Missing required parameter: {}", name);
            ROS_BREAK();
        }
    };
};

}  // namespace px4ctrl

#endif
