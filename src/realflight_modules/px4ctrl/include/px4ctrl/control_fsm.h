#ifndef PX4CTRL_CONTROL_FSM_H_
#define PX4CTRL_CONTROL_FSM_H_

#include <ros/ros.h>
#include <ros/assert.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/CommandBool.h>

#include <px4ctrl/input_data.h>
// #include "ThrustCurve.h"
#include <px4ctrl/controller.h>

namespace px4ctrl {

struct AutoTakeoffLandState {
    bool landed{true};
    ros::Time toggle_takeoff_land_time;
    std::pair<bool, ros::Time> delay_trigger{std::pair<bool, ros::Time>(false, ros::Time(0))};
    Eigen::Vector4d start_pose;

    static constexpr double kMotorsSpeedupTime = 3.0; // motors idle running for 3 seconds before takeoff
    static constexpr double kDelayTriggerTime = 2.0;  // Time to be delayed when reach at target height
};

class Px4CtrlFsm {
  public:
    Parameters &parameters_;

    RcData rc_data_;
    StateData state_data_;
    ExtendedStateData extended_state_data_;
    OdometryData odom_data_;
    OdometryData tag_odom_data_;
    ImuData imu_data_;
    CommandData command_data_;
    BatteryData battery_data_;
    TakeoffLandData takeoff_land_data_;

    Controller &controller_;

    ros::Publisher trajectory_start_trigger_pub_;
    ros::Publisher control_fcu_pub_;
    ros::Publisher debug_pub_; //debug
    ros::Publisher takeoff_land_command_pub_;
    ros::ServiceClient set_fcu_mode_service_;
    ros::ServiceClient arming_service_;
    ros::ServiceClient reboot_fcu_service_;

    quadrotor_msgs::Px4ctrlDebug debug_msg_; //debug

    Eigen::Vector4d hover_pose_;
    ros::Time last_set_hover_pose_time_;

    enum State {
        kManualControl = 1, // px4ctrl is deactived. FCU is controled by the remote controller only
        kAutoHover, // px4ctrl is actived, it will keep the drone hover from odom measurments while waiting for commands from PositionCommand topic.
        kCommandControl,	// px4ctrl is actived, and controling the drone.
        kAutoTakeoff,
        kAutoLand
    };

    Px4CtrlFsm(Parameters &, Controller &);
    void Process();
    bool RcIsReceived(const ros::Time &now_time);
    bool CommandIsReceived(const ros::Time &now_time);
    bool OdometryIsReceived(const ros::Time &now_time);
    bool ImuIsReceived(const ros::Time &now_time);
    bool BatteryIsReceived(const ros::Time &now_time);
    bool ReceiveNewOdometry();
    State state() const {
        return state_;
    }
    bool landed() const {
        return takeoff_land_state_.landed;
    }

  private:
    State state_; // Should only be changed in Px4CtrlFsm::Process() function!
    AutoTakeoffLandState takeoff_land_state_;

    // ---- control related ----
    DesiredState GetHoverDesiredState();
    DesiredState GetCommandDesiredState();

    // ---- auto takeoff/land ----
    void MotorsIdling(const ImuData &imu, ControllerOutput &u);
    void DetectLanding(State current_state, const DesiredState &des, const OdometryData &odom); // Detect landing
    void SetStartPoseForTakeoffLand(const OdometryData &odom);
    DesiredState GetRotorSpeedUpDesiredState(const ros::Time now);
    DesiredState GetTakeoffLandDesiredState(const double speed);
    void ProcessRcTakeoffLandTrigger();


    // ---- tools ----
    void SetHoverFromOdometry();
    void SetHoverFromRc();

    bool ToggleOffboardMode(bool on_off); // It will only try to toggle once, so not blocked.
    bool ToggleArmDisarm(bool arm); // It will only try to toggle once, so not blocked.
    void RebootFcu();

    void PublishBodyrateControl(const ControllerOutput &u, const ros::Time &stamp);
    void PublishAttitudeControl(const ControllerOutput &u, const ros::Time &stamp);
    void PublishTrigger(const nav_msgs::Odometry &odom_msg);
};

}  // namespace px4ctrl

#endif
