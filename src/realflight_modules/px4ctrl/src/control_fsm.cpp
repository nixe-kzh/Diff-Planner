#include <px4ctrl/control_fsm.h>
#include <rosfmt/rosfmt.h>
#include <uav_utils/converters.h>

using namespace std;
using namespace uav_utils;

namespace px4ctrl {

Px4CtrlFsm::Px4CtrlFsm(Parameters &parameters, Controller &controller) : parameters_(parameters), controller_(controller) { /*, thrust_curve(thrust_curve_)*/
    state_ = kManualControl;
    hover_pose_.setZero();
    rc_data_.ConfigureTakeoffLandTrigger(parameters_.takeoff_land_.enable_rc_trigger,
                                         parameters_.takeoff_land_.rc_trigger_channel,
                                         parameters_.takeoff_land_.rc_trigger_threshold);
}

/*
        Finite State Machine

	      system start
	            |
	            |
	            v
	----- > kManualControl <-----------------
	|         ^   |    \                 |
	|         |   |     \                |
	|         |   |      > kAutoTakeoff  |
	|         |   |        /             |
	|         |   |       /              |
	|         |   |      /               |
	|         |   v     /                |
	|       kAutoHover <                 |
	|         ^   |  \  \                |
	|         |   |   \  \               |
	|         |	  |    > kAutoLand -------
	|         |   |
	|         |   v
	-------- kCommandControl

*/

void Px4CtrlFsm::Process() {
    ros::Time now_time = ros::Time::now();
    ControllerOutput u;
    DesiredState des(odom_data_);
    bool rotor_low_speed_during_land = false;

    ProcessRcTakeoffLandTrigger();

    // STEP1: state_ machine runs
    switch (state_) {
    case kManualControl: {
        if (rc_data_.enter_hover_mode_) { // Try to jump to kAutoHover
            if (!OdometryIsReceived(now_time)) {
                ROSFMT_ERROR("Auto hover rejected: odometry unavailable");
                break;
            }
            if (CommandIsReceived(now_time)) {
                ROSFMT_ERROR("Auto hover rejected: command input is active");
                break;
            }
            if (odom_data_.v_.norm() > 3.0) {
                ROSFMT_ERROR("Auto hover rejected: odometry speed {:.2f} m/s", odom_data_.v_.norm());
                break;
            }

            state_ = kAutoHover;
            controller_.ResetThrustMapping();
            SetHoverFromOdometry();
            ToggleOffboardMode(true);

            ROSFMT_INFO("State: manual -> auto hover");
        } else if (parameters_.takeoff_land_.enable && takeoff_land_data_.triggered_ && takeoff_land_data_.takeoff_land_cmd_ == quadrotor_msgs::TakeoffLand::TAKEOFF) { // Try to jump to kAutoTakeoff
            if (!OdometryIsReceived(now_time)) {
                ROSFMT_ERROR("Auto takeoff rejected: odometry unavailable");
                break;
            }
            if (CommandIsReceived(now_time)) {
                ROSFMT_ERROR("Auto takeoff rejected: command input is active");
                break;
            }
            if (odom_data_.v_.norm() > 1) {
                ROSFMT_ERROR("Auto takeoff rejected: speed {:.2f} m/s", odom_data_.v_.norm());
                break;
            }
            if (!landed()) {
                ROSFMT_ERROR("Auto takeoff rejected: vehicle is airborne");
                break;
            }
            if (RcIsReceived(now_time)) { // Check this only if RC is connected.
                if (!rc_data_.is_hover_mode_ || !rc_data_.is_command_mode_ || !rc_data_.CheckCentered()) {
                    ROSFMT_ERROR("Auto takeoff rejected: select hover and command modes, then center sticks");
                    while (ros::ok()) {
                        ros::Duration(0.01).sleep();
                        ros::spinOnce();
                        if (rc_data_.is_hover_mode_ && rc_data_.is_command_mode_ && rc_data_.CheckCentered()) {
                            ROSFMT_INFO("Auto takeoff controls ready");
                            break;
                        }
                    }
                    break;
                }
            }

            state_ = kAutoTakeoff;
            controller_.ResetThrustMapping();
            SetStartPoseForTakeoffLand(odom_data_);
            ToggleOffboardMode(true);				  // toggle on offboard before arm
            for (int i = 0; i < 10 && ros::ok(); ++i) { // wait for 0.1 seconds to allow mode change by FMU // mark
                ros::Duration(0.01).sleep();
                ros::spinOnce();
            }
            if (parameters_.takeoff_land_.enable_auto_arm) {
                ToggleArmDisarm(true);
            }
            takeoff_land_state_.toggle_takeoff_land_time = now_time;

            ROSFMT_INFO("State: manual -> auto takeoff");
        }

        if (rc_data_.toggle_reboot_) { // Try to reboot. EKF2 based PX4 FCU requires reboot when its state_ estimator goes wrong.
            if (state_data_.current_state_.armed) {
                ROSFMT_ERROR("FCU reboot rejected: vehicle is armed");
                break;
            }
            RebootFcu();
        }

        break;
    }

    case kAutoHover: {
        if (!rc_data_.is_hover_mode_ || !OdometryIsReceived(now_time)) {
            state_ = kManualControl;
            ToggleOffboardMode(false);

            ROSFMT_WARN("State: auto hover -> manual");
        } else if (rc_data_.is_command_mode_ && CommandIsReceived(now_time)) {
            if (state_data_.current_state_.mode == "OFFBOARD") {
                state_ = kCommandControl;
                des = GetCommandDesiredState();
                ROSFMT_INFO("State: auto hover -> command control");
            }
        } else if (takeoff_land_data_.triggered_ && takeoff_land_data_.takeoff_land_cmd_ == quadrotor_msgs::TakeoffLand::LAND) {

            state_ = kAutoLand;
            SetStartPoseForTakeoffLand(odom_data_);

            ROSFMT_INFO("State: auto hover -> auto land");
        } else {
            SetHoverFromRc();
            des = GetHoverDesiredState();
            if ((rc_data_.enter_command_mode_) ||
                    (takeoff_land_state_.delay_trigger.first && now_time > takeoff_land_state_.delay_trigger.second)) {
                takeoff_land_state_.delay_trigger.first = false;
                PublishTrigger(odom_data_.msg_);
                ROSFMT_INFO("Command trigger published");
            }

            // cout << "des.p=" << des.p.transpose() << endl;
        }

        break;
    }

    case kCommandControl: {
        if (!rc_data_.is_hover_mode_ || !OdometryIsReceived(now_time)) {
            state_ = kManualControl;
            ToggleOffboardMode(false);

            ROSFMT_WARN("State: command control -> manual");
        } else if (!rc_data_.is_command_mode_ || !CommandIsReceived(now_time)) {
            state_ = kAutoHover;
            SetHoverFromOdometry();
            des = GetHoverDesiredState();
            ROSFMT_INFO("State: command control -> auto hover");
        } else {
            des = GetCommandDesiredState();
        }

        if (takeoff_land_data_.triggered_ && takeoff_land_data_.takeoff_land_cmd_ == quadrotor_msgs::TakeoffLand::LAND) {
            ROSFMT_ERROR("Auto land rejected: command input is active; wait {:.1f} s for auto hover",
                         parameters_.message_timeout_.cmd);
        }

        break;
    }

    case kAutoTakeoff: {
        if ((now_time - takeoff_land_state_.toggle_takeoff_land_time).toSec() < AutoTakeoffLandState::kMotorsSpeedupTime) { // Wait for several seconds to warn prople.
            des = GetRotorSpeedUpDesiredState(now_time);
        } else if (odom_data_.p_(2) >= (takeoff_land_state_.start_pose(2) + parameters_.takeoff_land_.height)) { // reach the desired height
            state_ = kAutoHover;
            SetHoverFromOdometry();
            ROSFMT_INFO("State: auto takeoff -> auto hover");

            takeoff_land_state_.delay_trigger.first = true;
            takeoff_land_state_.delay_trigger.second = now_time + ros::Duration(AutoTakeoffLandState::kDelayTriggerTime);
        } else {
            des = GetTakeoffLandDesiredState(parameters_.takeoff_land_.speed);
        }

        break;
    }

    case kAutoLand: {
        if (!rc_data_.is_hover_mode_ || !OdometryIsReceived(now_time)) {
            state_ = kManualControl;
            ToggleOffboardMode(false);

            ROSFMT_WARN("State: auto land -> manual");
        } else if (!rc_data_.is_command_mode_) {
            state_ = kAutoHover;
            SetHoverFromOdometry();
            des = GetHoverDesiredState();
            ROSFMT_INFO("State: auto land -> auto hover");
        } else if (!landed()) {
            des = GetTakeoffLandDesiredState(-parameters_.takeoff_land_.speed);
        } else {
            rotor_low_speed_during_land = true;

            static bool print_once_flag = true;
            if (print_once_flag) {
                ROSFMT_INFO("Landing detected; waiting to disarm");
                print_once_flag = false;
            }

            if (extended_state_data_.current_extended_state_.landed_state == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND) { // PX4 allows disarm after this
                static double last_trial_time = 0; // Avoid too frequent calls
                if (now_time.toSec() - last_trial_time > 1.0) {
                    if (ToggleArmDisarm(false)) { // disarm
                        print_once_flag = true;
                        state_ = kManualControl;
                        ToggleOffboardMode(false); // toggle off offboard after disarm
                        ROSFMT_INFO("State: auto land -> manual");
                    }

                    last_trial_time = now_time.toSec();
                }
            }
        }

        break;
    }

    default:
        break;
    }

    // STEP2: solve and update new control commands
    if (rotor_low_speed_during_land) { // used at the start of auto takeoff
        MotorsIdling(imu_data_, u);
    } else {
        switch (parameters_.pose_solver_) {
        case 0:
            debug_msg_ = controller_.UpdateAlg0(des, odom_data_, imu_data_, u, battery_data_.volt_);
            debug_msg_.header.stamp = now_time;
            debug_pub_.publish(debug_msg_);
            break;
        case 1:
            debug_msg_ = controller_.UpdateAlg1(des, odom_data_, imu_data_, u, battery_data_.volt_);
            debug_msg_.header.stamp = now_time;
            debug_pub_.publish(debug_msg_);
            break;

        case 2:
            controller_.UpdateAlg2(des, odom_data_, imu_data_, u, battery_data_.volt_);
            break;

        default:
            ROSFMT_ERROR("Invalid pose solver: {}", parameters_.pose_solver_);
            return;
        }
    }

    // STEP3: estimate thrust model
    if (state_ == kAutoHover || state_ == kCommandControl) {
        controller_.EstimateThrustModel(imu_data_.a_, battery_data_.volt_, odom_data_.v_, parameters_);
    }

    // STEP4: publish control commands to mavros
    PublishAttitudeControl(u, now_time);

    // STEP5: Detect if the drone has landed
    DetectLanding(state_, des, odom_data_);
    // cout << takeoff_land_state_.landed << " ";
    // fflush(stdout);

    // STEP6: Clear flags beyound their lifetime
    rc_data_.enter_hover_mode_ = false;
    rc_data_.enter_command_mode_ = false;
    rc_data_.toggle_reboot_ = false;
    takeoff_land_data_.triggered_ = false;
}

void Px4CtrlFsm::ProcessRcTakeoffLandTrigger() {
    if (!rc_data_.takeoff_land_triggered_)
        return;

    // Consume every switch edge exactly once, including rejected commands.
    rc_data_.takeoff_land_triggered_ = false;

    if (!parameters_.takeoff_land_.enable) {
        ROSFMT_WARN("RC takeoff/land rejected: feature disabled");
        return;
    }

    quadrotor_msgs::TakeoffLand msg;
    if (state_ == kManualControl && landed()) {
        if (!rc_data_.is_hover_mode_ || !rc_data_.is_command_mode_ || !rc_data_.CheckCentered()) {
            ROSFMT_WARN("RC takeoff rejected: select hover and command modes, then center sticks");
            return;
        }
        msg.takeoff_land_cmd = quadrotor_msgs::TakeoffLand::TAKEOFF;
        // An explicit takeoff trigger wins if the hover-mode switch rose in the same RC frame.
        rc_data_.enter_hover_mode_ = false;
        ROSFMT_INFO("RC takeoff accepted");
    } else if (state_ == kAutoHover && !landed()) {
        msg.takeoff_land_cmd = quadrotor_msgs::TakeoffLand::LAND;
        ROSFMT_INFO("RC land accepted");
    } else {
        ROSFMT_WARN("RC takeoff/land rejected: state={}, landed={}",
                    static_cast<int>(state_), landed());
        return;
    }

    // Reuse the same topic path as takeoff.sh/land.sh. The existing subscriber
    // feeds this command into the FSM exactly once on the next spin.
    takeoff_land_command_pub_.publish(msg);
}

void Px4CtrlFsm::MotorsIdling(const ImuData &imu, ControllerOutput &u) {
    u.q = imu.q_;
    u.bodyrates = Eigen::Vector3d::Zero();
    u.thrust = 0.04;
}

void Px4CtrlFsm::DetectLanding(const State current_state, const DesiredState &des, const OdometryData &odom) {
    static State last_state = State::kManualControl;
    if (last_state == State::kManualControl && (current_state == State::kAutoHover || current_state == State::kAutoTakeoff)) {
        takeoff_land_state_.landed = false; // Always holds
    }
    last_state = current_state;

    if (current_state == State::kManualControl && !state_data_.current_state_.armed) {
        takeoff_land_state_.landed = true;
        return; // No need of other decisions
    }

    // DetectLanding parameters
    constexpr double kPositionDeviation = -0.5; // Constraint 1: target position below real position for kPositionDeviation meters.
    constexpr double kVelocityThreshold = 0.1;		  // Constraint 2: velocity below VELOCITY_MIN_C m/s.
    constexpr double kTimeKeep = 3.0;			  // Constraint 3: Time(s) the Constraint 1&2 need to keep.

    static ros::Time time_c12_reached; // time_Constraints12_reached
    static bool last_c12_satisfied;
    if (takeoff_land_state_.landed) {
        time_c12_reached = ros::Time::now();
        last_c12_satisfied = false;
    } else {
        bool c12_satisfied = (des.p(2) - odom.p_(2)) < kPositionDeviation && odom.v_.norm() < kVelocityThreshold;
        if (c12_satisfied && !last_c12_satisfied) {
            time_c12_reached = ros::Time::now();
        } else if (c12_satisfied && last_c12_satisfied) {
            if ((ros::Time::now() - time_c12_reached).toSec() > kTimeKeep) { //Constraint 3 reached
                takeoff_land_state_.landed = true;
            }
        }

        last_c12_satisfied = c12_satisfied;
    }
}

DesiredState Px4CtrlFsm::GetHoverDesiredState() {
    DesiredState des;
    des.p = hover_pose_.head<3>();
    des.v = Eigen::Vector3d::Zero();
    des.a = Eigen::Vector3d::Zero();
    des.j = Eigen::Vector3d::Zero();
    des.yaw = hover_pose_(3);
    des.yaw_rate = 0.0;

    return des;
}

DesiredState Px4CtrlFsm::GetCommandDesiredState() {
    DesiredState des;
    des.p = command_data_.p_;
    des.v = command_data_.v_;
    des.a = command_data_.a_;
    des.j = command_data_.j_;
    des.yaw = command_data_.yaw_;
    des.yaw_rate = command_data_.yaw_rate_;

    return des;
}

DesiredState Px4CtrlFsm::GetRotorSpeedUpDesiredState(const ros::Time now) {
    double delta_t = (now - takeoff_land_state_.toggle_takeoff_land_time).toSec();
    double des_a_z = exp((delta_t - AutoTakeoffLandState::kMotorsSpeedupTime) * 6.0) * 7.0 - 7.0; // Parameters 6.0 and 7.0 are just heuristic values which result in a saticfactory curve.
    if (des_a_z > 0.1) {
        ROSFMT_ERROR("Takeoff acceleration out of range: {:.3f} m/s^2", des_a_z);
        des_a_z = 0.0;
    }

    DesiredState des;
    des.p = takeoff_land_state_.start_pose.head<3>();
    des.v = Eigen::Vector3d::Zero();
    des.a = Eigen::Vector3d(0, 0, des_a_z);
    des.j = Eigen::Vector3d::Zero();
    des.yaw = takeoff_land_state_.start_pose(3);
    des.yaw_rate = 0.0;

    return des;
}

DesiredState Px4CtrlFsm::GetTakeoffLandDesiredState(const double speed) {
    ros::Time now = ros::Time::now();
    double delta_t = (now - takeoff_land_state_.toggle_takeoff_land_time).toSec() - (speed > 0 ? AutoTakeoffLandState::kMotorsSpeedupTime : 0); // speed > 0 means takeoff
    // takeoff_land_state_.last_set_cmd_time = now;

    // takeoff_land_state_.start_pose(2) += speed * delta_t;

    DesiredState des;
    if (speed >= 0) {
        des.p = takeoff_land_state_.start_pose.head<3>() + Eigen::Vector3d(0, 0, speed * delta_t);
        des.v = Eigen::Vector3d(0, 0, speed);
    } else {
        if (tag_odom_data_.recv_new_msg_) {
            double dist_tolerance = 0.05;
            double dist_to_tag = (tag_odom_data_.p_.head(2) -  odom_data_.p_.head(2)).norm();
            des.p = takeoff_land_state_.start_pose.head<3>();
            des.p.head(2) += (tag_odom_data_.p_.head(2) -  odom_data_.p_.head(2)) / dist_to_tag * 0.2 / parameters_.ctrl_freq_max_;
            des.v = Eigen::Vector3d(0, 0, 0);
            des.v.head(2) = (tag_odom_data_.p_.head(2) -  odom_data_.p_.head(2)) / dist_to_tag * 0.2;
            static bool is_landing = false;
            if (dist_to_tag > dist_tolerance && !is_landing) {
                takeoff_land_state_.start_pose.head(3) = des.p;
                takeoff_land_state_.toggle_takeoff_land_time = now;
            } else {
                is_landing = true;
                des.p += Eigen::Vector3d(0, 0, speed * delta_t);
                if (dist_to_tag < dist_tolerance) {
                    des.v.head(2) /= (dist_tolerance / dist_to_tag * 2.0);
                }
                des.v += Eigen::Vector3d(0, 0, speed);
            }
        } else {
            des.p = takeoff_land_state_.start_pose.head<3>() + Eigen::Vector3d(0, 0, speed * delta_t);
            des.v = Eigen::Vector3d(0, 0, speed);
        }
    }
    // std::cout << "des p = " << des.p.transpose() << " des v = " <<  des.v.transpose() << std::endl;
    des.a = Eigen::Vector3d::Zero();
    des.j = Eigen::Vector3d::Zero();
    des.yaw = takeoff_land_state_.start_pose(3);
    des.yaw_rate = 0.0;

    return des;
}

void Px4CtrlFsm::SetHoverFromOdometry() {
    hover_pose_.head<3>() = odom_data_.p_;
    hover_pose_(3) = get_yaw_from_quaternion(odom_data_.q_);

    last_set_hover_pose_time_ = ros::Time::now();
}

void Px4CtrlFsm::SetHoverFromRc() {
    ros::Time now = ros::Time::now();
    double delta_t = (now - last_set_hover_pose_time_).toSec();
    last_set_hover_pose_time_ = now;

    hover_pose_(0) += rc_data_.ch_[1] * parameters_.max_manual_vel_ * delta_t * (parameters_.rc_reverse_.pitch ? 1 : -1);
    hover_pose_(1) += rc_data_.ch_[0] * parameters_.max_manual_vel_ * delta_t * (parameters_.rc_reverse_.roll ? 1 : -1);
    hover_pose_(2) += rc_data_.ch_[2] * parameters_.max_manual_vel_ * delta_t * (parameters_.rc_reverse_.throttle ? 1 : -1);
    hover_pose_(3) += rc_data_.ch_[3] * parameters_.max_manual_vel_ * delta_t * (parameters_.rc_reverse_.yaw ? 1 : -1);

    if (hover_pose_(2) < -0.3)
        hover_pose_(2) = -0.3;

    // if (parameters_.print_dbg)
    // {
    // 	static unsigned int count = 0;
    // 	if (count++ % 100 == 0)
    // 	{
    // 		cout << "hover_pose_=" << hover_pose_.transpose() << endl;
    // 		cout << "ch[0~3]=" << rc_data_.ch_[0] << " " << rc_data_.ch_[1] << " " << rc_data_.ch_[2] << " " << rc_data_.ch_[3] << endl;
    // 	}
    // }
}

void Px4CtrlFsm::SetStartPoseForTakeoffLand(const OdometryData &odom) {
    takeoff_land_state_.start_pose.head<3>() = odom_data_.p_;
    takeoff_land_state_.start_pose(3) = get_yaw_from_quaternion(odom_data_.q_);

    takeoff_land_state_.toggle_takeoff_land_time = ros::Time::now();
}

bool Px4CtrlFsm::RcIsReceived(const ros::Time &now_time) {
    return (now_time - rc_data_.rcv_stamp_).toSec() < parameters_.message_timeout_.rc;
}

bool Px4CtrlFsm::CommandIsReceived(const ros::Time &now_time) {
    return (now_time - command_data_.rcv_stamp_).toSec() < parameters_.message_timeout_.cmd;
}

bool Px4CtrlFsm::OdometryIsReceived(const ros::Time &now_time) {
    return (now_time - odom_data_.rcv_stamp_).toSec() < parameters_.message_timeout_.odom;
}

bool Px4CtrlFsm::ImuIsReceived(const ros::Time &now_time) {
    return (now_time - imu_data_.rcv_stamp_).toSec() < parameters_.message_timeout_.imu;
}

bool Px4CtrlFsm::BatteryIsReceived(const ros::Time &now_time) {
    return (now_time - battery_data_.rcv_stamp_).toSec() < parameters_.message_timeout_.bat;
}

bool Px4CtrlFsm::ReceiveNewOdometry() {
    if (odom_data_.recv_new_msg_) {
        odom_data_.recv_new_msg_ = false;
        return true;
    }

    return false;
}

void Px4CtrlFsm::PublishBodyrateControl(const ControllerOutput &u, const ros::Time &stamp) {
    mavros_msgs::AttitudeTarget msg;

    msg.header.stamp = stamp;
    msg.header.frame_id = std::string("FCU");

    msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;

    msg.body_rate.x = u.bodyrates.x();
    msg.body_rate.y = u.bodyrates.y();
    msg.body_rate.z = u.bodyrates.z();

    msg.thrust = u.thrust;

    control_fcu_pub_.publish(msg);
}

void Px4CtrlFsm::PublishAttitudeControl(const ControllerOutput &u, const ros::Time &stamp) {
    mavros_msgs::AttitudeTarget msg;

    msg.header.stamp = stamp;
    msg.header.frame_id = std::string("FCU");

    msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
                    mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
                    mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;

    msg.orientation.x = u.q.x();
    msg.orientation.y = u.q.y();
    msg.orientation.z = u.q.z();
    msg.orientation.w = u.q.w();

    msg.thrust = u.thrust;

    control_fcu_pub_.publish(msg);
}

void Px4CtrlFsm::PublishTrigger(const nav_msgs::Odometry &odom_msg) {
    geometry_msgs::PoseStamped msg;
    msg.header.frame_id = "world";
    msg.pose = odom_msg.pose.pose;

    trajectory_start_trigger_pub_.publish(msg);
}

bool Px4CtrlFsm::ToggleOffboardMode(bool on_off) {
    mavros_msgs::SetMode offb_set_mode;

    if (on_off) {
        state_data_.state_before_offboard_ = state_data_.current_state_;
        if (state_data_.state_before_offboard_.mode == "OFFBOARD") // Not allowed
            state_data_.state_before_offboard_.mode = "MANUAL";

        offb_set_mode.request.custom_mode = "OFFBOARD";
        if (!(set_fcu_mode_service_.call(offb_set_mode) && offb_set_mode.response.mode_sent)) {
            ROSFMT_ERROR("PX4 rejected OFFBOARD entry");
            return false;
        }
    } else {
        offb_set_mode.request.custom_mode = state_data_.state_before_offboard_.mode;
        if (!(set_fcu_mode_service_.call(offb_set_mode) && offb_set_mode.response.mode_sent)) {
            ROSFMT_ERROR("PX4 rejected OFFBOARD exit");
            return false;
        }
    }

    return true;

    // if (parameters_.print_dbg)
    // 	printf("offb_set_mode mode_sent=%d(uint8_t)\n", offb_set_mode.response.mode_sent);
}

bool Px4CtrlFsm::ToggleArmDisarm(bool arm) {
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = arm;
    if (!(arming_service_.call(arm_cmd) && arm_cmd.response.success)) {
        if (arm)
            ROSFMT_ERROR("PX4 rejected arming; check kill switch");
        else
            ROSFMT_ERROR("PX4 rejected disarming");

        return false;
    }

    return true;
}

void Px4CtrlFsm::RebootFcu() {
    // https://mavlink.io/en/messages/common.html, MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN(#246)
    mavros_msgs::CommandLong reboot_srv;
    reboot_srv.request.broadcast = false;
    reboot_srv.request.command = 246; // MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
    reboot_srv.request.param1 = 1;	  // Reboot autopilot
    reboot_srv.request.param2 = 0;	  // Do nothing for onboard computer
    reboot_srv.request.confirmation = true;

    reboot_fcu_service_.call(reboot_srv);

    ROSFMT_INFO("FCU reboot requested");

    // if (parameters_.print_dbg)
    // 	printf("reboot result=%d(uint8_t), success=%d(uint8_t)\n", reboot_srv.response.result, reboot_srv.response.success);
}

}  // namespace px4ctrl
