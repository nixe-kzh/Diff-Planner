#include <ros/ros.h>
#include <rosfmt/rosfmt.h>
#include <px4ctrl/control_fsm.h>
#include <signal.h>
#include <quadrotor_msgs/Px4ctrlState.h>

void HandleSigint(int sig) {
    ROSFMT_INFO("Shutting down");
    ros::shutdown();
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "px4ctrl");
    ros::NodeHandle nh("~");

    signal(SIGINT, HandleSigint);
    ros::Duration(1.0).sleep();

    px4ctrl::Parameters parameters;
    parameters.ConfigFromRosHandle(nh);

    px4ctrl::Controller controller(parameters);
    px4ctrl::Px4CtrlFsm fsm(parameters, controller);

    ros::Subscriber state_sub =
        nh.subscribe<mavros_msgs::State>("/mavros/state",
                10,
                boost::bind(&px4ctrl::StateData::Feed, &fsm.state_data_, _1));

    ros::Subscriber extended_state_sub =
        nh.subscribe<mavros_msgs::ExtendedState>("/mavros/extended_state",
                10,
                boost::bind(&px4ctrl::ExtendedStateData::Feed, &fsm.extended_state_data_, _1));

    ros::Subscriber odom_sub =
        nh.subscribe<nav_msgs::Odometry>("odom",
                100,
                boost::bind(&px4ctrl::OdometryData::Feed, &fsm.odom_data_, _1),
                ros::VoidConstPtr(),
                ros::TransportHints().tcpNoDelay());

    ros::Subscriber cmd_sub =
        nh.subscribe<quadrotor_msgs::PositionCommand>("cmd",
                100,
                boost::bind(&px4ctrl::CommandData::Feed, &fsm.command_data_, _1),
                ros::VoidConstPtr(),
                ros::TransportHints().tcpNoDelay());

    ros::Subscriber imu_sub =
        nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", // Note: do NOT change it to /mavros/imu/data_raw !!!
                100,
                boost::bind(&px4ctrl::ImuData::Feed, &fsm.imu_data_, _1),
                ros::VoidConstPtr(),
                ros::TransportHints().tcpNoDelay());

    ros::Subscriber rc_sub;
    if (parameters.takeoff_land_.no_rc) {
        ROSFMT_INFO("RC input: /mavros/rc/in_sim (simulation)");
        rc_sub = nh.subscribe<mavros_msgs::RCIn>("/mavros/rc/in_sim",
                10,
                boost::bind(&px4ctrl::RcData::Feed, &fsm.rc_data_, _1));
    } else {
        ROSFMT_INFO("RC input: /mavros/rc/in (real flight)");
        rc_sub = nh.subscribe<mavros_msgs::RCIn>("/mavros/rc/in",
                10,
                boost::bind(&px4ctrl::RcData::Feed, &fsm.rc_data_, _1));
    }

    ros::Subscriber bat_sub =
        nh.subscribe<sensor_msgs::BatteryState>("/mavros/battery",
                100,
                boost::bind(&px4ctrl::BatteryData::Feed, &fsm.battery_data_, _1),
                ros::VoidConstPtr(),
                ros::TransportHints().tcpNoDelay());

    ros::Subscriber takeoff_land_sub =
        nh.subscribe<quadrotor_msgs::TakeoffLand>("takeoff_land",
                100,
                boost::bind(&px4ctrl::TakeoffLandData::Feed, &fsm.takeoff_land_data_, _1),
                ros::VoidConstPtr(),
                ros::TransportHints().tcpNoDelay());

    ros::Publisher px4ctrl_state_pub =
                nh.advertise<quadrotor_msgs::Px4ctrlState>("/px4ctrl/state", 5);

    fsm.control_fcu_pub_ =
                nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);

    fsm.trajectory_start_trigger_pub_ = 
                nh.advertise<geometry_msgs::PoseStamped>("/traj_start_trigger", 10);

    fsm.takeoff_land_command_pub_ = 
                nh.advertise<quadrotor_msgs::TakeoffLand>("takeoff_land", 10);
    
    // debug msg
    fsm.debug_pub_ = 
                nh.advertise<quadrotor_msgs::Px4ctrlDebug>("/debugPx4ctrl", 10); 

    // px4 services
    fsm.set_fcu_mode_service_ = 
                nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    fsm.arming_service_ = 
                nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    fsm.reboot_fcu_service_ = 
                nh.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");

    ros::Duration(0.5).sleep();

    ROSFMT_INFO("Waiting for RC input");
    while (ros::ok()) {
        ros::spinOnce();
        if (fsm.RcIsReceived(ros::Time::now())) {
            ROSFMT_INFO("RC input received");
            break;
        }
        ros::Duration(0.1).sleep();
    }

    int trials = 0;
    while (ros::ok() && !fsm.state_data_.current_state_.connected) {
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        if (trials++ > 5)
            ROSFMT_ERROR("PX4 connection unavailable");
    }

    ros::Rate control_rate(parameters.ctrl_freq_max_);
    while (ros::ok()) {
        control_rate.sleep();
        ros::spinOnce();
        fsm.Process(); // We DO NOT rely on feedback as trigger, since there is no significant performance difference through our test.

        quadrotor_msgs::Px4ctrlState state_msg;
        state_msg.header.stamp = ros::Time::now();
        state_msg.header.frame_id = "px4ctrl";
        switch (fsm.state()) {
        case px4ctrl::Px4CtrlFsm::kManualControl:
            state_msg.state = quadrotor_msgs::Px4ctrlState::MANUAL_CTRL;
            break;
        case px4ctrl::Px4CtrlFsm::kAutoHover:
            state_msg.state = quadrotor_msgs::Px4ctrlState::AUTO_HOVER;
            break;
        case px4ctrl::Px4CtrlFsm::kCommandControl:
            state_msg.state = quadrotor_msgs::Px4ctrlState::CMD_CTRL;
            break;
        case px4ctrl::Px4CtrlFsm::kAutoTakeoff:
            state_msg.state = quadrotor_msgs::Px4ctrlState::AUTO_TAKEOFF;
            break;
        case px4ctrl::Px4CtrlFsm::kAutoLand:
            state_msg.state = quadrotor_msgs::Px4ctrlState::AUTO_LAND;
            break;
        default:
            state_msg.state = 0;
            break;
        }
        px4ctrl_state_pub.publish(state_msg);
    }

    return 0;
}
