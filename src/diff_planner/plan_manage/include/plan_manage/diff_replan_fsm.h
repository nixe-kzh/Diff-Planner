#ifndef DIFF_PLANNER_PLAN_MANAGE_INCLUDE_PLAN_MANAGE_DIFF_REPLAN_FSM_H_
#define DIFF_PLANNER_PLAN_MANAGE_INCLUDE_PLAN_MANAGE_DIFF_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <string>
#include <utility>
#include <vector>
#include <visualization_msgs/Marker.h>

#include <optimizer/poly_traj_optimizer.h>
#include <plan_env/grid_map.h>
#include <geometry_msgs/PoseStamped.h>
#include <quadrotor_msgs/GoalSet.h>
#include <traj_utils/DataDisp.h>
#include <plan_manage/planner_manager.h>
#include <traj_utils/planning_visualization.h>
#include <traj_utils/PolyTraj.h>
#include <traj_utils/MINCOTraj.h>

using std::vector;

namespace diff_planner
{

  class DiffReplanFSM
  {
  public:
    DiffReplanFSM() {}
    ~DiffReplanFSM() {}

    void Init(ros::NodeHandle &node_handle);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  private:
    /* ---------- flag ---------- */
    enum FsmExecutionState
    {
      kInit,
      kWaitTarget,
      kGenerateNewTrajectory,
      kReplanTrajectory,
      kExecuteTrajectory,
      kEmergencyStop,
      kSequentialStart
    };
    enum TargetType
    {
      kManualTarget = 1,
      kPresetTarget = 2,
      kReferencePath = 3
    };
    /* Anomaly Detection Parameters */
    Eigen::Vector3d last_local_target_position_;
    double last_target_change_time_;
    int replan_failure_count_;
    static constexpr double kTargetStuckThreshold = 0.3;  // Threshold for target movement below which it's considered "stuck"
    double target_stuck_time_;                           // Default time threshold (seconds) for being considered stuck before reinitialization
    static constexpr int kMaxReplanFailureCount = 10;    // Threshold for maximum optimization failure count
    /* planning utils */
    DiffPlannerManager::Ptr planner_manager_;
    PlanningVisualization::Ptr visualization_;
    traj_utils::DataDisp display_data_;

    /* parameters */
    int target_type_; // 1 mannual select, 2 hard code
    double no_replan_threshold_, replan_threshold_;
    double waypoints_[50][3];
    int waypoint_count_, waypoint_index_;
    double planning_horizon_;
    double emergency_time_;
    bool is_real_world_experiment_;
    bool enable_fail_safe_;
    bool enable_ground_height_measurement_;
    bool escape_emergency_;
    bool need_hover_stop_;
    bool modify_final_goal_;
    bool enable_stuck_detect_; // Whether to enable stuck detection

    bool has_trigger_, has_target_, has_odometry_, has_new_target_, has_received_previous_agent_, touch_goal_, mandatory_stop_;
    FsmExecutionState execution_state_;
    int consecutive_call_count_{0};

    Eigen::Vector3d start_point_, start_velocity_, start_acceleration_;   // start state
    Eigen::Vector3d final_goal_;                             // goal state
    Eigen::Vector3d local_target_point_, local_target_velocity_; // local target state
    Eigen::Vector3d odometry_position_, odometry_velocity_, odometry_acceleration_;     // odometry state
    std::vector<Eigen::Vector3d> waypoint_positions_;

    /* ROS utils */
    ros::NodeHandle node_;
    ros::Timer execution_timer_, safety_timer_;
    ros::Subscriber waypoint_subscriber_, odometry_subscriber_, trigger_subscriber_, broadcast_trajectory_subscriber_, mandatory_stop_subscriber_;
    ros::Publisher polynomial_trajectory_publisher_, display_data_publisher_, broadcast_trajectory_publisher_, heartbeat_publisher_, ground_height_publisher_;

    /* state machine functions */
    void ExecutionTimerCallback(const ros::TimerEvent &event);
    void ChangeExecutionState(FsmExecutionState new_state,
                              std::string caller);
    void PrintExecutionState();
    std::pair<int, DiffReplanFSM::FsmExecutionState> GetConsecutiveStateCalls();

    /* safety */
    void SafetyTimerCallback(const ros::TimerEvent &event);
    bool CallEmergencyStop(Eigen::Vector3d stop_position);

    /* local planning */
    bool CallReboundReplan(bool use_polynomial_initialization,
                           bool use_random_polynomial_trajectory);
    bool PlanFromGlobalTrajectory(int trial_count = 1);
    bool PlanFromLocalTrajectory(int trial_count = 1);

    /* global trajectory */
    void WaypointCallback(const geometry_msgs::PoseStampedPtr &message);
    void ReadGivenWaypointsAndPlan();
    bool PlanNextWaypoint(const Eigen::Vector3d next_waypoint,
                          bool trigger_replan);
    bool ModifyInCollisionFinalGoal();
    void FinishProcess();

    /* input-output */
    void MandatoryStopCallback(const std_msgs::Empty &message);
    void OdometryCallback(const nav_msgs::OdometryConstPtr &message);
    void TriggerCallback(const geometry_msgs::PoseStampedPtr &message);
    void ReceiveBroadcastMincoTrajectoryCallback(
        const traj_utils::MINCOTrajConstPtr &message);
    void ConvertPolynomialTrajectoryToRosMessage(
        traj_utils::PolyTraj &polynomial_message,
        traj_utils::MINCOTraj &minco_message);

    /* ground height measurement */
    bool MeasureGroundHeight(double &height);
    Eigen::Vector3d ProjectPointToLineSegment(
        const Eigen::Vector3d &line_start,
        const Eigen::Vector3d &line_end,
        const Eigen::Vector3d &point);
  };

}  // namespace diff_planner

#endif  // DIFF_PLANNER_PLAN_MANAGE_INCLUDE_PLAN_MANAGE_DIFF_REPLAN_FSM_H_
