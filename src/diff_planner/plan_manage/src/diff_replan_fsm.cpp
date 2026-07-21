
#include <plan_manage/diff_replan_fsm.h>

namespace diff_planner
{

  void DiffReplanFSM::Init(ros::NodeHandle &node_handle)
  {
    execution_state_ = FsmExecutionState::kInit;
    has_target_ = false;
    has_odometry_ = false;
    has_received_previous_agent_ = false;
    escape_emergency_ = true;
    mandatory_stop_ = false;

    /*  fsm param  */
    node_handle.param("fsm/flight_type", target_type_, -1);
    node_handle.param("fsm/thresh_replan_time", replan_threshold_, -1.0);
    node_handle.param("fsm/planning_horizon", planning_horizon_, -1.0);
    node_handle.param("fsm/emergency_time", emergency_time_, 1.0);
    node_handle.param("fsm/realworld_experiment", is_real_world_experiment_, false);
    node_handle.param("fsm/fail_safe", enable_fail_safe_, true);
    node_handle.param("fsm/ground_height_measurement", enable_ground_height_measurement_, false);
    node_handle.param("fsm/mondify_final_goal", modify_final_goal_, true);
    node_handle.param("fsm/enable_stuck_detect", enable_stuck_detect_, true);

    node_handle.param("fsm/waypoint_num", waypoint_count_, -1);
    for (int i = 0; i < waypoint_count_; i++)
    {
      node_handle.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
      node_handle.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
      node_handle.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
    }


    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(node_handle));
    planner_manager_.reset(new DiffPlannerManager);
    planner_manager_->InitPlanModules(node_handle, visualization_);

    has_trigger_ = !is_real_world_experiment_;
    no_replan_threshold_ = 0.5 * emergency_time_ * planner_manager_->plan_parameters_.max_velocity;

    /* initialize  Anomaly Detection Parameters */
    last_local_target_position_.setZero();
    last_target_change_time_ = ros::Time::now().toSec();
    replan_failure_count_ = 0;
    target_stuck_time_ = 1.5 * planning_horizon_ / planner_manager_->plan_parameters_.max_velocity;
    need_hover_stop_ = false;

    /* callback */
    execution_timer_ = node_handle.createTimer(ros::Duration(0.01), &DiffReplanFSM::ExecutionTimerCallback, this);
    safety_timer_ = node_handle.createTimer(ros::Duration(0.05), &DiffReplanFSM::SafetyTimerCallback, this);

    odometry_subscriber_ = node_handle.subscribe("odom_world", 1, &DiffReplanFSM::OdometryCallback, this);
    mandatory_stop_subscriber_ = node_handle.subscribe("mandatory_stop", 1, &DiffReplanFSM::MandatoryStopCallback, this);

    /* Use MINCO trajectory to minimize the message size in wireless communication */
    broadcast_trajectory_publisher_ = node_handle.advertise<traj_utils::MINCOTraj>("planning/broadcast_traj_send", 10);
    broadcast_trajectory_subscriber_ = node_handle.subscribe<traj_utils::MINCOTraj>("planning/broadcast_traj_recv", 100,
                                                                  &DiffReplanFSM::ReceiveBroadcastMincoTrajectoryCallback,
                                                                  this,
                                                                  ros::TransportHints().tcpNoDelay());

    polynomial_trajectory_publisher_ = node_handle.advertise<traj_utils::PolyTraj>("planning/trajectory", 10);
    display_data_publisher_ = node_handle.advertise<traj_utils::DataDisp>("planning/data_display", 100);
    heartbeat_publisher_ = node_handle.advertise<std_msgs::Empty>("planning/heartbeat", 10);
    ground_height_publisher_ = node_handle.advertise<std_msgs::Float64>("/ground_height_measurement", 10);

    if (target_type_ == TargetType::kManualTarget)
    {
      waypoint_subscriber_ = node_handle.subscribe("/goal", 1, &DiffReplanFSM::WaypointCallback, this);
    }
    else if (target_type_ == TargetType::kPresetTarget)
    {
      trigger_subscriber_ = node_handle.subscribe("/traj_start_trigger", 1, &DiffReplanFSM::TriggerCallback, this);

      ROS_INFO("Wait for 2 second.");
      int wait_iteration = 0;
      while (ros::ok() && wait_iteration++ < 2000)
      {
        ros::spinOnce();
        ros::Duration(0.001).sleep();
      }

      ReadGivenWaypointsAndPlan();
    }
    else
      cout << "Wrong target_type_ value! target_type_=" << target_type_ << endl;
  }

  void DiffReplanFSM::ExecutionTimerCallback(const ros::TimerEvent &event)
  {
    execution_timer_.stop(); // To avoid blockage
    std_msgs::Empty heartbeat_message;
    heartbeat_publisher_.publish(heartbeat_message);

    static int callback_count = 0;
    callback_count++;
    if (callback_count == 500)
    {
      callback_count = 0;
      PrintExecutionState();
    }

    switch (execution_state_)
    {
    case kInit:
    {
      if (!has_odometry_)
      {
        goto force_return; // return;
      }
      ChangeExecutionState(kWaitTarget, "FSM");
      break;
    }

    case kWaitTarget:
    {
      if (!has_target_ || !has_trigger_)
        goto force_return; // return;
      else
      {
        ChangeExecutionState(kSequentialStart, "FSM");
      }
      break;
    }

    case kSequentialStart: // for swarm or single drone with drone_id = 0
    {
      if (planner_manager_->plan_parameters_.drone_id <= 0 || (planner_manager_->plan_parameters_.drone_id >= 1 && has_received_previous_agent_))
      {
        if (!modify_final_goal_ && planner_manager_->grid_map_->GetInflatedOccupancy(final_goal_))
        {
          ROS_WARN("Final goal in obstacle, unsafe. Emergency stop.");
          need_hover_stop_ = true;
          escape_emergency_ = true;
          ChangeExecutionState(kEmergencyStop, "STUCK_DETECT");
        }
        else
        {
          bool success = PlanFromGlobalTrajectory(10); // zx-todo
          if (success)
          {
            replan_failure_count_ = 0;
            ChangeExecutionState(kExecuteTrajectory, "FSM");
          }
          else
          {
            ROS_WARN("Failed to generate the first trajectory, keep trying");
            replan_failure_count_++;
            ChangeExecutionState(kSequentialStart, "FSM"); // "changeFSMExecState" must be called each time planned
          }
        }
      }
      break;
    }

    case kGenerateNewTrajectory:
    {
      if (!modify_final_goal_ && planner_manager_->grid_map_->GetInflatedOccupancy(final_goal_))
      {
        ROS_WARN("Final goal in obstacle, unsafe. Emergency stop.");
        need_hover_stop_ = true;
        escape_emergency_ = true;
        ChangeExecutionState(kEmergencyStop, "STUCK_DETECT");
      }
      else
      {
        bool success = PlanFromGlobalTrajectory(10); // zx-todo
        if (success)
        {
          replan_failure_count_ = 0;
          ChangeExecutionState(kExecuteTrajectory, "FSM");
          escape_emergency_ = true;
        }
        else
        {
          replan_failure_count_++;
          ChangeExecutionState(kGenerateNewTrajectory, "FSM"); // "changeFSMExecState" must be called each time planned
        }
      }
      break;
    }

    case kReplanTrajectory:
    {

      if (PlanFromLocalTrajectory(1))
      {
        replan_failure_count_ = 0;
        ChangeExecutionState(kExecuteTrajectory, "FSM");
      }
      else
      {
        replan_failure_count_++;
        ChangeExecutionState(kReplanTrajectory, "FSM");
      }

      break;
    }

    case kExecuteTrajectory:
    {
      /* determine if need to replan */
      LocalTrajectoryData *local_trajectory = &planner_manager_->trajectory_container_.local_trajectory_;
      double current_time = ros::Time::now().toSec() - local_trajectory->start_time;
      current_time = min(local_trajectory->duration, current_time);
      Eigen::Vector3d position = local_trajectory->trajectory.GetPosition(current_time);
      bool reached_goal = ((local_target_point_ - final_goal_).norm() < 1e-2);

      const PointsToCheck *check_points = &planner_manager_->trajectory_container_.local_trajectory_.points_to_check;
      bool close_to_current_trajectory_end = (check_points->size() >= 1 && check_points->back().size() >= 1) ? check_points->back().back().first - current_time < emergency_time_ : 0; // In case of empty vector

      if (planner_manager_->grid_map_->GetInflatedOccupancy(final_goal_))
      {
        if (!modify_final_goal_)
        {
          ROS_WARN("Final goal in obstacle, unsafe. Emergency stop.");
          need_hover_stop_ = true;
          escape_emergency_ = true;
          ChangeExecutionState(kEmergencyStop, "STUCK_DETECT");
        }
        else if (ModifyInCollisionFinalGoal())
        {
          ROS_WARN("Successfully modified final_goal in EXEC_TRAJ !!!");
          ChangeExecutionState(kReplanTrajectory, "mondify_FSM");
        }
      }
      else if ((target_type_ == TargetType::kPresetTarget) &&
               (waypoint_index_ < waypoint_count_ - 1) &&
               (final_goal_ - position).norm() < no_replan_threshold_) // case 2: assign the next waypoint
      {
        waypoint_index_++;
        PlanNextWaypoint(waypoint_positions_[waypoint_index_], true);
      }
      else if ((current_time > local_trajectory->duration - 1e-2) && reached_goal) // case 3: the final waypoint reached
      {
        has_target_ = false;
        has_trigger_ = false;
        if (target_type_ == TargetType::kPresetTarget)
        {
          // prepare for next round
          waypoint_index_ = 0;
          PlanNextWaypoint(waypoint_positions_[waypoint_index_], true);
        }

        /* The navigation task completed */
        ChangeExecutionState(kWaitTarget, "FSM");
      }
      else if (current_time > replan_threshold_ || (!reached_goal && close_to_current_trajectory_end)) // case 3: time to perform next replan
      {
        ChangeExecutionState(kReplanTrajectory, "FSM");
      }
      // ROS_ERROR("AAAA");
      if (enable_stuck_detect_)
      {
        /* Avoid getting stuck wandering around large obstacles */
        static bool baseline_initialized = false;
        if (reached_goal)
        {
          static double last_projected_length = 0.0;
          static Eigen::Vector3d baseline_start = odometry_position_;
          static Eigen::Vector3d baseline_goal = final_goal_;
          if (!baseline_initialized || (baseline_goal - final_goal_).norm() > 0.1)
          {
            baseline_start = odometry_position_;
            baseline_goal = final_goal_;
            last_projected_length = 0.0;
            baseline_initialized = true;
          }
          Eigen::Vector3d current_position = odometry_position_;
          Eigen::Vector3d baseline_to_current = current_position - baseline_start;
          Eigen::Vector3d projected_position = ProjectPointToLineSegment(baseline_start, final_goal_, current_position);
          double projected_length = (projected_position - baseline_start).norm();
          if (projected_length - last_projected_length < kTargetStuckThreshold)
          {
            if (ros::Time::now().toSec() - last_target_change_time_ > target_stuck_time_)
            {
              ROS_WARN("Drone stuck! Obstacle too large and near final goal. Emergency stop.");
              need_hover_stop_ = true;
              escape_emergency_ = true;
              ChangeExecutionState(kEmergencyStop, "STUCK_DETECT");
            }
          }
          else
          {
            last_projected_length = projected_length;
            last_target_change_time_ = ros::Time::now().toSec();
          }

          if (baseline_to_current.norm() > planning_horizon_ * M_SQRT2)
          {
            ROS_WARN("Drone stuck! The drone flew too far out of its way . Emergency stop.");
            need_hover_stop_ = true;
            escape_emergency_ = true;
            ChangeExecutionState(kEmergencyStop, "STUCK_DETECT");
          }
        }
        else
        {
          baseline_initialized = false;
          if ((local_target_point_ - last_local_target_position_).norm() < kTargetStuckThreshold)
          {
            if (ros::Time::now().toSec() - last_target_change_time_ > target_stuck_time_)
            {
              ROS_WARN("Drone stuck! Obstacle too large. Emergency stop.");
              need_hover_stop_ = true;
              escape_emergency_ = true;
              ChangeExecutionState(kEmergencyStop, "STUCK_DETECT");
            }
          }
          else
          {
            last_local_target_position_ = local_target_point_;
            last_target_change_time_ = ros::Time::now().toSec();
          }
        }
      }
      break;
    }

    case kEmergencyStop:
    {
      if (escape_emergency_) // Avoiding repeated calls
      {
        CallEmergencyStop(odometry_position_);
      }
      else
      {
        if (enable_fail_safe_ && !need_hover_stop_ && odometry_velocity_.norm() < 0.1)
        {
          last_target_change_time_ = ros::Time::now().toSec();
          ChangeExecutionState(kGenerateNewTrajectory, "FSM");
        }
        else if (enable_fail_safe_ && need_hover_stop_ && odometry_velocity_.norm() < 0.1)
        {
          ROS_INFO("Exiting EMERGENCY_STOP. Switching to WAIT_TARGET. Need a new target point !!!");
          need_hover_stop_ = false;
          has_target_ = false;
          has_trigger_ = false;
          ChangeExecutionState(kWaitTarget, "EMERGENCY_EXIT");
        }
      }

      escape_emergency_ = false;
      break;
    }
    }
    FinishProcess();
    display_data_.header.stamp = ros::Time::now();
    display_data_publisher_.publish(display_data_);

  force_return:;
    execution_timer_.start();
  }
  void DiffReplanFSM::FinishProcess()
  {
    if (replan_failure_count_ > kMaxReplanFailureCount)
    {
      ROS_WARN("replan fail too much. Emergency stop.");
      replan_failure_count_ = 0;
      need_hover_stop_ = true;
      escape_emergency_ = true;
      ChangeExecutionState(kEmergencyStop, "finishProcess");
    }
  }

  void DiffReplanFSM::ChangeExecutionState(FsmExecutionState new_state,
                                           std::string caller)
  {

    if (new_state == execution_state_)
      consecutive_call_count_++;
    else
      consecutive_call_count_ = 1;

    static const std::string state_names[8] = {
        "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ",
        "EMERGENCY_STOP", "SEQUENTIAL_START"};
    int previous_state = int(execution_state_);
    execution_state_ = new_state;
    cout << "[" + caller + "]"
         << "Drone:" << planner_manager_->plan_parameters_.drone_id << ", from " + state_names[previous_state] + " to " + state_names[int(new_state)] << endl;
  }

  void DiffReplanFSM::PrintExecutionState()
  {
    static const std::string state_names[8] = {
        "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ",
        "EMERGENCY_STOP", "SEQUENTIAL_START"};

    cout << "\r[FSM]: state: " + state_names[int(execution_state_)] << ", Drone:" << planner_manager_->plan_parameters_.drone_id;

    // some warnings
    if (!has_odometry_ || !has_target_ || !has_trigger_ || (planner_manager_->plan_parameters_.drone_id >= 1 && !has_received_previous_agent_))
    {
      cout << ". Waiting for ";
    }
    if (!has_odometry_)
    {
      cout << "odom,";
    }
    if (!has_target_)
    {
      cout << "target,";
    }
    if (!has_trigger_)
    {
      cout << "trigger,";
    }
    if (planner_manager_->plan_parameters_.drone_id >= 1 && !has_received_previous_agent_)
    {
      cout << "prev traj,";
    }

    cout << endl;
  }

  std::pair<int, DiffReplanFSM::FsmExecutionState> DiffReplanFSM::GetConsecutiveStateCalls()
  {
    return std::pair<int, FsmExecutionState>(consecutive_call_count_, execution_state_);
  }

  void DiffReplanFSM::SafetyTimerCallback(const ros::TimerEvent &event)
  {
    // check ground height by the way
    if (enable_ground_height_measurement_)
    {
      double height;
      MeasureGroundHeight(height);
    }

    /* --------- collision check data ---------- */
    LocalTrajectoryData *local_trajectory = &planner_manager_->trajectory_container_.local_trajectory_;
    auto map = planner_manager_->grid_map_;
    const double current_time = ros::Time::now().toSec() - local_trajectory->start_time;
    PointsToCheck points_to_check = local_trajectory->points_to_check;

    if (execution_state_ == kWaitTarget || execution_state_ ==  kEmergencyStop || local_trajectory->trajectory_id <= 0)
      return;

    /* ---------- check lost of depth ---------- */
    if (map->GetOdometryDepthTimeout())
    {
      ROS_ERROR("Depth Lost! EMERGENCY_STOP");
      enable_fail_safe_ = false;
      ChangeExecutionState(kEmergencyStop, "SAFETY");
    }

    /* ---------- check trajectory ---------- */
    double temporary_time = current_time; // temporary_time will be changed in the next function!
    int start_piece_index = local_trajectory->trajectory.LocatePieceIndex(temporary_time);

    if (start_piece_index >= (int)points_to_check.size())
    {
      return;
    }
    size_t start_point_index = 0;
    for (; start_piece_index < (int)points_to_check.size(); ++start_piece_index)
    {
      for (start_point_index = 0; start_point_index < points_to_check[start_piece_index].size(); ++start_point_index)
      {
        if (points_to_check[start_piece_index][start_point_index].first > current_time)
        {
          goto find_ij_start;
        }
      }
    }
  find_ij_start:;

    const bool reached_end = ((local_target_point_ - final_goal_).norm() < 1e-2);
    size_t end_piece_index = reached_end ? points_to_check.size() : points_to_check.size() * 3 / 4;
    for (size_t i = start_piece_index; i < end_piece_index; ++i)
    {
      for (size_t j = start_point_index; j < points_to_check[i].size(); ++j)
      {

        double t = points_to_check[i][j].first;
        Eigen::Vector3d p = points_to_check[i][j].second;

        bool dangerous = false;
        dangerous |= map->GetInflatedOccupancy(p);

        for (size_t id = 0; id < planner_manager_->trajectory_container_.swarm_trajectories_.size(); id++)
        {
          if ((planner_manager_->trajectory_container_.swarm_trajectories_.at(id).drone_id != (int)id) ||
              (planner_manager_->trajectory_container_.swarm_trajectories_.at(id).drone_id == planner_manager_->plan_parameters_.drone_id))
          {
            continue;
          }

          double other_trajectory_time = t + (local_trajectory->start_time - planner_manager_->trajectory_container_.swarm_trajectories_.at(id).start_time);
          if (other_trajectory_time > 0 && other_trajectory_time < planner_manager_->trajectory_container_.swarm_trajectories_.at(id).duration)
          {
            Eigen::Vector3d predicted_swarm_position = planner_manager_->trajectory_container_.swarm_trajectories_.at(id).trajectory.GetPosition(other_trajectory_time);
            double dist = (p - predicted_swarm_position).norm();
            double allowed_distance = planner_manager_->GetSwarmClearance() + planner_manager_->trajectory_container_.swarm_trajectories_.at(id).desired_clearance;
            if (dist < allowed_distance)
            {
              ROS_WARN("swarm distance between drone %d and drone %d is %f, too close!",
                       planner_manager_->plan_parameters_.drone_id, (int)id, dist);
              dangerous = true;
              break;
            }
          }
        }

        if (dangerous)
        {
          /* Handle the collided case immediately */
          if (PlanFromLocalTrajectory()) // Make a chance
          {
            ROS_INFO("Plan success when detect collision. %f", t / local_trajectory->duration);
            ChangeExecutionState(kExecuteTrajectory, "SAFETY");
            return;
          }
          else
          {
            if (t - current_time < emergency_time_) // 0.8s of emergency time
            {
              ROS_WARN("Emergency stop! time=%f", t - current_time);
              ChangeExecutionState(kEmergencyStop, "SAFETY");
            }
            else
            {
              ROS_WARN("current traj in collision, replan.");
              ChangeExecutionState(kReplanTrajectory, "SAFETY");
            }
            return;
          }
          break;
        }
      }
      start_point_index = 0;
    }
  }

  bool DiffReplanFSM::CallEmergencyStop(Eigen::Vector3d stop_position)
  {

    planner_manager_->EmergencyStop(stop_position);

    traj_utils::PolyTraj polynomial_message;
    traj_utils::MINCOTraj minco_message;
    ConvertPolynomialTrajectoryToRosMessage(polynomial_message, minco_message);
    polynomial_trajectory_publisher_.publish(polynomial_message);
    broadcast_trajectory_publisher_.publish(minco_message);
    return true;
  }

  bool DiffReplanFSM::CallReboundReplan(bool use_polynomial_initialization, bool use_random_polynomial_trajectory)
  {
    if (modify_final_goal_ && ModifyInCollisionFinalGoal())
    {
      ROS_WARN("Successfully modified final_goal in callReboundReplan !!!");
    }
    planner_manager_->GetLocalTarget(
        planning_horizon_, start_point_, final_goal_,
        local_target_point_, local_target_velocity_,
        touch_goal_);

    bool planning_succeeded = planner_manager_->ReboundReplan(
        start_point_, start_velocity_, start_acceleration_,
        local_target_point_, local_target_velocity_,
        (has_new_target_ || use_polynomial_initialization),
        use_random_polynomial_trajectory, touch_goal_);

    has_new_target_ = false;

    if (planning_succeeded)
    {
      traj_utils::PolyTraj polynomial_message;
      traj_utils::MINCOTraj minco_message;
      ConvertPolynomialTrajectoryToRosMessage(polynomial_message, minco_message);
      polynomial_trajectory_publisher_.publish(polynomial_message);
      broadcast_trajectory_publisher_.publish(minco_message);
    }

    return planning_succeeded;
  }

  bool DiffReplanFSM::PlanFromGlobalTrajectory(const int trial_count /*=1*/) //zx-todo
  {

    start_point_ = odometry_position_;
    start_velocity_ = odometry_velocity_;
    start_acceleration_.setZero();

    bool use_random_polynomial_initialization;
    if (GetConsecutiveStateCalls().first == 1)
      use_random_polynomial_initialization = false;
    else
      use_random_polynomial_initialization = true;

    for (int i = 0; i < trial_count; i++)
    {
      if (CallReboundReplan(true, use_random_polynomial_initialization))
      {
        return true;
      }
    }
    return false;
  }

  bool DiffReplanFSM::PlanFromLocalTrajectory(const int trial_count /*=1*/)
  {

    LocalTrajectoryData *local_trajectory = &planner_manager_->trajectory_container_.local_trajectory_;
    double current_time = ros::Time::now().toSec() - local_trajectory->start_time;

    start_point_ = local_trajectory->trajectory.GetPosition(current_time);
    start_velocity_ = local_trajectory->trajectory.GetVelocity(current_time);
    start_acceleration_ = local_trajectory->trajectory.GetAcceleration(current_time);

    bool success = CallReboundReplan(false, false);

    if (!success)
    {
      success = CallReboundReplan(true, false);
      if (!success)
      {
        for (int i = 0; i < trial_count; i++)
        {
          success = CallReboundReplan(true, true);
          if (success)
            break;
        }
        if (!success)
        {
          return false;
        }
      }
    }

    return true;
  }

  bool DiffReplanFSM::PlanNextWaypoint(const Eigen::Vector3d next_waypoint,
                                       bool trigger_replan)
  {
    bool success = false;
    std::vector<Eigen::Vector3d> single_waypoint;
    single_waypoint.push_back(next_waypoint);
    success = planner_manager_->PlanGlobalTrajectoryWaypoints(
        odometry_position_, odometry_velocity_, Eigen::Vector3d::Zero(),
        single_waypoint, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    // visualization_->DisplayGoalPoint(next_waypoint, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);
    if (success)
    {
      final_goal_ = next_waypoint;
      /*** display ***/
      constexpr double kVisualizationTimeStep = 0.1;
      int end_piece_index = floor(planner_manager_->trajectory_container_.global_trajectory_.duration / kVisualizationTimeStep);
      vector<Eigen::Vector3d> global_trajectory_points(end_piece_index);
      for (int i = 0; i < end_piece_index; i++)
      {
        global_trajectory_points[i] = planner_manager_->trajectory_container_.global_trajectory_.trajectory.GetPosition(i * kVisualizationTimeStep);
      }
      has_target_ = true;
      has_new_target_ = true;
      /*** FSM ***/
      if (execution_state_ != kWaitTarget && trigger_replan && execution_state_ != kEmergencyStop)
      {
        ros::Time start_time = ros::Time::now();
        ros::Duration timeout(0.5); 
        while (execution_state_ != kExecuteTrajectory)
        {
          ros::spinOnce();
          ros::Duration(0.001).sleep();
          if (ros::Time::now() - start_time > timeout)
          {
            ROS_WARN("Timeout waiting for state to change to EXEC_TRAJ.");
            return false; 
          }
        }
        ChangeExecutionState(kReplanTrajectory, "TRIG");
      }
      else if(execution_state_ == kEmergencyStop)
      {
        return true;
      }
      // visualization_->DisplayGoalPoint(final_goal_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
       visualization_->DisplayGlobalPathList(global_trajectory_points, 0.1, 0);
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }
    return success;
  }

  bool DiffReplanFSM::ModifyInCollisionFinalGoal()
  {
    if (planner_manager_->grid_map_->GetInflatedOccupancy(final_goal_))
    {
      Eigen::Vector3d original_goal = final_goal_;
      double time_step = planner_manager_->grid_map_->GetResolution() / planner_manager_->plan_parameters_.max_velocity;
      for (double t = planner_manager_->trajectory_container_.global_trajectory_.duration; t > 0; t -= time_step)
      {
        Eigen::Vector3d pt = planner_manager_->trajectory_container_.global_trajectory_.trajectory.GetPosition(t);
        if (!planner_manager_->grid_map_->GetInflatedOccupancy(pt))
        {
          for (int i = 6; i > 0; i--)
          {
            if (t - i * time_step > 0)
            {
              Eigen::Vector3d temporary_point = planner_manager_->trajectory_container_.global_trajectory_.trajectory.GetPosition(t - i * time_step);
              if (!planner_manager_->grid_map_->GetInflatedOccupancy(temporary_point))
              {
                pt = temporary_point;
                break;
              }
            }
          }
          if (PlanNextWaypoint(pt, false)) // final_goal_=pt inside if success
          {
            ROS_INFO("Current in-collision waypoint (%.3f, %.3f %.3f) has been modified to (%.3f, %.3f %.3f)",
                     original_goal(0), original_goal(1), original_goal(2), final_goal_(0), final_goal_(1), final_goal_(2));
            return true;
          }
        }

        if (t <= time_step)
        {
          ROS_ERROR("Can't find any collision-free point on global traj.");
        }
      }
    }

    return false;
  }

  void DiffReplanFSM::WaypointCallback(const geometry_msgs::PoseStampedPtr &message)
  {
    Eigen::Vector3d end_waypoint(message->pose.position.x, message->pose.position.y, message->pose.position.z);
    if (planner_manager_->grid_map_->GetInflatedOccupancy(end_waypoint) == -1)
    {
      ROS_WARN("The goal is outside the safe fence, ignore this goal!");
      return;
    }
    ROS_INFO("Received goal: %f, %f, %f", end_waypoint(0), end_waypoint(1), end_waypoint(2));
    if (PlanNextWaypoint(end_waypoint, true))
    {
      last_target_change_time_ = ros::Time::now().toSec();
      has_trigger_ = true;
    }
  }

  void DiffReplanFSM::ReadGivenWaypointsAndPlan()
  {
    if (waypoint_count_ <= 0)
    {
      ROS_ERROR("Wrong waypoint_num_ = %d", waypoint_count_);
      return;
    }

    waypoint_positions_.resize(waypoint_count_);
    for (int i = 0; i < waypoint_count_; i++)
    {
      waypoint_positions_[i](0) = waypoints_[i][0];
      waypoint_positions_[i](1) = waypoints_[i][1];
      waypoint_positions_[i](2) = waypoints_[i][2];
    }

    for (size_t i = 0; i < (size_t)waypoint_count_; i++)
    {
      visualization_->DisplayGoalPoint(waypoint_positions_[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
      ros::Duration(0.001).sleep();
    }

    // plan first global waypoint
    waypoint_index_ = 0;
    PlanNextWaypoint(waypoint_positions_[waypoint_index_], true);
  }

  void DiffReplanFSM::MandatoryStopCallback(const std_msgs::Empty &message)
  {
    mandatory_stop_ = true;
    ROS_ERROR("Received a mandatory stop command!");
    ChangeExecutionState(kEmergencyStop, "Mandatory Stop");
    enable_fail_safe_ = false;
  }

  void DiffReplanFSM::OdometryCallback(const nav_msgs::OdometryConstPtr &message)
  {
    odometry_position_(0) = message->pose.pose.position.x;
    odometry_position_(1) = message->pose.pose.position.y;
    odometry_position_(2) = message->pose.pose.position.z;

    odometry_velocity_(0) = message->twist.twist.linear.x;
    odometry_velocity_(1) = message->twist.twist.linear.y;
    odometry_velocity_(2) = message->twist.twist.linear.z;

    has_odometry_ = true;
  }

  void DiffReplanFSM::TriggerCallback(const geometry_msgs::PoseStampedPtr &message)
  {
    has_trigger_ = true;
    cout << "Triggered!" << endl;
  }

  void DiffReplanFSM::ReceiveBroadcastMincoTrajectoryCallback(const traj_utils::MINCOTrajConstPtr &message)
  {
    const size_t received_drone_id = (size_t)message->drone_id;
    if ((int)received_drone_id == planner_manager_->plan_parameters_.drone_id) // myself
      return;

    if (message->drone_id < 0)
    {
      ROS_ERROR("drone_id < 0 is not allowed in a swarm system!");
      return;
    }
    if (message->order != 5)
    {
      ROS_ERROR("Only support trajectory order equals 5 now!");
      return;
    }
    if (message->duration.size() != (message->inner_x.size() + 1))
    {
      ROS_ERROR("WRONG trajectory parameters.");
      return;
    }
    if (planner_manager_->trajectory_container_.swarm_trajectories_.size() > received_drone_id &&
        planner_manager_->trajectory_container_.swarm_trajectories_[received_drone_id].drone_id == (int)received_drone_id &&
        message->start_time.toSec() - planner_manager_->trajectory_container_.swarm_trajectories_[received_drone_id].start_time <= 0)
    {
      ROS_WARN("Received drone %d's trajectory out of order or duplicated, abandon it.", (int)received_drone_id);
      return;
    }

    ros::Time current_ros_time = ros::Time::now();
    if (abs((current_ros_time - message->start_time).toSec()) > 0.25)
    {

      if (abs((current_ros_time - message->start_time).toSec()) < 10.0) // 10 seconds offset, more likely to be caused by unsynced system time.
      {
        ROS_WARN("Time stamp diff: Local - Remote Agent %d = %fs",
                 message->drone_id, (current_ros_time - message->start_time).toSec());
      }
      else
      {
        ROS_ERROR("Time stamp diff: Local - Remote Agent %d = %fs, swarm time seems not synchronized, abandon!",
                  message->drone_id, (current_ros_time - message->start_time).toSec());
        return;
      }
    }

    /* Fill up the buffer */
    if (planner_manager_->trajectory_container_.swarm_trajectories_.size() <= received_drone_id)
    {
      for (size_t i = planner_manager_->trajectory_container_.swarm_trajectories_.size(); i <= received_drone_id; i++)
      {
        LocalTrajectoryData blank;
        blank.drone_id = -1;
        blank.start_time = 0.0;
        planner_manager_->trajectory_container_.swarm_trajectories_.push_back(blank);
      }
    }

    if ( message->start_time.toSec() <= planner_manager_->trajectory_container_.swarm_trajectories_[received_drone_id].start_time ) // This must be called after buffer fill-up
    {
      ROS_WARN("Old traj received, ignored.");
      return;
    }

    /* Parse and store data */

    int piece_count = message->duration.size();
    Eigen::Matrix<double, 3, 3> head_state, tail_state;
    head_state << message->start_p[0], message->start_v[0], message->start_a[0],
        message->start_p[1], message->start_v[1], message->start_a[1],
        message->start_p[2], message->start_v[2], message->start_a[2];
    tail_state << message->end_p[0], message->end_v[0], message->end_a[0],
        message->end_p[1], message->end_v[1], message->end_a[1],
        message->end_p[2], message->end_v[2], message->end_a[2];
    Eigen::MatrixXd inner_points(3, piece_count - 1);
    Eigen::VectorXd durations(piece_count);
    for (int i = 0; i < piece_count - 1; i++)
      inner_points.col(i) << message->inner_x[i], message->inner_y[i], message->inner_z[i];
    for (int i = 0; i < piece_count; i++)
      durations(i) = message->duration[i];
    poly_traj::MinJerkOpt jerk_optimizer;
    jerk_optimizer.Reset(head_state, tail_state, piece_count);
    jerk_optimizer.Generate(inner_points, durations);

    /* Ignore the trajectories that are far away */
    Eigen::MatrixXd check_control_points = jerk_optimizer.GetInitialConstraintPoints(5); // K = 5, such accuracy is sufficient
    bool far_away = true;
    for (int i = 0; i < check_control_points.cols(); ++i)
    {
      if ((check_control_points.col(i) - odometry_position_).norm() < planner_manager_->plan_parameters_.planning_horizon * 4 / 3) // close to me that can not be ignored
      {
        far_away = false;
        break;
      }
    }
    if (!far_away || !has_received_previous_agent_) // Accept a far traj if no previous agent received
    {
      poly_traj::Trajectory trajectory = jerk_optimizer.GetTrajectory();
      planner_manager_->trajectory_container_.swarm_trajectories_[received_drone_id].trajectory = trajectory;
      planner_manager_->trajectory_container_.swarm_trajectories_[received_drone_id].drone_id = received_drone_id;
      planner_manager_->trajectory_container_.swarm_trajectories_[received_drone_id].trajectory_id = message->traj_id;
      planner_manager_->trajectory_container_.swarm_trajectories_[received_drone_id].start_time = message->start_time.toSec();
      planner_manager_->trajectory_container_.swarm_trajectories_[received_drone_id].duration = trajectory.GetTotalDuration();
      planner_manager_->trajectory_container_.swarm_trajectories_[received_drone_id].start_position = trajectory.GetPosition(0.0);
      planner_manager_->trajectory_container_.swarm_trajectories_[received_drone_id].desired_clearance = message->des_clearance;

      /* Check Collision */
      if (planner_manager_->CheckCollision(received_drone_id))
      {
        ChangeExecutionState(kReplanTrajectory, "SWARM_CHECK");
      }

      /* Check if receive agents have lower drone id */
      if (!has_received_previous_agent_)
      {
        if ((int)planner_manager_->trajectory_container_.swarm_trajectories_.size() >= planner_manager_->plan_parameters_.drone_id)
        {
          for (int i = 0; i < planner_manager_->plan_parameters_.drone_id; ++i)
          {
            if (planner_manager_->trajectory_container_.swarm_trajectories_[i].drone_id != i)
            {
              break;
            }

            has_received_previous_agent_ = true;
          }
        }
      }
    }
    else
    {
      planner_manager_->trajectory_container_.swarm_trajectories_[received_drone_id].drone_id = -1; // Means this trajectory is invalid
    }
  }

  void DiffReplanFSM::ConvertPolynomialTrajectoryToRosMessage(traj_utils::PolyTraj &polynomial_message, traj_utils::MINCOTraj &minco_message)
  {

    auto data = &planner_manager_->trajectory_container_.local_trajectory_;
    Eigen::VectorXd durations = data->trajectory.GetDurations();
    int piece_count = data->trajectory.GetPieceCount();

    polynomial_message.drone_id = planner_manager_->plan_parameters_.drone_id;
    polynomial_message.traj_id = data->trajectory_id;
    polynomial_message.start_time = ros::Time(data->start_time);
    polynomial_message.order = 5; // todo, only support order = 5 now.
    polynomial_message.duration.resize(piece_count);
    polynomial_message.coef_x.resize(6 * piece_count);
    polynomial_message.coef_y.resize(6 * piece_count);
    polynomial_message.coef_z.resize(6 * piece_count);
    for (int i = 0; i < piece_count; ++i)
    {
      polynomial_message.duration[i] = durations(i);

      poly_traj::CoefficientMatrix coefficient_matrix = data->trajectory.GetPiece(i).GetCoefficientMatrix();
      int coefficient_offset = i * 6;
      for (int j = 0; j < 6; j++)
      {
        polynomial_message.coef_x[coefficient_offset + j] = coefficient_matrix(0, j);
        polynomial_message.coef_y[coefficient_offset + j] = coefficient_matrix(1, j);
        polynomial_message.coef_z[coefficient_offset + j] = coefficient_matrix(2, j);
      }
    }

    minco_message.drone_id = planner_manager_->plan_parameters_.drone_id;
    minco_message.traj_id = data->trajectory_id;
    minco_message.start_time = ros::Time(data->start_time);
    minco_message.order = 5; // todo, only support order = 5 now.
    minco_message.duration.resize(piece_count);
    minco_message.des_clearance = planner_manager_->GetSwarmClearance();
    Eigen::Vector3d vector;
    vector = data->trajectory.GetPosition(0);
    minco_message.start_p[0] = vector(0), minco_message.start_p[1] = vector(1), minco_message.start_p[2] = vector(2);
    vector = data->trajectory.GetVelocity(0);
    minco_message.start_v[0] = vector(0), minco_message.start_v[1] = vector(1), minco_message.start_v[2] = vector(2);
    vector = data->trajectory.GetAcceleration(0);
    minco_message.start_a[0] = vector(0), minco_message.start_a[1] = vector(1), minco_message.start_a[2] = vector(2);
    vector = data->trajectory.GetPosition(data->duration);
    minco_message.end_p[0] = vector(0), minco_message.end_p[1] = vector(1), minco_message.end_p[2] = vector(2);
    vector = data->trajectory.GetVelocity(data->duration);
    minco_message.end_v[0] = vector(0), minco_message.end_v[1] = vector(1), minco_message.end_v[2] = vector(2);
    vector = data->trajectory.GetAcceleration(data->duration);
    minco_message.end_a[0] = vector(0), minco_message.end_a[1] = vector(1), minco_message.end_a[2] = vector(2);
    minco_message.inner_x.resize(piece_count - 1);
    minco_message.inner_y.resize(piece_count - 1);
    minco_message.inner_z.resize(piece_count - 1);
    Eigen::MatrixXd position = data->trajectory.GetPositions();
    for (int i = 0; i < piece_count - 1; i++)
    {
      minco_message.inner_x[i] = position(0, i + 1);
      minco_message.inner_y[i] = position(1, i + 1);
      minco_message.inner_z[i] = position(2, i + 1);
    }
    for (int i = 0; i < piece_count; i++)
      minco_message.duration[i] = durations[i];
  }

  bool DiffReplanFSM::MeasureGroundHeight(double &height)
  {
    if (planner_manager_->trajectory_container_.local_trajectory_.points_to_check.size() < 3) // means planning have not started
    {
      return false;
    }

    auto traj = &planner_manager_->trajectory_container_.local_trajectory_;
    auto map = planner_manager_->grid_map_;
    ros::Time current_ros_time = ros::Time::now();

    double forward_time = 2.0 / planner_manager_->plan_parameters_.max_velocity; //2.0m
    double trajectory_time = (current_ros_time.toSec() - traj->start_time) + forward_time;
    if (trajectory_time <= traj->duration)
    {
      Eigen::Vector3d forward_position = traj->trajectory.GetPosition(trajectory_time);

      double resolution = map->GetResolution();
      for (;; forward_position(2) -= resolution)
      {
        int ret = map->GetOccupancy(forward_position);
        if (ret == -1) // reach map bottom
        {
          return false;
        }
        if (ret == 1) // reach the ground
        {
          height = forward_position(2);

          std_msgs::Float64 height_msg;
          height_msg.data = height;
          ground_height_publisher_.publish(height_msg);

          return true;
        }
      }
    }

    return false;
  }
  Eigen::Vector3d DiffReplanFSM::ProjectPointToLineSegment(
      const Eigen::Vector3d &line_start, const Eigen::Vector3d &line_end,
      const Eigen::Vector3d &point)
  {
      double t = 0.0;
      Eigen::Vector3d line_segment = line_end - line_start;
      double squared_length = line_segment.squaredNorm();
      if (squared_length < 1e-8)
      {
        t = 0.0;
        return line_start;
      }
      t = (point - line_start).dot(line_segment) / squared_length;
      if (t < 0.0)
      {
        t = 0.0;
        return line_start;
      }
      else if (t > 1.0)
      {
        t = 1.0;
        return line_end;
      }
      return line_start + t * line_segment;
  }
} // namespace diff_planner
