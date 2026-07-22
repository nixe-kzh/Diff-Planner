// #include <fstream>
#include <plan_manage/planner_manager.h>
#include <thread>
#include "visualization_msgs/Marker.h" // zx-todo

namespace diff_planner
{

  // SECTION interfaces for setup and query

  DiffPlannerManager::DiffPlannerManager() {}

  DiffPlannerManager::~DiffPlannerManager() { std::cout << "des manager" << std::endl; }

  void DiffPlannerManager::InitPlanModules(
      ros::NodeHandle &node_handle,
      PlanningVisualization::Ptr visualization)
  {
    /* read algorithm parameters */

    node_handle.param("manager/max_vel", plan_parameters_.max_velocity, -1.0);
    node_handle.param("manager/max_acc", plan_parameters_.max_acceleration, -1.0);
    node_handle.param("manager/feasibility_tolerance", plan_parameters_.feasibility_tolerance, 0.0);
    node_handle.param("manager/trajectory_piece_length", plan_parameters_.trajectory_piece_length, -1.0);
    node_handle.param("manager/planning_horizon", plan_parameters_.planning_horizon, 5.0);
    node_handle.param("manager/use_multitopology_trajs", plan_parameters_.use_multi_topology_trajectories, false);
    node_handle.param("manager/drone_id", plan_parameters_.drone_id, -1);

    grid_map_.reset(new GridMap);
    grid_map_->InitMap(node_handle);

    trajectory_optimizer_.reset(new PolyTrajOptimizer);
    trajectory_optimizer_->SetParameters(node_handle);
    trajectory_optimizer_->SetEnvironment(grid_map_);

    visualization_ = visualization;

    trajectory_optimizer_->SetSwarmTrajectories(&trajectory_container_.swarm_trajectories_);
    trajectory_optimizer_->SetDroneId(plan_parameters_.drone_id);
  }

  bool DiffPlannerManager::ReboundReplan(
      const Eigen::Vector3d &start_point, const Eigen::Vector3d &start_velocity,
      const Eigen::Vector3d &start_acceleration, const Eigen::Vector3d &local_target_point,
      const Eigen::Vector3d &local_target_velocity, const bool use_polynomial_initialization,
      const bool use_random_polynomial_trajectory, const bool touch_goal)
  {
    ros::Time start_time = ros::Time::now();
    ros::Duration initialization_duration, optimization_duration;

    static int replan_count = 0;
    cout << "\033[47;30m\n[" << start_time << "] Drone " << plan_parameters_.drone_id << " Replan " << replan_count++ << "\033[0m" << endl;
    // cout.precision(3);
    // cout << "start: " << start_point.transpose() << ", " << start_velocity.transpose() << "\ngoal:" << local_target_point.transpose() << ", " << local_target_velocity.transpose()
    //      << endl;
    // if ((start_point - local_target_point).norm() < 0.2)
    //   cout << "Close to goal" << endl;

    /*** STEP 1: INIT ***/
    trajectory_optimizer_->SetTouchGoal(touch_goal);
    double piece_duration = plan_parameters_.trajectory_piece_length / plan_parameters_.max_velocity;

    poly_traj::MinJerkOpt initial_jerk_optimizer;
    if (!ComputeInitialState(start_point, start_velocity, start_acceleration, local_target_point, local_target_velocity,
                          use_polynomial_initialization, use_random_polynomial_trajectory, piece_duration, initial_jerk_optimizer))
    {
      return false;
    }

    Eigen::MatrixXd constraint_points = initial_jerk_optimizer.GetInitialConstraintPoints(trajectory_optimizer_->GetConstraintPointsPerPiece());
    vector<std::pair<int, int>> segments;
    if (trajectory_optimizer_->FinelyCheckAndSetConstraintPoints(segments, initial_jerk_optimizer, true) == PolyTrajOptimizer::CheckResult::kError)
    {
      return false;
    }

    initialization_duration = ros::Time::now() - start_time;

    std::vector<Eigen::Vector3d> path_points;
    for (int i = 0; i < constraint_points.cols(); ++i)
      path_points.push_back(constraint_points.col(i));
    visualization_->DisplayInitialPathList(path_points, 0.2, 0);

    start_time = ros::Time::now();

    /*** STEP 2: OPTIMIZE ***/
    bool optimization_succeeded = false;
    vector<vector<Eigen::Vector3d>> visualized_trajectories;
    poly_traj::MinJerkOpt best_jerk_optimizer;

    // ROS_ERROR("BBBB");

    if (plan_parameters_.use_multi_topology_trajectories)
    {
      std::vector<ConstraintPoints> trajectory_candidates = trajectory_optimizer_->GenerateDistinctiveTrajectories(segments);
      Eigen::VectorXi optimization_results = Eigen::VectorXi::Zero(trajectory_candidates.size());
      poly_traj::Trajectory initial_trajectory = initial_jerk_optimizer.GetTrajectory();
      int piece_count = initial_trajectory.GetPieceCount();
      Eigen::MatrixXd all_positions = initial_trajectory.GetPositions();
      Eigen::MatrixXd inner_points = all_positions.block(0, 1, 3, piece_count - 1);
      Eigen::Matrix<double, 3, 3> head_state, tail_state;
      head_state << initial_trajectory.GetJunctionPosition(0), initial_trajectory.GetJunctionVelocity(0), initial_trajectory.GetJunctionAcceleration(0);
      tail_state << initial_trajectory.GetJunctionPosition(piece_count), initial_trajectory.GetJunctionVelocity(piece_count), initial_trajectory.GetJunctionAcceleration(piece_count);
      double final_cost, minimum_cost = 999999.0;

      for (int i = trajectory_candidates.size() - 1; i >= 0; i--)
      {
        trajectory_optimizer_->SetConstraintPoints(trajectory_candidates[i]);
        trajectory_optimizer_->SetUseMultiTopologyTrajectories(true);
        if (trajectory_optimizer_->OptimizeTrajectory(head_state, tail_state,
                                               inner_points, initial_trajectory.GetDurations(), final_cost))
        {
          optimization_results[i] = true;

          if (final_cost < minimum_cost)
          {
            minimum_cost = final_cost;
            best_jerk_optimizer = trajectory_optimizer_->GetMinimumJerkOptimizer();
            optimization_succeeded = true;
          }

          // visualization
          Eigen::MatrixXd temporary_control_points = trajectory_optimizer_->GetMinimumJerkOptimizer().GetInitialConstraintPoints(trajectory_optimizer_->GetConstraintPointsPerPiece());
          std::vector<Eigen::Vector3d> path_points;
          for (int j = 0; j < temporary_control_points.cols(); j++)
          {
            path_points.push_back(temporary_control_points.col(j));
          }
          visualized_trajectories.push_back(path_points);
        }
      }

      optimization_duration = ros::Time::now() - start_time;

      if (trajectory_candidates.size() > 1)
      {
        cout << "\033[1;33m"
             << "multi-trajs=" << trajectory_candidates.size() << ",\033[1;0m"
             << " Success:fail=" << optimization_results.sum() << ":" << optimization_results.size() - optimization_results.sum() << endl;
      }

      visualization_->DisplayMultiOptimalPathList(visualized_trajectories, 0.1); // This visuallization will take up several milliseconds.
    }
    else
    {
      poly_traj::Trajectory initial_trajectory = initial_jerk_optimizer.GetTrajectory();
      int piece_count = initial_trajectory.GetPieceCount();
      Eigen::MatrixXd all_positions = initial_trajectory.GetPositions();
      Eigen::MatrixXd inner_points = all_positions.block(0, 1, 3, piece_count - 1);
      Eigen::Matrix<double, 3, 3> head_state, tail_state;
      head_state << initial_trajectory.GetJunctionPosition(0), initial_trajectory.GetJunctionVelocity(0), initial_trajectory.GetJunctionAcceleration(0);
      tail_state << initial_trajectory.GetJunctionPosition(piece_count), initial_trajectory.GetJunctionVelocity(piece_count), initial_trajectory.GetJunctionAcceleration(piece_count);
      double final_cost;
      optimization_succeeded = trajectory_optimizer_->OptimizeTrajectory(head_state, tail_state,
                                                        inner_points, initial_trajectory.GetDurations(), final_cost);
      best_jerk_optimizer = trajectory_optimizer_->GetMinimumJerkOptimizer();

      optimization_duration = ros::Time::now() - start_time;
    }

    /*** STEP 3: Store and display results ***/
    cout << "Success=" << (optimization_succeeded ? "yes" : "no") << endl;
    if (optimization_succeeded)
    {
      static double total_time = 0;
      static int success_count = 0;
      total_time += (initialization_duration + optimization_duration).toSec();
      success_count++;
      printf("Time:\033[42m%.3fms,\033[0m init:%.3fms, optimize:%.3fms, avg=%.3fms\n",
             (initialization_duration + optimization_duration).toSec() * 1000, initialization_duration.toSec() * 1000, optimization_duration.toSec() * 1000, total_time / success_count * 1000);
      // cout << "total time:\033[42m" << (initialization_duration + optimization_duration).toSec()
      //      << "\033[0m,init:" << initialization_duration.toSec()
      //      << ",optimize:" << optimization_duration.toSec()
      //      << ",avg_time=" << total_time / success_count << endl;

      SetLocalTrajectoryFromOptimizer(best_jerk_optimizer, touch_goal);
      constraint_points = best_jerk_optimizer.GetInitialConstraintPoints(trajectory_optimizer_->GetConstraintPointsPerPiece());
      visualization_->DisplayOptimalList(constraint_points, 0);

      continuous_failure_count_ = 0;
    }
    else
    {
      constraint_points = trajectory_optimizer_->GetMinimumJerkOptimizer().GetInitialConstraintPoints(trajectory_optimizer_->GetConstraintPointsPerPiece());
      visualization_->DisplayFailedList(constraint_points, 0);

      continuous_failure_count_++;
    }

    return optimization_succeeded;
  }

  bool DiffPlannerManager::ComputeInitialState(
      const Eigen::Vector3d &start_point, const Eigen::Vector3d &start_velocity, const Eigen::Vector3d &start_acceleration,
      const Eigen::Vector3d &local_target_point, const Eigen::Vector3d &local_target_velocity,
      const bool use_polynomial_initialization,
      const bool use_random_polynomial_trajectory, const double piece_duration,
      poly_traj::MinJerkOpt &initial_jerk_optimizer)
  {

    static bool first_call = true;

    if (first_call || use_polynomial_initialization) /*** case 1: polynomial initialization ***/
    {
      first_call = false;

      /* basic params */
      Eigen::Matrix3d head_state, tail_state;
      Eigen::MatrixXd inner_points;
      Eigen::VectorXd piece_durations;
      int piece_count;
      constexpr double kInitialTotalDuration = 2.0;
      head_state << start_point, start_velocity, start_acceleration;
      tail_state << local_target_point, local_target_velocity, Eigen::Vector3d::Zero();

      /* determined or random inner point */
      if (!use_random_polynomial_trajectory)
      {
        if (inner_points.cols() != 0)
        {
          ROS_ERROR("innerPs.cols() != 0");
        }

        piece_count = 1;
        piece_durations.resize(1);
        piece_durations(0) = kInitialTotalDuration;
      }
      else
      {
        Eigen::Vector3d horizontal_direction = ((start_point - local_target_point).cross(Eigen::Vector3d(0, 0, 1))).normalized();
        Eigen::Vector3d vertical_direction = ((start_point - local_target_point).cross(horizontal_direction)).normalized();
        inner_points.resize(3, 1);
        inner_points = (start_point + local_target_point) / 2 +
                  (((double)rand()) / RAND_MAX - 0.5) *
                      (start_point - local_target_point).norm() *
                      horizontal_direction * 0.8 * (-0.978 / (continuous_failure_count_ + 0.989) + 0.989) +
                  (((double)rand()) / RAND_MAX - 0.5) *
                      (start_point - local_target_point).norm() *
                      vertical_direction * 0.4 * (-0.978 / (continuous_failure_count_ + 0.989) + 0.989);

        piece_count = 2;
        piece_durations.resize(2);
        piece_durations = Eigen::Vector2d(kInitialTotalDuration / 2, kInitialTotalDuration / 2);
      }

      /* generate the init of init trajectory */
      initial_jerk_optimizer.Reset(head_state, tail_state, piece_count);
      initial_jerk_optimizer.Generate(inner_points, piece_durations);
      poly_traj::Trajectory initial_trajectory = initial_jerk_optimizer.GetTrajectory();

      /* generate the real init trajectory */
      piece_count = round((head_state.col(0) - tail_state.col(0)).norm() / plan_parameters_.trajectory_piece_length);
      if (piece_count < 2)
        piece_count = 2;
      double duration_per_piece = kInitialTotalDuration / (double)piece_count;
      piece_durations.resize(piece_count);
      piece_durations = Eigen::VectorXd::Constant(piece_count, piece_duration);
      inner_points.resize(3, piece_count - 1);
      int id = 0;
      double initial_sample_time = duration_per_piece, final_sample_time = kInitialTotalDuration - duration_per_piece / 2;
      for (double t = initial_sample_time; t < final_sample_time; t += duration_per_piece)
      {
        inner_points.col(id++) = initial_trajectory.GetPosition(t);
      }
      if (id != piece_count - 1)
      {
        ROS_ERROR("Should not happen! x_x");
        return false;
      }
      initial_jerk_optimizer.Reset(head_state, tail_state, piece_count);
      initial_jerk_optimizer.Generate(inner_points, piece_durations);
    }
    else /*** case 2: initialize from previous optimal trajectory ***/
    {
      if (trajectory_container_.global_trajectory_.previous_local_target_time < 0.0)
      {
        ROS_ERROR("You are initialzing a trajectory from a previous optimal trajectory, but no previous trajectories up to now.");
        return false;
      }

      /* the trajectory time system is a little bit complicated... */
      double elapsed_local_trajectory_time = ros::Time::now().toSec() - trajectory_container_.local_trajectory_.start_time;
      double time_to_local_end = trajectory_container_.local_trajectory_.duration - elapsed_local_trajectory_time;
      if (time_to_local_end < 0)
      {
        ROS_INFO("t_to_lc_end < 0, exit and wait for another call.");
        return false;
      }
      double time_to_local_target = time_to_local_end +
                           (trajectory_container_.global_trajectory_.local_target_time - trajectory_container_.global_trajectory_.previous_local_target_time);
      int piece_count = ceil((start_point - local_target_point).norm() / plan_parameters_.trajectory_piece_length);
      if (piece_count < 2)
        piece_count = 2;

      Eigen::Matrix3d head_state, tail_state;
      Eigen::MatrixXd inner_points(3, piece_count - 1);
      Eigen::VectorXd piece_durations = Eigen::VectorXd::Constant(piece_count, time_to_local_target / piece_count);
      head_state << start_point, start_velocity, start_acceleration;
      tail_state << local_target_point, local_target_velocity, Eigen::Vector3d::Zero();

      double t = piece_durations(0);
      for (int i = 0; i < piece_count - 1; ++i)
      {
        if (t < time_to_local_end)
        {
          inner_points.col(i) = trajectory_container_.local_trajectory_.trajectory.GetPosition(t + elapsed_local_trajectory_time);
        }
        else if (t <= time_to_local_target)
        {
          double global_time = t - time_to_local_end + trajectory_container_.global_trajectory_.previous_local_target_time - trajectory_container_.global_trajectory_.global_start_time;
          inner_points.col(i) = trajectory_container_.global_trajectory_.trajectory.GetPosition(global_time);
        }
        else
        {
          ROS_ERROR("Should not happen! x_x 0x88 t=%.2f, t_to_lc_end=%.2f, t_to_lc_tgt=%.2f", t, time_to_local_end, time_to_local_target);
        }

        t += piece_durations(i + 1);
      }

      initial_jerk_optimizer.Reset(head_state, tail_state, piece_count);
      initial_jerk_optimizer.Generate(inner_points, piece_durations);
    }

    return true;
  }

  void DiffPlannerManager::GetLocalTarget(
      const double planning_horizon, const Eigen::Vector3d &start_point,
      const Eigen::Vector3d &global_end_point, Eigen::Vector3d &local_target_position,
      Eigen::Vector3d &local_target_velocity, bool &touch_goal)
  {
    double t;
    touch_goal = false;

    trajectory_container_.global_trajectory_.previous_local_target_time = trajectory_container_.global_trajectory_.local_target_time;

    double time_step = planning_horizon / 20 / plan_parameters_.max_velocity;
    // double dist_min = 9999, dist_min_t = 0.0;
    for (t = trajectory_container_.global_trajectory_.local_target_time;
         t < (trajectory_container_.global_trajectory_.global_start_time + trajectory_container_.global_trajectory_.duration);
         t += time_step)
    {
      Eigen::Vector3d position_at_time = trajectory_container_.global_trajectory_.trajectory.GetPosition(t - trajectory_container_.global_trajectory_.global_start_time);
      double distance = (position_at_time - start_point).norm();

      if (distance >= planning_horizon)
      {
        local_target_position = position_at_time;
        trajectory_container_.global_trajectory_.local_target_time = t;
        break;
      }
    }

    if ((t - trajectory_container_.global_trajectory_.global_start_time) >= trajectory_container_.global_trajectory_.duration - 1e-5) // Last global point
    {
      local_target_position = global_end_point;
      trajectory_container_.global_trajectory_.local_target_time = trajectory_container_.global_trajectory_.global_start_time + trajectory_container_.global_trajectory_.duration;
      touch_goal = true;
    }

    if ((global_end_point - local_target_position).norm() < (plan_parameters_.max_velocity * plan_parameters_.max_velocity) / (2 * plan_parameters_.max_acceleration))
    {
      local_target_velocity = Eigen::Vector3d::Zero();
    }
    else
    {
      local_target_velocity = trajectory_container_.global_trajectory_.trajectory.GetVelocity(t - trajectory_container_.global_trajectory_.global_start_time);
    }
  }

  bool DiffPlannerManager::SetLocalTrajectoryFromOptimizer(
      const poly_traj::MinJerkOpt &optimizer, const bool touch_goal)
  {
    poly_traj::Trajectory trajectory = optimizer.GetTrajectory();
    Eigen::MatrixXd constraint_points = optimizer.GetInitialConstraintPoints(GetConstraintPointsPerPiece());
    PointsToCheck points_to_check;
    bool result = trajectory_optimizer_->ComputePointsToCheck(
        trajectory, ConstraintPoints::TwoThirdsIndex(constraint_points, touch_goal),
        points_to_check);
    if (result && points_to_check.size() >= 1 && points_to_check.back().size() >= 1)
    {
      trajectory_container_.SetLocalTrajectory(trajectory, points_to_check,
                                               ros::Time::now().toSec());
    }

    return result;
  }

  bool DiffPlannerManager::EmergencyStop(Eigen::Vector3d stop_position)
  {
    auto zero = Eigen::Vector3d::Zero();
    Eigen::Matrix<double, 3, 3> head_state, tail_state;
    head_state << stop_position, zero, zero;
    tail_state = head_state;
    poly_traj::MinJerkOpt stop_optimizer;
    stop_optimizer.Reset(head_state, tail_state, 2);
    stop_optimizer.Generate(stop_position, Eigen::Vector2d(1.0, 1.0));

    SetLocalTrajectoryFromOptimizer(stop_optimizer, false);

    return true;
  }

  bool DiffPlannerManager::CheckCollision(int drone_id)
  {
    if (trajectory_container_.local_trajectory_.start_time < 1e9) // It means my first planning has not started
      return false;
    if (trajectory_container_.swarm_trajectories_[drone_id].drone_id != drone_id) // The trajectory is invalid
      return false;

    double own_trajectory_start_time = trajectory_container_.local_trajectory_.start_time;
    double other_trajectory_start_time = trajectory_container_.swarm_trajectories_[drone_id].start_time;

    double start_time = max(own_trajectory_start_time, other_trajectory_start_time);
    double end_time = min(own_trajectory_start_time + trajectory_container_.local_trajectory_.duration * 2 / 3,
                       other_trajectory_start_time + trajectory_container_.swarm_trajectories_[drone_id].duration);

    for (double t = start_time; t < end_time; t += 0.03)
    {
      if ((trajectory_container_.local_trajectory_.trajectory.GetPosition(t - own_trajectory_start_time) -
           trajectory_container_.swarm_trajectories_[drone_id].trajectory.GetPosition(t - other_trajectory_start_time))
              .norm() < (GetSwarmClearance() + trajectory_container_.swarm_trajectories_[drone_id].desired_clearance) )
      {
        return true;
      }
    }

    return false;
  }

  bool DiffPlannerManager::PlanGlobalTrajectoryWaypoints(
      const Eigen::Vector3d &start_position, const Eigen::Vector3d &start_velocity,
      const Eigen::Vector3d &start_acceleration, const std::vector<Eigen::Vector3d> &waypoints,
      const Eigen::Vector3d &end_velocity, const Eigen::Vector3d &end_acceleration)
  {

    poly_traj::MinJerkOpt global_optimizer;
    Eigen::Matrix<double, 3, 3> head_state, tail_state;
    head_state << start_position, start_velocity, start_acceleration;
    tail_state << waypoints.back(), end_velocity, end_acceleration;
    Eigen::MatrixXd inner_points;

    if (waypoints.size() > 1)
    {

      inner_points.resize(3, waypoints.size() - 1);
      for (int i = 0; i < (int)waypoints.size() - 1; ++i)
      {
        inner_points.col(i) = waypoints[i];
      }
    }
    else
    {
      if (inner_points.size() != 0)
      {
        ROS_ERROR("innerPts.size() != 0");
      }
    }

    global_optimizer.Reset(head_state, tail_state, waypoints.size());

    double desired_velocity = plan_parameters_.max_velocity / 1.5;
    Eigen::VectorXd durations(waypoints.size());

    for (int j = 0; j < 2; ++j)
    {
      for (size_t i = 0; i < waypoints.size(); ++i)
      {
        durations(i) = (i == 0) ? (waypoints[0] - start_position).norm() / desired_velocity
                               : (waypoints[i] - waypoints[i - 1]).norm() / desired_velocity;
      }

      global_optimizer.Generate(inner_points, durations);

      if (global_optimizer.GetTrajectory().GetMaxVelocityRate() < plan_parameters_.max_velocity ||
          start_velocity.norm() > plan_parameters_.max_velocity ||
          end_velocity.norm() > plan_parameters_.max_velocity)
      {
        break;
      }

      if (j == 2)
      {
        ROS_WARN("Global traj MaxVel = %f > set_max_vel", global_optimizer.GetTrajectory().GetMaxVelocityRate());
        cout << "headState=" << endl
             << head_state << endl;
        cout << "tailState=" << endl
             << tail_state << endl;
      }

      desired_velocity /= 1.5;
    }

    auto current_time = ros::Time::now();
    trajectory_container_.SetGlobalTrajectory(global_optimizer.GetTrajectory(), current_time.toSec());

    return true;
  }

} // namespace diff_planner
