#include "optimizer/poly_traj_optimizer.h"

using namespace std;

#define VERBOSE_OUTPUT false
#define PRINTF_COND(STR, ...) \
  if (VERBOSE_OUTPUT)         \
  printf(STR, __VA_ARGS__)

namespace diff_planner
{
  /* main planning API */
  bool PolyTrajOptimizer::OptimizeTrajectory(
      const Eigen::MatrixXd &initial_state, const Eigen::MatrixXd &final_state,
      const Eigen::MatrixXd &initial_inner_points, const Eigen::VectorXd &initial_durations,
      double &final_cost)
  {
    if (initial_inner_points.cols() != (initial_durations.size() - 1))
    {
      ROS_ERROR("initInnerPts.cols() != (initT.size()-1)");
      return false;
    }

    // Preparision 1: Some mise params
    ros::Time t0 = ros::Time::now(), t1, t2;
    int restart_count = 0, rebound_count = 0;
    bool force_return, still_unsafe, success, swarm_too_close;
    multi_topology_data_.initial_obstacles_avoided = false;
    modified_swarm_weight_ = swarm_weight_;

    // Preparision 2: Trajectory related params
    current_time_ = ros::Time::now().toSec();
    piece_count_ = initial_durations.size();
    jerk_optimizer_.Reset(initial_state, final_state, piece_count_);
    variable_count_ = 4 * (piece_count_ - 1) + 1;
    double x_init[variable_count_];
    memcpy(x_init, initial_inner_points.data(), initial_inner_points.size() * sizeof(x_init[0]));
    Eigen::Map<Eigen::VectorXd> virtual_time(x_init + initial_inner_points.size(), initial_durations.size());
    RealTimeToVirtualTime(initial_durations, virtual_time);
    minimum_ellipsoid_distances_squared_.resize(swarm_trajectories_->size());

    // Preparision 3: LBFGS related params
    lbfgs::lbfgs_parameter_t lbfgs_params;
    lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
    lbfgs_params.mem_size = 16;
    lbfgs_params.max_iterations = 200;
    lbfgs_params.min_step = 1e-32;
    // lbfgs_params.abs_curv_cond = 0;
    lbfgs_params.past = 3;
    lbfgs_params.delta = 1.0e-2;
    do
    {
      /* ---------- prepare ---------- */
      iteration_count_ = 0;
      force_return = false;
      force_stop_type_ = kDoNotStop;
      still_unsafe = false;
      success = false;
      swarm_too_close = false;

      /* ---------- optimize ---------- */
      t1 = ros::Time::now();
      int result = lbfgs::lbfgs_optimize(
          variable_count_,
          x_init,
          &final_cost,
          PolyTrajOptimizer::CostFunctionCallback,
          NULL,
          PolyTrajOptimizer::EarlyExitCallback,
          this,
          &lbfgs_params);

      t2 = ros::Time::now();
      double optimization_time_ms = (t2 - t1).toSec() * 1000;
      double total_optimization_time_ms = (t2 - t0).toSec() * 1000;

      /* ---------- get result and check collision ---------- */
      if (result == lbfgs::LBFGS_CONVERGENCE ||
          result == lbfgs::LBFGSERR_MAXIMUMITERATION ||
          result == lbfgs::LBFGS_ALREADY_MINIMIZED ||
          result == lbfgs::LBFGS_STOP)
      {
        force_return = false;

        /* double check: fine collision check */
        std::vector<std::pair<int, int>> unused_segments;
        for (size_t i = 0; i < swarm_trajectories_->size(); ++i)
        {
          swarm_too_close |= minimum_ellipsoid_distances_squared_[i] < pow((swarm_clearance_ + swarm_trajectories_->at(i).desired_clearance) * 1.25, 2);
        }
        if (!swarm_too_close)
        {
          if (!CheckDynamicFeasibility(jerk_optimizer_))
          {
            // If infeasible, flag for another optimization attempt
            still_unsafe = true;
            restart_count++;
            PRINTF_COND("\033[32miter=%d,time(ms)=%5.3f, Dynamic feasibility failed, keep optimizing\n\033[0m", iteration_count_, optimization_time_ms);
          }
          else if (FinelyCheckAndSetConstraintPoints(unused_segments, jerk_optimizer_, false) == CheckResult::kObstacleFree)
          {

            success = true;
            PRINTF_COND("\033[32miter=%d,time(ms)=%5.3f,total_t(ms)=%5.3f,cost=%5.3f\n\033[0m", iteration_count_, optimization_time_ms, total_optimization_time_ms, final_cost);
          }
          else
          {
            // A not-blank return value means collision to obstales
            still_unsafe = true;
            restart_count++;
            PRINTF_COND("\033[32miter=%d,time(ms)=%5.3f, fine check collided, keep optimizing\n\033[0m", iteration_count_, optimization_time_ms);
          }
        }
        else
        {
          PRINTF_COND("Swarm clearance not satisfied, keep optimizing. iter=%d,time(ms)=%5.3f, wei_swarm_mod_=%f\n", iteration_count_, optimization_time_ms, modified_swarm_weight_);
          still_unsafe = true;
          restart_count++;
          modified_swarm_weight_ *= 2;
        }
      }
      else if (result == lbfgs::LBFGSERR_CANCELED)
      {
        force_return = true;
        rebound_count++;
        PRINTF_COND("iter=%d, time(ms)=%f, rebound\n", iteration_count_, optimization_time_ms);
      }
      else
      {
        PRINTF_COND("iter=%d, time(ms)=%f, error\n", iteration_count_, optimization_time_ms);
        ROS_WARN_COND(VERBOSE_OUTPUT, "Solver error. Return = %d, %s. Skip this planning.", result, lbfgs::lbfgs_strerror(result));
      }

    } while ((still_unsafe && restart_count < 3) ||
             (force_return && force_stop_type_ == kStopForRebound && rebound_count <= 20));

    return success;
  }
  bool PolyTrajOptimizer::CheckDynamicFeasibility(const poly_traj::MinJerkOpt &trajectory_optimizer)
  {
    poly_traj::Trajectory traj = trajectory_optimizer.GetTrajectory();
    Eigen::VectorXd durations = traj.GetDurations();
    const double resolution = grid_map_->GetResolution();
    double time_step = min(resolution / max_velocity_, durations.minCoeff() / max(constraint_points_per_piece_, 1) / 1.5);
    double trajectory_duration = traj.GetTotalDuration();

    // Iterate through the trajectory duration with the specified time step
    for (double t = 0.0; t < trajectory_duration; t += time_step)
    {
      // Check velocity constraint
      Eigen::Vector3d vel = traj.GetVelocity(t);
      if (vel.norm() > max_velocity_+ velocity_tolerance_)
      {
        ROS_WARN_STREAM("Dynamic feasibility check failed: velocity limit exceeded at t="
                        << t << ", |v|=" << vel.norm() << " > " << max_velocity_ + 1.0);
        return false; // Violation found
      }

      // Check acceleration constraint
      Eigen::Vector3d acc = traj.GetAcceleration(t);
      if (acc.norm() > max_acceleration_ + acceleration_tolerance_)
      {
        ROS_WARN_STREAM("Dynamic feasibility check failed: acceleration limit exceeded at t="
                        << t << ", |a|=" << acc.norm() << " > " << max_acceleration_ + 1.0);
        return false; // Violation found
      }
    }

    // Check the very last point
    Eigen::Vector3d end_velocity = traj.GetVelocity(trajectory_duration);
    if (end_velocity.norm() > max_velocity_ + velocity_tolerance_)
    {
      ROS_WARN_STREAM("Dynamic feasibility check failed: velocity limit exceeded at the end of trajectory.");
      return false;
    }
    Eigen::Vector3d end_acceleration = traj.GetAcceleration(trajectory_duration);
    if (end_acceleration.norm() > max_acceleration_ + acceleration_tolerance_)
    {
      ROS_WARN_STREAM("Dynamic feasibility check failed: acceleration limit exceeded at the end of trajectory.");
      return false;
    }

    return true; // Trajectory is feasible
  }
  bool PolyTrajOptimizer::ComputePointsToCheck(
      poly_traj::Trajectory &traj,
      int end_index, PointsToCheck &points_to_check)
  {
    points_to_check.clear();
    points_to_check.resize(end_index);
    const double resolution = grid_map_->GetResolution(), half_resolution = resolution / 2;
    Eigen::VectorXd durations = traj.GetDurations();
    Eigen::VectorXd segment_start_times(durations.size() + 1);
    segment_start_times(0) = 0;
    for (int i = 0; i < durations.size(); ++i)
      segment_start_times(i + 1) = segment_start_times(i) + durations(i);
    const double total_duration = durations.sum();
    double t = 0.0, time_step = min(resolution / max_velocity_, durations.minCoeff() / max(constraint_points_per_piece_, 1) / 1.5);
    Eigen::Vector3d previous_point = traj.GetPosition(0.0);
    // points_to_check[0].push_back(previous_point);
    int current_control_point_index = 0, current_piece_index = 0;

    while (true)
    {
      if (t > total_duration)
      {
        if (touch_goal_ && points_to_check.size() > 0)
        {
          while (points_to_check.back().size() == 0)
          {
            points_to_check.pop_back();
          }

          if (points_to_check.size() <= 0)
          {
            ROS_ERROR("Failed to get points list to check (0x02). pts_check.size()=%d", (int)points_to_check.size());
            return false;
          }
          else
          {
            return true;
          }
        }
        else
        {
          ROS_ERROR("Failed to get points list to check (0x01). touch_goal_=%d, pts_check.size()=%d", touch_goal_, (int)points_to_check.size());
          points_to_check.clear();
          return false;
        }
      }

      const double next_time_step = segment_start_times(current_piece_index) + durations(current_piece_index) / constraint_points_per_piece_ * ((current_control_point_index + 1) - constraint_points_per_piece_ * current_piece_index);
      if (t >= next_time_step)
      {
        if (current_control_point_index + 1 >= constraint_points_per_piece_ * (current_piece_index + 1))
        {
          ++current_piece_index;
        }
        if (++current_control_point_index >= end_index)
        {
          break;
        }
      }

      Eigen::Vector3d pt = traj.GetPosition(t);
      if (t < 1e-5 || points_to_check[current_control_point_index].size() == 0 || (pt - previous_point).cwiseAbs().maxCoeff() > half_resolution)
      {
        points_to_check[current_control_point_index].emplace_back(std::pair<double, Eigen::Vector3d>(t, pt));
        previous_point = pt;
      }

      t += time_step;
    }

    return true;
  }

  /* check collision and set {p,v} pairs to constrain points */
  PolyTrajOptimizer::CheckResult PolyTrajOptimizer::FinelyCheckAndSetConstraintPoints(
      std::vector<std::pair<int, int>> &segments,
      const poly_traj::MinJerkOpt &trajectory_optimizer,
      const bool is_first_initialization /*= true*/)
  {

    Eigen::MatrixXd init_points = trajectory_optimizer.GetInitialConstraintPoints(constraint_points_per_piece_);
    poly_traj::Trajectory traj = trajectory_optimizer.GetTrajectory();

    if (is_first_initialization)
    {
      constraint_points_.Resize(init_points.cols());
      constraint_points_.points_ = init_points;
    }

    /*** Segment the initial trajectory according to obstacles ***/
    vector<std::pair<int, int>> segment_ids;
    constexpr int kEnoughInterval = 2;
    int entry_index = -1, exit_index = -1;
    int same_occupancy_count = kEnoughInterval + 1;
    bool occ, previous_occupancy = false;
    bool found_start = false, found_end = false, maybe_found_end = false;
    int i_end = ConstraintPoints::TwoThirdsIndex(init_points, touch_goal_); // only check closed 2/3 points.

    PointsToCheck points_to_check;
    if (!ComputePointsToCheck(traj, i_end, points_to_check))
    {
      return CheckResult::kError;
    }

    for (int i = 0; i < i_end; ++i)
    {
      for (size_t j = 0; j < points_to_check[i].size(); ++j)
      {
        occ = grid_map_->GetInflatedOccupancy(points_to_check[i][j].second);

        if (occ && !previous_occupancy)
        {
          if (same_occupancy_count > kEnoughInterval || i == 0)
          {
            entry_index = i;
            found_start = true;
          }
          same_occupancy_count = 0;
          maybe_found_end = false; // terminate in advance
        }
        else if (!occ && previous_occupancy)
        {
          exit_index = i + 1;
          maybe_found_end = true;
          same_occupancy_count = 0;
        }
        else
        {
          ++same_occupancy_count;
        }

        if (maybe_found_end && (same_occupancy_count > kEnoughInterval || (i == i_end - 1)))
        {
          maybe_found_end = false;
          found_end = true;
        }

        previous_occupancy = occ;

        if (found_start && found_end)
        {
          found_start = false;
          found_end = false;
          if (entry_index < 0 || exit_index < 0)
          {
            ROS_ERROR("Should not happen! in_id=%d, out_id=%d", entry_index, exit_index);
            return CheckResult::kError;
          }
          segment_ids.push_back(std::pair<int, int>(entry_index, exit_index));
        }
      }
    }

    /* Collision free and return in advance */
    if (segment_ids.size() == 0)
    {
      return CheckResult::kObstacleFree;
    }

    /*** a star search ***/
    vector<vector<Eigen::Vector3d>> a_star_paths;
    for (size_t i = 0; i < segment_ids.size(); ++i)
    {
      // Search from back to head
      Eigen::Vector3d in(init_points.col(segment_ids[i].second)), out(init_points.col(segment_ids[i].first));
      AStarResult ret = a_star_->Search(grid_map_->GetResolution(), in, out);
      if (ret == AStarResult::kSuccess)
      {
        a_star_paths.push_back(a_star_->GetPath());
      }
      else if (ret == AStarResult::kSearchError && i + 1 < segment_ids.size()) // connect the next Segment
      {
        segment_ids[i].second = segment_ids[i + 1].second;
        segment_ids.erase(segment_ids.begin() + i + 1);
        --i;
        ROS_WARN("A corner case 2, I have never exeam it.");
      }
      else
      {
        ROS_WARN_COND(VERBOSE_OUTPUT, "A-star error, force return!");
        ROS_WARN("finelyCheck A-star error, force return!");
        return CheckResult::kError;
      }
    }

    /*** calculate bounds ***/
    int id_low_bound, id_up_bound;
    vector<std::pair<int, int>> bounds(segment_ids.size());
    for (size_t i = 0; i < segment_ids.size(); i++)
    {

      if (i == 0) // first Segment
      {
        id_low_bound = 1;
        if (segment_ids.size() > 1)
        {
          id_up_bound = (int)(((segment_ids[0].second + segment_ids[1].first) - 1.0f) / 2); // id_up_bound : -1.0f fix()
        }
        else
        {
          id_up_bound = init_points.cols() - 2;
        }
      }
      else if (i == segment_ids.size() - 1) // last Segment, i != 0 here
      {
        id_low_bound = (int)(((segment_ids[i].first + segment_ids[i - 1].second) + 1.0f) / 2); // id_low_bound : +1.0f ceil()
        id_up_bound = init_points.cols() - 2;
      }
      else
      {
        id_low_bound = (int)(((segment_ids[i].first + segment_ids[i - 1].second) + 1.0f) / 2); // id_low_bound : +1.0f ceil()
        id_up_bound = (int)(((segment_ids[i].second + segment_ids[i + 1].first) - 1.0f) / 2);  // id_up_bound : -1.0f fix()
      }

      bounds[i] = std::pair<int, int>(id_low_bound, id_up_bound);
    }

    /*** Adjust Segment length ***/
    vector<std::pair<int, int>> adjusted_segment_ids(segment_ids.size());
    constexpr double kMinimumPercent = 0.0; // Each Segment is guaranteed to have sufficient points to generate sufficient force
    int minimum_points = round(init_points.cols() * kMinimumPercent), num_points;
    for (size_t i = 0; i < segment_ids.size(); i++)
    {
      /*** Adjust Segment length ***/
      num_points = segment_ids[i].second - segment_ids[i].first + 1;
      if (num_points < minimum_points)
      {
        double add_points_each_side = (int)(((minimum_points - num_points) + 1.0f) / 2);

        adjusted_segment_ids[i].first = segment_ids[i].first - add_points_each_side >= bounds[i].first
                                            ? segment_ids[i].first - add_points_each_side
                                            : bounds[i].first;

        adjusted_segment_ids[i].second = segment_ids[i].second + add_points_each_side <= bounds[i].second
                                             ? segment_ids[i].second + add_points_each_side
                                             : bounds[i].second;
      }
      else
      {
        adjusted_segment_ids[i].first = segment_ids[i].first;
        adjusted_segment_ids[i].second = segment_ids[i].second;
      }
    }

    for (size_t i = 1; i < adjusted_segment_ids.size(); i++) // Avoid overlap
    {
      if (adjusted_segment_ids[i - 1].second >= adjusted_segment_ids[i].first)
      {
        double middle = (double)(adjusted_segment_ids[i - 1].second + adjusted_segment_ids[i].first) / 2.0;
        adjusted_segment_ids[i - 1].second = static_cast<int>(middle - 0.1);
        adjusted_segment_ids[i].first = static_cast<int>(middle + 1.1);
      }
    }

    // Used for return
    vector<std::pair<int, int>> final_segment_ids;

    /*** Assign data to each Segment ***/
    for (size_t i = 0; i < segment_ids.size(); i++)
    {
      // step 1
      for (int j = adjusted_segment_ids[i].first; j <= adjusted_segment_ids[i].second; ++j)
        constraint_points_.temporary_flags_[j] = false;

      // step 2
      int intersection_index = -1;
      for (int j = segment_ids[i].first + 1; j < segment_ids[i].second; ++j)
      {
        Eigen::Vector3d control_point_normal(init_points.col(j + 1) - init_points.col(j - 1)), intersection_point;
        int a_star_index = a_star_paths[i].size() / 2, previous_a_star_index; // Let "a_star_index = farthest_a_star_point_index" will be better, but it needs more computation
        double val = (a_star_paths[i][a_star_index] - init_points.col(j)).dot(control_point_normal), initial_value = val;
        while (true)
        {

          previous_a_star_index = a_star_index;

          if (val >= 0)
          {
            ++a_star_index; // Previous Astar search from back to head
            if (a_star_index >= (int)a_star_paths[i].size())
            {
              break;
            }
          }
          else
          {
            --a_star_index;
            if (a_star_index < 0)
            {
              break;
            }
          }

          val = (a_star_paths[i][a_star_index] - init_points.col(j)).dot(control_point_normal);

          if (val * initial_value <= 0 && (abs(val) > 0 || abs(initial_value) > 0)) // val = initial_value = 0.0 is not allowed
          {
            intersection_point =
                a_star_paths[i][a_star_index] +
                ((a_star_paths[i][a_star_index] - a_star_paths[i][previous_a_star_index]) *
                 (control_point_normal.dot(init_points.col(j) - a_star_paths[i][a_star_index]) / control_point_normal.dot(a_star_paths[i][a_star_index] - a_star_paths[i][previous_a_star_index])) // = t
                );

            intersection_index = j;
            break;
          }
        }

        if (intersection_index >= 0)
        {
          double length = (intersection_point - init_points.col(j)).norm();
          if (length > 1e-5)
          {
            constraint_points_.temporary_flags_[j] = true;
            for (double a = length; a >= 0.0; a -= grid_map_->GetResolution())
            {
              bool occ = grid_map_->GetInflatedOccupancy((a / length) * intersection_point + (1 - a / length) * init_points.col(j));

              if (occ || a < grid_map_->GetResolution())
              {
                if (occ)
                  a += grid_map_->GetResolution();
                constraint_points_.base_points_[j].push_back((a / length) * intersection_point + (1 - a / length) * init_points.col(j));
                constraint_points_.directions_[j].push_back((intersection_point - init_points.col(j)).normalized());
                break;
              }
            }
          }
          else
          {
            intersection_index = -1;
          }
        }
      }

      /* Corner case: the Segment length is too short. Here the control points may outside the A* path, leading to opposite gradient direction. So I have to take special care of it */
      if (segment_ids[i].second - segment_ids[i].first == 1)
      {
        Eigen::Vector3d control_point_normal(init_points.col(segment_ids[i].second) - init_points.col(segment_ids[i].first)), intersection_point;
        Eigen::Vector3d middle_point = (init_points.col(segment_ids[i].second) + init_points.col(segment_ids[i].first)) / 2;
        int a_star_index = a_star_paths[i].size() / 2, previous_a_star_index; // Let "a_star_index = farthest_a_star_point_index" will be better, but it needs more computation
        double val = (a_star_paths[i][a_star_index] - middle_point).dot(control_point_normal), initial_value = val;
        while (true)
        {

          previous_a_star_index = a_star_index;

          if (val >= 0)
          {
            ++a_star_index; // Previous Astar search from back to head
            if (a_star_index >= (int)a_star_paths[i].size())
            {
              break;
            }
          }
          else
          {
            --a_star_index;
            if (a_star_index < 0)
            {
              break;
            }
          }

          val = (a_star_paths[i][a_star_index] - middle_point).dot(control_point_normal);

          if (val * initial_value <= 0 && (abs(val) > 0 || abs(initial_value) > 0)) // val = initial_value = 0.0 is not allowed
          {
            intersection_point =
                a_star_paths[i][a_star_index] +
                ((a_star_paths[i][a_star_index] - a_star_paths[i][previous_a_star_index]) *
                 (control_point_normal.dot(middle_point - a_star_paths[i][a_star_index]) / control_point_normal.dot(a_star_paths[i][a_star_index] - a_star_paths[i][previous_a_star_index])) // = t
                );

            if ((intersection_point - middle_point).norm() > 0.01) // 1cm.
            {
              constraint_points_.temporary_flags_[segment_ids[i].first] = true;
              constraint_points_.base_points_[segment_ids[i].first].push_back(init_points.col(segment_ids[i].first));
              constraint_points_.directions_[segment_ids[i].first].push_back((intersection_point - middle_point).normalized());

              intersection_index = segment_ids[i].first;
            }
            break;
          }
        }
      }

      //step 3
      if (intersection_index >= 0)
      {
        for (int j = intersection_index + 1; j <= adjusted_segment_ids[i].second; ++j)
          if (!constraint_points_.temporary_flags_[j])
          {
            constraint_points_.base_points_[j].push_back(constraint_points_.base_points_[j - 1].back());
            constraint_points_.directions_[j].push_back(constraint_points_.directions_[j - 1].back());
          }

        for (int j = intersection_index - 1; j >= adjusted_segment_ids[i].first; --j)
          if (!constraint_points_.temporary_flags_[j])
          {
            constraint_points_.base_points_[j].push_back(constraint_points_.base_points_[j + 1].back());
            constraint_points_.directions_[j].push_back(constraint_points_.directions_[j + 1].back());
          }

        final_segment_ids.push_back(adjusted_segment_ids[i]);
      }
      else
      {
        // Just ignore, it does not matter ^_^.
        // ROS_ERROR("Failed to generate direction! segment_id=%d", i);
      }
    }

    segments = final_segment_ids;
    return CheckResult::kFinished;
  }

  bool PolyTrajOptimizer::RoughlyCheckConstraintPoints(void)
  {

    // int end_idx = constraint_points_.control_point_count_ - 1;

    /*** Check and Segment the initial trajectory according to obstacles ***/
    int entry_index, exit_index;
    vector<std::pair<int, int>> segment_ids;
    bool new_obstacle_valid = false;
    int i_end = ConstraintPoints::TwoThirdsIndex(constraint_points_.points_, touch_goal_); // only check closed 2/3 points.
    for (int i = 1; i <= i_end; ++i)
    {

      bool occ = grid_map_->GetInflatedOccupancy(constraint_points_.points_.col(i));

      /*** check if the new collision will be valid ***/
      if (occ)
      {
        for (size_t k = 0; k < constraint_points_.directions_[i].size(); ++k)
        {
          if ((constraint_points_.points_.col(i) - constraint_points_.base_points_[i][k]).dot(constraint_points_.directions_[i][k]) < 1 * grid_map_->GetResolution()) // current point is outside all the collision_points.
          {
            occ = false;
            break;
          }
        }
      }

      if (occ)
      {
        new_obstacle_valid = true;

        int j;
        for (j = i - 1; j >= 0; --j)
        {
          occ = grid_map_->GetInflatedOccupancy(constraint_points_.points_.col(j));
          if (!occ)
          {
            entry_index = j;
            break;
          }
        }
        if (j < 0) // fail to get the obs free point
        {
          ROS_ERROR("The drone is in obstacle. It means a crash in real-world.");
          entry_index = 0;
        }

        for (j = i + 1; j < constraint_points_.control_point_count_; ++j)
        {
          occ = grid_map_->GetInflatedOccupancy(constraint_points_.points_.col(j));

          if (!occ)
          {
            exit_index = j;
            break;
          }
        }
        if (j >= constraint_points_.control_point_count_) // fail to get the obs free point
        {
          ROS_WARN("Local target in collision, skip this planning.");

          force_stop_type_ = kStopForError;
          return false;
        }

        i = j + 1;

        segment_ids.push_back(std::pair<int, int>(entry_index, exit_index));
      }
    }

    if (new_obstacle_valid)
    {
      vector<vector<Eigen::Vector3d>> a_star_paths;
      for (size_t i = 0; i < segment_ids.size(); ++i)
      {
        /*** a star search ***/
        Eigen::Vector3d in(constraint_points_.points_.col(segment_ids[i].second)), out(constraint_points_.points_.col(segment_ids[i].first));
        AStarResult ret = a_star_->Search(/*(in-out).norm()/10+0.05*/ grid_map_->GetResolution(), in, out);
        if (ret == AStarResult::kSuccess)
        {
          a_star_paths.push_back(a_star_->GetPath());
        }
        else if (ret == AStarResult::kSearchError && i + 1 < segment_ids.size()) // connect the next Segment
        {
          segment_ids[i].second = segment_ids[i + 1].second;
          segment_ids.erase(segment_ids.begin() + i + 1);
          --i;
          ROS_WARN("A corner case 2, I have never exeam it.");
        }
        else
        {
          ROS_ERROR_COND(VERBOSE_OUTPUT, "A-star error");
          segment_ids.erase(segment_ids.begin() + i);
          --i;
        }
      }

      for (size_t i = 1; i < segment_ids.size(); i++) // Avoid overlap
      {
        if (segment_ids[i - 1].second >= segment_ids[i].first)
        {
          double middle = (double)(segment_ids[i - 1].second + segment_ids[i].first) / 2.0;
          segment_ids[i - 1].second = static_cast<int>(middle - 0.1);
          segment_ids[i].first = static_cast<int>(middle + 1.1);
        }
      }

      /*** Assign parameters to each Segment ***/
      for (size_t i = 0; i < segment_ids.size(); ++i)
      {
        // step 1
        for (int j = segment_ids[i].first; j <= segment_ids[i].second; ++j)
          constraint_points_.temporary_flags_[j] = false;

        // step 2
        int intersection_index = -1;
        for (int j = segment_ids[i].first + 1; j < segment_ids[i].second; ++j)
        {
          Eigen::Vector3d control_point_normal(constraint_points_.points_.col(j + 1) - constraint_points_.points_.col(j - 1)), intersection_point;
          int a_star_index = a_star_paths[i].size() / 2, previous_a_star_index; // Let "a_star_index = farthest_a_star_point_index" will be better, but it needs more computation
          double val = (a_star_paths[i][a_star_index] - constraint_points_.points_.col(j)).dot(control_point_normal), initial_value = val;
          while (true)
          {

            previous_a_star_index = a_star_index;

            if (val >= 0)
            {
              ++a_star_index; // Previous Astar search from back to head
              if (a_star_index >= (int)a_star_paths[i].size())
              {
                break;
              }
            }
            else
            {
              --a_star_index;
              if (a_star_index < 0)
              {
                break;
              }
            }

            val = (a_star_paths[i][a_star_index] - constraint_points_.points_.col(j)).dot(control_point_normal);

            if (val * initial_value <= 0 && (abs(val) > 0 || abs(initial_value) > 0)) // val = initial_value = 0.0 is not allowed
            {
              intersection_point =
                  a_star_paths[i][a_star_index] +
                  ((a_star_paths[i][a_star_index] - a_star_paths[i][previous_a_star_index]) *
                   (control_point_normal.dot(constraint_points_.points_.col(j) - a_star_paths[i][a_star_index]) / control_point_normal.dot(a_star_paths[i][a_star_index] - a_star_paths[i][previous_a_star_index])) // = t
                  );

              intersection_index = j;
              break;
            }
          }

          if (intersection_index >= 0)
          {
            double length = (intersection_point - constraint_points_.points_.col(j)).norm();
            if (length > 1e-5)
            {
              constraint_points_.temporary_flags_[j] = true;
              for (double a = length; a >= 0.0; a -= grid_map_->GetResolution())
              {
                bool occ = grid_map_->GetInflatedOccupancy((a / length) * intersection_point + (1 - a / length) * constraint_points_.points_.col(j));

                if (occ || a < grid_map_->GetResolution())
                {
                  if (occ)
                    a += grid_map_->GetResolution();
                  constraint_points_.base_points_[j].push_back((a / length) * intersection_point + (1 - a / length) * constraint_points_.points_.col(j));
                  constraint_points_.directions_[j].push_back((intersection_point - constraint_points_.points_.col(j)).normalized());
                  break;
                }
              }
            }
            else
            {
              intersection_index = -1;
            }
          }
        }

        //step 3
        if (intersection_index >= 0)
        {
          for (int j = intersection_index + 1; j <= segment_ids[i].second; ++j)
            if (!constraint_points_.temporary_flags_[j])
            {
              constraint_points_.base_points_[j].push_back(constraint_points_.base_points_[j - 1].back());
              constraint_points_.directions_[j].push_back(constraint_points_.directions_[j - 1].back());
            }

          for (int j = intersection_index - 1; j >= segment_ids[i].first; --j)
            if (!constraint_points_.temporary_flags_[j])
            {
              constraint_points_.base_points_[j].push_back(constraint_points_.base_points_[j + 1].back());
              constraint_points_.directions_[j].push_back(constraint_points_.directions_[j + 1].back());
            }
        }
        else
          ROS_WARN_COND(VERBOSE_OUTPUT, "Failed to generate direction. It doesn't matter.");
      }

      force_stop_type_ = kStopForRebound;
      return true;
    }

    return false;
  }

  bool PolyTrajOptimizer::AllowRebound(void) //zxzxzx
  {
    // criterion 1
    if (iteration_count_ < 3)
      return false;

    // criterion 2
    double min_product = 1;
    for (int i = 3; i <= constraint_points_.points_.cols() - 4; ++i) // ignore head and tail
    {
      double product = ((constraint_points_.points_.col(i) - constraint_points_.points_.col(i - 1)).normalized()).dot((constraint_points_.points_.col(i + 1) - constraint_points_.points_.col(i)).normalized());
      if (product < min_product)
      {
        min_product = product;
      }
    }
    if (min_product < 0.87) // 30 degree
      return false;

    // criterion 3
    if (multi_topology_data_.use_multi_topology_trajectories)
    {
      if (!multi_topology_data_.initial_obstacles_avoided)
      {
        bool avoided = true;
        for (int i = 1; i < constraint_points_.points_.cols() - 1; ++i)
        {
          if (constraint_points_.base_points_[i].size() > 0)
          {
            // Only adopts "0" since FinelyCheckAndSetConstraintPoints() after one optimization can add more base_points.
            if ((constraint_points_.points_.col(i) - constraint_points_.base_points_[i][0]).dot(constraint_points_.directions_[i][0]) < 0)
            {
              avoided = false;
              break;
            }
          }
        }

        multi_topology_data_.initial_obstacles_avoided = avoided;
      }

      if (!multi_topology_data_.initial_obstacles_avoided)
      {
        return false;
      }
    }

    // all the criterion passed
    return true;
  }

  /* multi-topo support */
  std::vector<ConstraintPoints> PolyTrajOptimizer::GenerateDistinctiveTrajectories(vector<std::pair<int, int>> segments)
  {
    if (segments.size() == 0) // will be invoked again later.
    {
      std::vector<ConstraintPoints> segment_info;
      segment_info.push_back(constraint_points_);
      return segment_info;
    }

    constexpr int kMaxTrajectories = 8;
    constexpr int kVariationCount = 2;
    int segment_count = std::min((int)segments.size(), static_cast<int>(floor(log(kMaxTrajectories) / log(kVariationCount))));
    std::vector<ConstraintPoints> control_point_candidates;
    control_point_candidates.reserve(kMaxTrajectories);
    const double resolution = grid_map_->GetResolution();
    const double control_point_distance = (constraint_points_.points_.col(0) - constraint_points_.points_.col(constraint_points_.control_point_count_ - 1)).norm() / (constraint_points_.control_point_count_ - 1);

    // Step 1. Find the opposite vectors and base points for every Segment.
    std::vector<std::pair<ConstraintPoints, ConstraintPoints>> rich_segment_info;
    for (int i = 0; i < segment_count; i++)
    {
      std::pair<ConstraintPoints, ConstraintPoints> segment_info;
      ConstraintPoints temporary_segment_info;
      constraint_points_.Segment(temporary_segment_info, segments[i].first, segments[i].second);
      segment_info.first = temporary_segment_info;
      segment_info.second = temporary_segment_info;
      rich_segment_info.push_back(segment_info);
    }

    for (int i = 0; i < segment_count; i++)
    {

      // 1.1 Find the start occupied point id and the last occupied point id
      if (rich_segment_info[i].first.control_point_count_ > 1)
      {
        int occ_start_id = -1, occ_end_id = -1;
        Eigen::Vector3d occ_start_pt, occ_end_pt;
        for (int j = 0; j < rich_segment_info[i].first.control_point_count_ - 1; j++)
        {
          double step_size = resolution / (rich_segment_info[i].first.points_.col(j) - rich_segment_info[i].first.points_.col(j + 1)).norm() / 2;
          for (double a = 1; a > 0; a -= step_size)
          {
            Eigen::Vector3d pt(a * rich_segment_info[i].first.points_.col(j) + (1 - a) * rich_segment_info[i].first.points_.col(j + 1));
            if (grid_map_->GetInflatedOccupancy(pt))
            {
              occ_start_id = j;
              occ_start_pt = pt;
              goto exit_multi_loop1;
            }
          }
        }
      exit_multi_loop1:;
        for (int j = rich_segment_info[i].first.control_point_count_ - 1; j >= 1; j--)
        {
          ;
          double step_size = resolution / (rich_segment_info[i].first.points_.col(j) - rich_segment_info[i].first.points_.col(j - 1)).norm();
          for (double a = 1; a > 0; a -= step_size)
          {
            Eigen::Vector3d pt(a * rich_segment_info[i].first.points_.col(j) + (1 - a) * rich_segment_info[i].first.points_.col(j - 1));
            if (grid_map_->GetInflatedOccupancy(pt))
            {
              occ_end_id = j;
              occ_end_pt = pt;
              goto exit_multi_loop2;
            }
          }
        }
      exit_multi_loop2:;

        // double check
        if (occ_start_id == -1 || occ_end_id == -1)
        {
          // It means that the first or the last control points of one Segment are in obstacles, which is not allowed.
          // ROS_WARN("What? occ_start_id=%d, occ_end_id=%d", occ_start_id, occ_end_id);

          segments.erase(segments.begin() + i);
          rich_segment_info.erase(rich_segment_info.begin() + i);
          segment_count--;
          i--;

          continue;
        }

        // 1.2 Reverse the vector and find new base points from occ_start_id to occ_end_id.
        for (int j = occ_start_id; j <= occ_end_id; j++)
        {
          Eigen::Vector3d reverse_base_point, reverse_base_vector;
          if (rich_segment_info[i].first.base_points_[j].size() != 1)
          {
            cout << "RichInfoSegs[" << i << "].first.base_point[" << j << "].size()=" << rich_segment_info[i].first.base_points_[j].size() << endl;
            ROS_ERROR("Wrong number of base_points!!! Should not be happen!.");

            cout << setprecision(5);
            cout << "cps_" << endl;
            cout << " clearance=" << obstacle_clearance_ << " cps.size=" << constraint_points_.control_point_count_ << endl;
            for (int temp_i = 0; temp_i < constraint_points_.control_point_count_; temp_i++)
            {
              if (constraint_points_.base_points_[temp_i].size() > 1 && constraint_points_.base_points_[temp_i].size() < 1000)
              {
                ROS_ERROR("Should not happen!!!");
                cout << "######" << constraint_points_.points_.col(temp_i).transpose() << endl;
                for (size_t temp_j = 0; temp_j < constraint_points_.base_points_[temp_i].size(); temp_j++)
                  cout << "      " << constraint_points_.base_points_[temp_i][temp_j].transpose() << " @ " << constraint_points_.directions_[temp_i][temp_j].transpose() << endl;
              }
            }

            std::vector<ConstraintPoints> blank;
            return blank;
          }

          reverse_base_vector = -rich_segment_info[i].first.directions_[j][0];

          // The start and the end case must get taken special care of.
          if (j == occ_start_id)
          {
            reverse_base_point = occ_start_pt;
          }
          else if (j == occ_end_id)
          {
            reverse_base_point = occ_end_pt;
          }
          else
          {
            reverse_base_point = rich_segment_info[i].first.points_.col(j) + reverse_base_vector * (rich_segment_info[i].first.base_points_[j][0] - rich_segment_info[i].first.points_.col(j)).norm();
          }

          if (grid_map_->GetInflatedOccupancy(reverse_base_point)) // Search outward.
          {
            double search_limit = 5 * control_point_distance; // "5" is the threshold.
            double l = resolution;
            for (; l <= search_limit; l += resolution)
            {
              Eigen::Vector3d temporary_base_point = reverse_base_point + l * reverse_base_vector;
              if (!grid_map_->GetInflatedOccupancy(temporary_base_point))
              {
                rich_segment_info[i].second.base_points_[j][0] = temporary_base_point;
                rich_segment_info[i].second.directions_[j][0] = reverse_base_vector;
                break;
              }
            }
            if (l > search_limit)
            {
              ROS_WARN_COND(VERBOSE_OUTPUT, "Can't find the new base points at the opposite within the threshold. i=%d, j=%d", i, j);

              segments.erase(segments.begin() + i);
              rich_segment_info.erase(rich_segment_info.begin() + i);
              segment_count--;
              i--;

              goto exit_multi_loop3; // break "for (int j = 0; j < rich_segment_info[i].first.size; j++)"
            }
          }
          else if ((reverse_base_point - rich_segment_info[i].first.points_.col(j)).norm() >= resolution) // Unnecessary to search.
          {
            rich_segment_info[i].second.base_points_[j][0] = reverse_base_point;
            rich_segment_info[i].second.directions_[j][0] = reverse_base_vector;
          }
          else
          {
            ROS_WARN_COND(VERBOSE_OUTPUT, "base_point and control point are too close!");
            if (VERBOSE_OUTPUT)
              cout << "base_point=" << rich_segment_info[i].first.base_points_[j][0].transpose() << " control point=" << rich_segment_info[i].first.points_.col(j).transpose() << endl;

            segments.erase(segments.begin() + i);
            rich_segment_info.erase(rich_segment_info.begin() + i);
            segment_count--;
            i--;

            goto exit_multi_loop3; // break "for (int j = 0; j < rich_segment_info[i].first.size; j++)"
          }
        }

        // 1.3 Assign the base points to control points within [0, occ_start_id) and (occ_end_id, rich_segment_info[i].first.size()-1].
        if (rich_segment_info[i].second.control_point_count_)
        {
          for (int j = occ_start_id - 1; j >= 0; j--)
          {
            rich_segment_info[i].second.base_points_[j][0] = rich_segment_info[i].second.base_points_[occ_start_id][0];
            rich_segment_info[i].second.directions_[j][0] = rich_segment_info[i].second.directions_[occ_start_id][0];
          }
          for (int j = occ_end_id + 1; j < rich_segment_info[i].second.control_point_count_; j++)
          {
            rich_segment_info[i].second.base_points_[j][0] = rich_segment_info[i].second.base_points_[occ_end_id][0];
            rich_segment_info[i].second.directions_[j][0] = rich_segment_info[i].second.directions_[occ_end_id][0];
          }
        }

      exit_multi_loop3:;
      }
      else
      {
        Eigen::Vector3d reverse_base_vector = -rich_segment_info[i].first.directions_[0][0];
        Eigen::Vector3d reverse_base_point = rich_segment_info[i].first.points_.col(0) + reverse_base_vector * (rich_segment_info[i].first.base_points_[0][0] - rich_segment_info[i].first.points_.col(0)).norm();

        if (grid_map_->GetInflatedOccupancy(reverse_base_point)) // Search outward.
        {
          double search_limit = 5 * control_point_distance; // "5" is the threshold.
          double l = resolution;
          for (; l <= search_limit; l += resolution)
          {
            Eigen::Vector3d temporary_base_point = reverse_base_point + l * reverse_base_vector;
            if (!grid_map_->GetInflatedOccupancy(temporary_base_point))
            {
              rich_segment_info[i].second.base_points_[0][0] = temporary_base_point;
              rich_segment_info[i].second.directions_[0][0] = reverse_base_vector;
              break;
            }
          }
          if (l > search_limit)
          {
            ROS_WARN_COND(VERBOSE_OUTPUT, "Can't find the new base points at the opposite within the threshold, 2. i=%d", i);

            segments.erase(segments.begin() + i);
            rich_segment_info.erase(rich_segment_info.begin() + i);
            segment_count--;
            i--;
          }
        }
        else if ((reverse_base_point - rich_segment_info[i].first.points_.col(0)).norm() >= resolution) // Unnecessary to search.
        {
          rich_segment_info[i].second.base_points_[0][0] = reverse_base_point;
          rich_segment_info[i].second.directions_[0][0] = reverse_base_vector;
        }
        else
        {
          ROS_WARN_COND(VERBOSE_OUTPUT, "base_point and control point are too close!, 2");
          if (VERBOSE_OUTPUT)
            cout << "base_point=" << rich_segment_info[i].first.base_points_[0][0].transpose() << " control point=" << rich_segment_info[i].first.points_.col(0).transpose() << endl;

          segments.erase(segments.begin() + i);
          rich_segment_info.erase(rich_segment_info.begin() + i);
          segment_count--;
          i--;
        }
      }
    }

    // Step 2. Assemble each Segment to make up the new control point sequence.
    if (segment_count == 0) // After the erase operation above, Segment legth will decrease to 0 again.
    {
      std::vector<ConstraintPoints> segment_info;
      segment_info.push_back(constraint_points_);
      return segment_info;
    }

    std::vector<int> selection(segment_count);
    std::fill(selection.begin(), selection.end(), 0);
    selection[0] = -1; // init
    int max_traj_nums = static_cast<int>(pow(kVariationCount, segment_count));
    for (int i = 0; i < max_traj_nums; i++)
    {
      // 2.1 Calculate the selection table.
      int digit_index = 0;
      selection[digit_index]++;
      while (digit_index < segment_count && selection[digit_index] >= kVariationCount)
      {
        selection[digit_index] = 0;
        digit_index++;
        if (digit_index >= segment_count)
        {
          ROS_ERROR("Should not happen!!! digit_id=%d, seg_upbound=%d", digit_index, segment_count);
        }
        selection[digit_index]++;
      }

      // 2.2 Assign params according to the selection table.
      ConstraintPoints sample_constraint_points;
      sample_constraint_points.Resize(constraint_points_.control_point_count_);
      int control_point_index = 0, segment_index = 0, segment_control_point_index = 0;
      while (/*segment_index < rich_segment_info.size() ||*/ control_point_index < constraint_points_.control_point_count_)
      {

        if (segment_index >= segment_count || control_point_index < segments[segment_index].first || control_point_index > segments[segment_index].second)
        {
          sample_constraint_points.points_.col(control_point_index) = constraint_points_.points_.col(control_point_index);
          sample_constraint_points.base_points_[control_point_index] = constraint_points_.base_points_[control_point_index];
          sample_constraint_points.directions_[control_point_index] = constraint_points_.directions_[control_point_index];
        }
        else if (control_point_index >= segments[segment_index].first && control_point_index <= segments[segment_index].second)
        {
          if (!selection[segment_index]) // zx-todo
          {
            sample_constraint_points.points_.col(control_point_index) = rich_segment_info[segment_index].first.points_.col(segment_control_point_index);
            sample_constraint_points.base_points_[control_point_index] = rich_segment_info[segment_index].first.base_points_[segment_control_point_index];
            sample_constraint_points.directions_[control_point_index] = rich_segment_info[segment_index].first.directions_[segment_control_point_index];
            segment_control_point_index++;
          }
          else
          {
            if (rich_segment_info[segment_index].second.control_point_count_)
            {
              sample_constraint_points.points_.col(control_point_index) = rich_segment_info[segment_index].second.points_.col(segment_control_point_index);
              sample_constraint_points.base_points_[control_point_index] = rich_segment_info[segment_index].second.base_points_[segment_control_point_index];
              sample_constraint_points.directions_[control_point_index] = rich_segment_info[segment_index].second.directions_[segment_control_point_index];
              segment_control_point_index++;
            }
            else
            {
              // Abandon this trajectory.
              goto abandon_this_trajectory;
            }
          }

          if (control_point_index == segments[segment_index].second)
          {
            segment_control_point_index = 0;
            segment_index++;
          }
        }
        else
        {
          ROS_ERROR("Shold not happen!!!!, cp_id=%d, seg_id=%d, segments.front().first=%d, segments.back().second=%d, segments[seg_id].first=%d, segments[seg_id].second=%d",
                    control_point_index, segment_index, segments.front().first, segments.back().second, segments[segment_index].first, segments[segment_index].second);
        }

        control_point_index++;
      }

      control_point_candidates.push_back(sample_constraint_points);

    abandon_this_trajectory:;
    }

    return control_point_candidates;
  }

  /* callbacks by the L-BFGS optimizer */
  double PolyTrajOptimizer::CostFunctionCallback(void *func_data, const double *x, double *grad, const int n)
  {
    PolyTrajOptimizer *opt = reinterpret_cast<PolyTrajOptimizer *>(func_data);

    fill(opt->minimum_ellipsoid_distances_squared_.begin(), opt->minimum_ellipsoid_distances_squared_.end(), std::numeric_limits<double>::max());

    Eigen::Map<const Eigen::MatrixXd> inner_points(x, 3, opt->piece_count_ - 1);
    // Eigen::VectorXd real_time(Eigen::VectorXd::Constant(piece_nums, opt->t2T(x[n - 1]))); // same t
    Eigen::Map<const Eigen::VectorXd> t(x + (3 * (opt->piece_count_ - 1)), opt->piece_count_);
    Eigen::Map<Eigen::MatrixXd> point_gradient(grad, 3, opt->piece_count_ - 1);
    Eigen::Map<Eigen::VectorXd> gradt(grad + (3 * (opt->piece_count_ - 1)), opt->piece_count_);
    Eigen::VectorXd real_time(opt->piece_count_);

    Eigen::VectorXd real_time_gradient(opt->piece_count_);
    double smoothness_cost = 0, time_cost = 0;
    Eigen::VectorXd dynamic_costs(4);

    opt->VirtualTimeToRealTime(t, real_time); // Unbounded virtual time to real time

    opt->jerk_optimizer_.Generate(inner_points, real_time); // Generate trajectory from {inner_points,real_time}

    opt->InitializeSmoothnessGradientCost(real_time_gradient, smoothness_cost); // Smoothness cost

    opt->AddDynamicGradientCost(real_time_gradient, dynamic_costs, opt->constraint_points_per_piece_); // Time int cost

    if (opt->AllowRebound())
    {
      opt->RoughlyCheckConstraintPoints(); // Trajectory rebound
    }

    opt->jerk_optimizer_.GetGradientsToTimeAndPoints(real_time_gradient, point_gradient); // Gradient prepagation

    opt->CalculateVirtualTimeGradientCost(real_time, t, real_time_gradient, gradt, time_cost); // Real time back to virtual time

    opt->iteration_count_ += 1;
    return smoothness_cost + dynamic_costs.sum() + time_cost;
  }

  int PolyTrajOptimizer::EarlyExitCallback(void *func_data, const double *x, const double *g, const double fx, const double xnorm, const double gnorm, const double step, int n, int k, int ls)
  {
    PolyTrajOptimizer *opt = reinterpret_cast<PolyTrajOptimizer *>(func_data);

    return (opt->force_stop_type_ == kStopForError || opt->force_stop_type_ == kStopForRebound);
  }

  /* mappings between real world time and unconstrained virtual time */
  template <typename EigenVectorType>
  void PolyTrajOptimizer::RealTimeToVirtualTime(const Eigen::VectorXd &real_time, EigenVectorType &virtual_time)
  {
    for (int i = 0; i < real_time.size(); ++i)
    {
      virtual_time(i) = real_time(i) > 1.0 ? (sqrt(2.0 * real_time(i) - 1.0) - 1.0)
                          : (1.0 - sqrt(2.0 / real_time(i) - 1.0));
    }
  }

  template <typename EigenVectorType>
  void PolyTrajOptimizer::VirtualTimeToRealTime(const EigenVectorType &virtual_time, Eigen::VectorXd &real_time)
  {
    for (int i = 0; i < virtual_time.size(); ++i)
    {
      real_time(i) = virtual_time(i) > 0.0 ? ((0.5 * virtual_time(i) + 1.0) * virtual_time(i) + 1.0)
                          : 1.0 / ((0.5 * virtual_time(i) - 1.0) * virtual_time(i) + 1.0);
    }
  }

  template <typename EigenVectorType, typename GradientVectorType>
  void PolyTrajOptimizer::CalculateVirtualTimeGradientCost(
      const Eigen::VectorXd &real_time, const EigenVectorType &virtual_time,
      const Eigen::VectorXd &real_time_gradient, GradientVectorType &virtual_time_gradient,
      double &time_cost)
  {
    for (int i = 0; i < virtual_time.size(); ++i)
    {
      double virtual_to_real_time_derivative;
      if (virtual_time(i) > 0)
      {
        virtual_to_real_time_derivative = virtual_time(i) + 1.0;
      }
      else
      {
        double denominator = (0.5 * virtual_time(i) - 1.0) * virtual_time(i) + 1.0;
        virtual_to_real_time_derivative = (1.0 - virtual_time(i)) / (denominator * denominator);
      }

      virtual_time_gradient(i) = (real_time_gradient(i) + time_weight_) * virtual_to_real_time_derivative;
    }

    time_cost = real_time.sum() * time_weight_;
  }

  /* gradient and cost evaluation functions */
  template <typename EigenVectorType>
  void PolyTrajOptimizer::InitializeSmoothnessGradientCost(EigenVectorType &duration_gradient, double &cost)
  {
    jerk_optimizer_.InitializeGradientCost(duration_gradient, cost);
  }

  template <typename EigenVectorType>
  void PolyTrajOptimizer::AddDynamicGradientCost(EigenVectorType &duration_gradient, Eigen::VectorXd &costs, const int &samples_per_piece)
  {
    //
    int element_count = duration_gradient.size();
    Eigen::Vector3d pos, vel, acc, jer, sna;
    Eigen::Vector3d gradp, gradv, grada, gradj;
    double costp, costv, costa, costj;
    Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3, beta4;
    double s1, s2, s3, s4, s5;
    double step, alpha;
    Eigen::Matrix<double, 6, 3> position_coefficient_gradient, velocity_coefficient_gradient, acceleration_coefficient_gradient, jerk_coefficient_gradient;
    double position_time_gradient, velocity_time_gradient, acceleration_time_gradient, jerk_time_gradient;
    double omg;
    int i_dp = 0;
    costs.setZero();
    // Eigen::MatrixXd constraint_pts(3, element_count * samples_per_piece + 1);

    // printf("A\n");

    // int integration_point_count;
    double t = 0;
    for (int i = 0; i < element_count; ++i)
    {

      const Eigen::Matrix<double, 6, 3> &c = jerk_optimizer_.GetCoefficients().block<6, 3>(i * 6, 0);
      step = jerk_optimizer_.GetDurations()(i) / samples_per_piece;
      s1 = 0.0;
      // integration_point_count = samples_per_piece;

      for (int j = 0; j <= samples_per_piece; ++j)
      {
        s2 = s1 * s1;
        s3 = s2 * s1;
        s4 = s2 * s2;
        s5 = s4 * s1;
        beta0 << 1.0, s1, s2, s3, s4, s5;
        beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
        beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;
        beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2;
        beta4 << 0.0, 0.0, 0.0, 0.0, 24.0, 120.0 * s1;
        alpha = 1.0 / samples_per_piece * j;
        pos = c.transpose() * beta0;
        vel = c.transpose() * beta1;
        acc = c.transpose() * beta2;
        jer = c.transpose() * beta3;
        sna = c.transpose() * beta4;

        omg = (j == 0 || j == samples_per_piece) ? 0.5 : 1.0;

        constraint_points_.points_.col(i_dp) = pos;

        // collision
        if (CalculateObstacleGradientCost(i_dp, pos, gradp, costp))
        {
          position_coefficient_gradient = beta0 * gradp.transpose();
          position_time_gradient = alpha * gradp.transpose() * vel;
          jerk_optimizer_.GetCoefficientGradients().block<6, 3>(i * 6, 0) += omg * step * position_coefficient_gradient;
          duration_gradient(i) += omg * (costp / samples_per_piece + step * position_time_gradient);
          costs(0) += omg * step * costp;
        }

        // swarm
        double gradt, grad_prev_t;
        if (CalculateSwarmGradientCost(i_dp, t + step * j, pos, vel, gradp, gradt, grad_prev_t, costp))
        {
          position_coefficient_gradient = beta0 * gradp.transpose();
          position_time_gradient = alpha * gradt;
          jerk_optimizer_.GetCoefficientGradients().block<6, 3>(i * 6, 0) += omg * step * position_coefficient_gradient;
          duration_gradient(i) += omg * (costp / samples_per_piece + step * position_time_gradient);
          if (i > 0)
          {
            duration_gradient.head(i).array() += omg * step * grad_prev_t;
          }
          costs(1) += omg * step * costp;
        }

        // feasibility
        if (CalculateVelocityFeasibilityGradientCost(vel, gradv, costv))
        {
          velocity_coefficient_gradient = beta1 * gradv.transpose();
          velocity_time_gradient = alpha * gradv.transpose() * acc;
          jerk_optimizer_.GetCoefficientGradients().block<6, 3>(i * 6, 0) += omg * step * velocity_coefficient_gradient;
          duration_gradient(i) += omg * (costv / samples_per_piece + step * velocity_time_gradient);
          costs(2) += omg * step * costv;
        }

        if (CalculateAccelerationFeasibilityGradientCost(acc, grada, costa))
        {
          acceleration_coefficient_gradient = beta2 * grada.transpose();
          acceleration_time_gradient = alpha * grada.transpose() * jer;
          jerk_optimizer_.GetCoefficientGradients().block<6, 3>(i * 6, 0) += omg * step * acceleration_coefficient_gradient;
          duration_gradient(i) += omg * (costa / samples_per_piece + step * acceleration_time_gradient);
          costs(2) += omg * step * costa;
        }

        if (CalculateJerkFeasibilityGradientCost(jer, gradj, costj))
        {
          jerk_coefficient_gradient = beta3 * gradj.transpose();
          jerk_time_gradient = alpha * gradj.transpose() * sna;
          jerk_optimizer_.GetCoefficientGradients().block<6, 3>(i * 6, 0) += omg * step * jerk_coefficient_gradient;
          duration_gradient(i) += omg * (costj / samples_per_piece + step * jerk_time_gradient);
          costs(2) += omg * step * costj;
        }

        // printf("L\n");

        s1 += step;
        if (j != samples_per_piece || (j == samples_per_piece && i == element_count - 1))
        {
          ++i_dp;
        }
      }

      t += jerk_optimizer_.GetDurations()(i);
    }

    // quratic variance
    Eigen::MatrixXd gdp;
    double var;
    // CalculateLengthVarianceGradientCost(constraint_points_.points_, samples_per_piece, gdp, var);
    CalculateSquaredDistanceVarianceGradientCost(constraint_points_.points_, gdp, var);

    i_dp = 0;
    for (int i = 0; i < element_count; ++i)
    {
      step = jerk_optimizer_.GetDurations()(i) / samples_per_piece;
      s1 = 0.0;

      for (int j = 0; j <= samples_per_piece; ++j)
      {
        s2 = s1 * s1;
        s3 = s2 * s1;
        s4 = s2 * s2;
        s5 = s4 * s1;
        beta0 << 1.0, s1, s2, s3, s4, s5;
        beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
        alpha = 1.0 / samples_per_piece * j;
        vel = jerk_optimizer_.GetCoefficients().block<6, 3>(i * 6, 0).transpose() * beta1;

        omg = (j == 0 || j == samples_per_piece) ? 0.5 : 1.0;

        position_coefficient_gradient = beta0 * gdp.col(i_dp).transpose();
        position_time_gradient = alpha * gdp.col(i_dp).transpose() * vel;
        jerk_optimizer_.GetCoefficientGradients().block<6, 3>(i * 6, 0) += omg * position_coefficient_gradient;
        duration_gradient(i) += omg * (position_time_gradient);

        s1 += step;
        if (j != samples_per_piece || (j == samples_per_piece && i == element_count - 1))
        {
          ++i_dp;
        }
      }
    }

    costs(3) += var;
  }

  bool PolyTrajOptimizer::CalculateObstacleGradientCost(const int i_dp,
                                            const Eigen::Vector3d &p,
                                            Eigen::Vector3d &gradp,
                                            double &costp)
  {
    if (i_dp == 0 || i_dp > ConstraintPoints::TwoThirdsIndex(constraint_points_.points_, touch_goal_)) // only apply to first 2/3
      return false;

    bool ret = false;

    gradp.setZero();
    costp = 0;

    // Obatacle cost
    for (size_t j = 0; j < constraint_points_.directions_[i_dp].size(); ++j)
    {
      Eigen::Vector3d ray = (p - constraint_points_.base_points_[i_dp][j]);
      double dist = ray.dot(constraint_points_.directions_[i_dp][j]);
      double dist_err = obstacle_clearance_ - dist;
      double dist_err_soft = soft_obstacle_clearance_ - dist;
      Eigen::Vector3d dist_grad = constraint_points_.directions_[i_dp][j];

      if (dist_err > 0)
      {
        ret = true;
        costp += obstacle_weight_ * pow(dist_err, 3);
        gradp += -obstacle_weight_ * 3.0 * dist_err * dist_err * dist_grad;
      }

      if (dist_err_soft > 0)
      {
        ret = true;
        double r = 0.05;
        double rsqr = r * r;
        double term = sqrt(1.0 + dist_err_soft * dist_err_soft / rsqr);
        costp += soft_obstacle_weight_ * rsqr * (term - 1.0);
        gradp += -soft_obstacle_weight_ * dist_err_soft / term * dist_grad;
      }
    }

    return ret;
  }

  bool PolyTrajOptimizer::CalculateSwarmGradientCost(const int i_dp,
                                         const double t,
                                         const Eigen::Vector3d &p,
                                         const Eigen::Vector3d &v,
                                         Eigen::Vector3d &gradp,
                                         double &gradt,
                                         double &grad_prev_t,
                                         double &costp)
  {
    if (i_dp <= 0 || i_dp > ConstraintPoints::TwoThirdsIndex(constraint_points_.points_, touch_goal_)) // only apply to first 2/3
      return false;

    bool ret = false;

    gradp.setZero();
    gradt = 0;
    grad_prev_t = 0;
    costp = 0;

    constexpr double kVerticalScale = 2.0;
    constexpr double kHorizontalScale = 1.0;
    constexpr double kInverseVerticalScaleSquared =
        1.0 / kVerticalScale / kVerticalScale;
    constexpr double kInverseHorizontalScaleSquared =
        1.0 / kHorizontalScale / kHorizontalScale;

    for (size_t id = 0; id < swarm_trajectories_->size(); id++)
    {
      if ((swarm_trajectories_->at(id).drone_id < 0) || swarm_trajectories_->at(id).drone_id == drone_id_)
      {
        continue;
      }

      double traj_i_satrt_time = swarm_trajectories_->at(id).start_time;
      double pt_time = (current_time_ - traj_i_satrt_time) + t; // never assign a high-precision golbal time to a double directly!
      const double clearance = (swarm_clearance_ + swarm_trajectories_->at(id).desired_clearance) * 1.5; // 1.5 is to compensate slight constraint violation
      const double clearance_squared = clearance * clearance;

      Eigen::Vector3d swarm_p, swarm_v;
      if (pt_time < swarm_trajectories_->at(id).duration)
      {
        swarm_p = swarm_trajectories_->at(id).trajectory.GetPosition(pt_time);
        swarm_v = swarm_trajectories_->at(id).trajectory.GetVelocity(pt_time);
      }
      else
      {
        double exceed_time = pt_time - swarm_trajectories_->at(id).duration;
        swarm_v = swarm_trajectories_->at(id).trajectory.GetVelocity(swarm_trajectories_->at(id).duration);
        swarm_p = swarm_trajectories_->at(id).trajectory.GetPosition(swarm_trajectories_->at(id).duration) +
                  exceed_time * swarm_v;
      }
      Eigen::Vector3d dist_vec = p - swarm_p;
      double ellip_dist2 = dist_vec(2) * dist_vec(2) * kInverseVerticalScaleSquared +
                           (dist_vec(0) * dist_vec(0) + dist_vec(1) * dist_vec(1)) *
                               kInverseHorizontalScaleSquared;
      double dist2_err = clearance_squared - ellip_dist2;
      double dist2_err2 = dist2_err * dist2_err;
      double dist2_err3 = dist2_err2 * dist2_err;

      if (dist2_err3 > 0)
      {
        ret = true;

        costp += modified_swarm_weight_ * dist2_err3;

        Eigen::Vector3d position_cost_gradient =
            modified_swarm_weight_ * 3 * dist2_err2 * (-2) *
            Eigen::Vector3d(kInverseHorizontalScaleSquared * dist_vec(0),
                            kInverseHorizontalScaleSquared * dist_vec(1),
                            kInverseVerticalScaleSquared * dist_vec(2));
        gradp += position_cost_gradient;
        gradt += position_cost_gradient.dot(v - swarm_v);
        grad_prev_t += position_cost_gradient.dot(-swarm_v);
      }

      if (minimum_ellipsoid_distances_squared_[id] > ellip_dist2)
      {
        minimum_ellipsoid_distances_squared_[id] = ellip_dist2;
      }
    }

    return ret;
  }

  bool PolyTrajOptimizer::CalculateVelocityFeasibilityGradientCost(const Eigen::Vector3d &v,
                                               Eigen::Vector3d &gradv,
                                               double &costv)
  {
    double vpen = v.squaredNorm() - max_velocity_ * max_velocity_;
    if (vpen > 0)
    {
      gradv = feasibility_weight_ * 6 * vpen * vpen * v;
      costv = feasibility_weight_ * vpen * vpen * vpen;
      return true;
    }
    return false;
  }

  bool PolyTrajOptimizer::CalculateAccelerationFeasibilityGradientCost(const Eigen::Vector3d &a,
                                               Eigen::Vector3d &grada,
                                               double &costa)
  {
    double apen = a.squaredNorm() - max_acceleration_ * max_acceleration_;
    if (apen > 0)
    {
      grada = feasibility_weight_ * 6 * apen * apen * a;
      costa = feasibility_weight_ * apen * apen * apen;
      return true;
    }
    return false;
  }

  bool PolyTrajOptimizer::CalculateJerkFeasibilityGradientCost(const Eigen::Vector3d &j,
                                               Eigen::Vector3d &gradj,
                                               double &costj)
  {
    double jpen = j.squaredNorm() - max_jerk_ * max_jerk_;
    if (jpen > 0)
    {
      gradj = feasibility_weight_ * 6 * jpen * jpen * j;
      costj = feasibility_weight_ * jpen * jpen * jpen;
      return true;
    }
    return false;
  }

  void PolyTrajOptimizer::CalculateSquaredDistanceVarianceGradientCost(const Eigen::MatrixXd &ps,
                                                            Eigen::MatrixXd &gdp,
                                                            double &var)
  {
    int element_count = ps.cols() - 1;
    Eigen::MatrixXd dps = ps.rightCols(element_count) - ps.leftCols(element_count);
    Eigen::VectorXd dsqrs = dps.colwise().squaredNorm().transpose();
    // double dsqrsum = dsqrs.sum();
    double dquarsum = dsqrs.squaredNorm();
    // double dsqrmean = dsqrsum / element_count;
    double dquarmean = dquarsum / element_count;
    var = squared_variance_weight_ * (dquarmean);
    gdp.resize(3, element_count + 1);
    gdp.setZero();
    for (int i = 0; i <= element_count; i++)
    {
      if (i != 0)
      {
        gdp.col(i) += squared_variance_weight_ * (4.0 * (dsqrs(i - 1)) / element_count * dps.col(i - 1));
      }
      if (i != element_count)
      {
        gdp.col(i) += squared_variance_weight_ * (-4.0 * (dsqrs(i)) / element_count * dps.col(i));
      }
    }
    return;
  }

  void PolyTrajOptimizer::CalculateLengthVarianceGradientCost(const Eigen::MatrixXd &ps,
                                                       const int n,
                                                       Eigen::MatrixXd &gdp,
                                                       double &var)
  {
    int element_count = ps.cols() - 1;
    int group_count = element_count / n;
    Eigen::MatrixXd dps = ps.rightCols(element_count) - ps.leftCols(element_count);
    Eigen::VectorXd ds = dps.colwise().norm().transpose();
    Eigen::VectorXd ls(group_count), lsqrs(group_count);
    for (int i = 0; i < group_count; i++)
    {
      ls(i) = ds.segment(i * n, n).sum();
      lsqrs(i) = ls(i) * ls(i);
    }
    double lm = ls.mean();
    double lsqrm = lsqrs.mean();
    var = squared_variance_weight_ * (lsqrm - lm * lm) + 250.0 * group_count * lm;
    Eigen::VectorXd gdls = squared_variance_weight_ * 2.0 / group_count * (ls.array() - lm) + 250.0;
    Eigen::MatrixXd gdds = dps.colwise().normalized();
    gdp.resize(3, element_count + 1);
    gdp.setZero();
    for (int i = 0; i < group_count; i++)
    {
      gdp.block(0, i * n, 3, n) -= gdls(i) * gdds.block(0, i * n, 3, n);
      gdp.block(0, i * n + 1, 3, n) += gdls(i) * gdds.block(0, i * n, 3, n);
    }
    return;
  }

  /* helper functions */
  void PolyTrajOptimizer::SetParameters(ros::NodeHandle &nh)
  {
    nh.param("optimization/constraint_points_per_piece", constraint_points_per_piece_, -1);
    nh.param("optimization/weight_obstacle", obstacle_weight_, -1.0);
    nh.param("optimization/weight_obstacle_soft", soft_obstacle_weight_, -1.0);
    nh.param("optimization/weight_swarm", swarm_weight_, -1.0);
    nh.param("optimization/weight_feasibility", feasibility_weight_, -1.0);
    nh.param("optimization/weight_sqrvariance", squared_variance_weight_, -1.0);
    nh.param("optimization/weight_time", time_weight_, -1.0);
    nh.param("optimization/obstacle_clearance", obstacle_clearance_, -1.0);
    nh.param("optimization/obstacle_clearance_soft", soft_obstacle_clearance_, -1.0);
    nh.param("optimization/swarm_clearance", swarm_clearance_, -1.0);
    nh.param("optimization/max_vel", max_velocity_, -1.0);
    nh.param("optimization/vel_tolerance", velocity_tolerance_, -1.0);
    nh.param("optimization/max_acc", max_acceleration_, -1.0);
    nh.param("optimization/acc_tolerance", acceleration_tolerance_, -1.0);
    nh.param("optimization/max_jer", max_jerk_, -1.0);
  }

  void PolyTrajOptimizer::SetEnvironment(const GridMap::Ptr &map)
  {
    grid_map_ = map;

    a_star_.reset(new AStar);
    a_star_->InitializeGridMap(grid_map_, Eigen::Vector3i(100, 100, 100));
  }

  void PolyTrajOptimizer::SetControlPoints(const Eigen::MatrixXd &points)
  {
    constraint_points_.points_ = points;
  }

  void PolyTrajOptimizer::SetSwarmTrajectories(SwarmTrajectoryData *swarm_trajectories) { swarm_trajectories_ = swarm_trajectories; }

  void PolyTrajOptimizer::SetDroneId(const int drone_id) { drone_id_ = drone_id; }

  void PolyTrajOptimizer::SetTouchGoal(const bool touch_goal) { touch_goal_ = touch_goal; }

  void PolyTrajOptimizer::SetConstraintPoints(ConstraintPoints constraint_points) { constraint_points_ = constraint_points; }

  void PolyTrajOptimizer::SetUseMultiTopologyTrajectories(bool use_multi_topology_trajectories) { multi_topology_data_.use_multi_topology_trajectories = use_multi_topology_trajectories; }

} // namespace diff_planner
