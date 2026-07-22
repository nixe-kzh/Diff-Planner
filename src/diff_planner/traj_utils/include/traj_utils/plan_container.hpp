#ifndef DIFF_PLANNER_TRAJ_UTILS_INCLUDE_TRAJ_UTILS_PLAN_CONTAINER_HPP_
#define DIFF_PLANNER_TRAJ_UTILS_INCLUDE_TRAJ_UTILS_PLAN_CONTAINER_HPP_

#include <Eigen/Eigen>
#include <vector>
#include <ros/ros.h>

#include <optimizer/poly_traj_utils.hpp>

namespace diff_planner
{

  using PointsToCheck = std::vector<std::vector<std::pair<double, Eigen::Vector3d>>>;

  struct GlobalTrajectoryData
  {
    poly_traj::Trajectory trajectory;
    double global_start_time; // world time
    double duration;

    /* Global traj time. 
       The corresponding global trajectory time of the current local target.
       Used in local target selection process */
    double local_target_time;
    /* Global traj time. 
       The corresponding global trajectory time of the last local target.
       Used in initial-path-from-last-optimal-trajectory generation process */
    double previous_local_target_time;
  };

  struct LocalTrajectoryData
  {
    poly_traj::Trajectory trajectory;
    PointsToCheck points_to_check;
    int drone_id; // A negative value indicates no received trajectories.
    int trajectory_id;
    double duration;
    double start_time; // world time
    double end_time;   // world time
    Eigen::Vector3d start_position;
    double desired_clearance;

  };

  using SwarmTrajectoryData = std::vector<LocalTrajectoryData>;

  class TrajectoryContainer
  {
  public:
    GlobalTrajectoryData global_trajectory_;
    LocalTrajectoryData local_trajectory_;
    SwarmTrajectoryData swarm_trajectories_;

    TrajectoryContainer()
    {
      local_trajectory_.trajectory_id = 0;
    }
    ~TrajectoryContainer() {}

    void SetGlobalTrajectory(const poly_traj::Trajectory &trajectory, const double &world_time)
    {
      global_trajectory_.trajectory = trajectory;
      global_trajectory_.duration = trajectory.GetTotalDuration();
      global_trajectory_.global_start_time = world_time;
      global_trajectory_.local_target_time = world_time;
      global_trajectory_.previous_local_target_time = -1.0;

      local_trajectory_.drone_id = -1;
      local_trajectory_.duration = 0.0;
      local_trajectory_.trajectory_id = 0;
    }

    void SetLocalTrajectory(const poly_traj::Trajectory &trajectory,
                            const PointsToCheck &points_to_check,
                            const double &world_time, const int drone_id = -1)
    {
      local_trajectory_.drone_id = drone_id;
      local_trajectory_.trajectory_id++;
      local_trajectory_.duration = trajectory.GetTotalDuration();
      local_trajectory_.start_position = trajectory.GetJunctionPosition(0);
      local_trajectory_.start_time = world_time;
      local_trajectory_.trajectory = trajectory;
      local_trajectory_.points_to_check = points_to_check;
    }

  };

  struct PlanParameters
  {
    /* planning algorithm parameters */
    double max_velocity, max_acceleration; // physical limits
    double trajectory_piece_length;       // distance between adjacent trajectory points
    double feasibility_tolerance;         // permitted ratio of vel/acc exceeding limits
    double planning_horizon;
    bool use_multi_topology_trajectories;
    bool touch_goal;
    int drone_id; // single drone: drone_id <= -1, swarm: drone_id >= 0

    /* processing time */
    double search_time = 0.0;
    double optimization_time = 0.0;
    double adjustment_time = 0.0;
  };

} // namespace diff_planner

#endif  // DIFF_PLANNER_TRAJ_UTILS_INCLUDE_TRAJ_UTILS_PLAN_CONTAINER_HPP_
