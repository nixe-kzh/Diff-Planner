#ifndef DIFF_PLANNER_PLAN_MANAGE_INCLUDE_PLAN_MANAGE_PLANNER_MANAGER_H_
#define DIFF_PLANNER_PLAN_MANAGE_INCLUDE_PLAN_MANAGE_PLANNER_MANAGER_H_

#include <stdlib.h>

#include <memory>
#include <vector>

#include <optimizer/poly_traj_optimizer.h>
#include <traj_utils/DataDisp.h>
#include <plan_env/grid_map.h>
#include <traj_utils/plan_container.hpp>
#include <ros/ros.h>
#include <traj_utils/planning_visualization.h>
#include <optimizer/poly_traj_utils.hpp>

namespace diff_planner
{

  // Fast Planner Manager
  // Key algorithms of mapping and planning are called

  class DiffPlannerManager
  {
    // SECTION stable
  public:
    DiffPlannerManager();
    ~DiffPlannerManager();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /* main planning interface */
    void InitPlanModules(ros::NodeHandle &node_handle,
                         PlanningVisualization::Ptr visualization = nullptr);
    bool ComputeInitialState(
        const Eigen::Vector3d &start_point, const Eigen::Vector3d &start_velocity,
        const Eigen::Vector3d &start_acceleration,
        const Eigen::Vector3d &local_target_point,
        const Eigen::Vector3d &local_target_velocity,
        bool use_polynomial_initialization,
        bool use_random_polynomial_trajectory, double piece_duration,
        poly_traj::MinJerkOpt &initial_jerk_optimizer);
    bool ReboundReplan(
        const Eigen::Vector3d &start_point, const Eigen::Vector3d &start_velocity,
        const Eigen::Vector3d &start_acceleration,
        const Eigen::Vector3d &local_target_point,
        const Eigen::Vector3d &local_target_velocity,
        bool use_polynomial_initialization,
        bool use_random_polynomial_trajectory, bool touch_goal);
    bool PlanGlobalTrajectoryWaypoints(
        const Eigen::Vector3d &start_position,
        const Eigen::Vector3d &start_velocity,
        const Eigen::Vector3d &start_acceleration,
        const std::vector<Eigen::Vector3d> &waypoints,
        const Eigen::Vector3d &end_velocity,
        const Eigen::Vector3d &end_acceleration);
    void GetLocalTarget(
        double planning_horizon,
        const Eigen::Vector3d &start_point,
        const Eigen::Vector3d &global_end_point,
        Eigen::Vector3d &local_target_position,
        Eigen::Vector3d &local_target_velocity,
        bool &touch_goal);
    bool EmergencyStop(Eigen::Vector3d stop_position);
    bool CheckCollision(int drone_id);
    bool SetLocalTrajectoryFromOptimizer(
        const poly_traj::MinJerkOpt &optimizer, bool touch_goal);
    inline double GetSwarmClearance() const {
      return trajectory_optimizer_->GetSwarmClearance();
    }
    inline int GetConstraintPointsPerPiece() const {
      return trajectory_optimizer_->GetConstraintPointsPerPiece();
    }
    PlanParameters plan_parameters_;
    GridMap::Ptr grid_map_;
    TrajectoryContainer trajectory_container_;

  private:
    PlanningVisualization::Ptr visualization_;

    PolyTrajOptimizer::Ptr trajectory_optimizer_;

    int continuous_failure_count_{0};

  public:
    using Ptr = std::unique_ptr<DiffPlannerManager>;

    // !SECTION
  };
}  // namespace diff_planner

#endif  // DIFF_PLANNER_PLAN_MANAGE_INCLUDE_PLAN_MANAGE_PLANNER_MANAGER_H_
