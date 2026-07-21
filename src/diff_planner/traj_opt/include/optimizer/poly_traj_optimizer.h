#ifndef DIFF_PLANNER_TRAJ_OPT_INCLUDE_OPTIMIZER_POLY_TRAJ_OPTIMIZER_H_
#define DIFF_PLANNER_TRAJ_OPT_INCLUDE_OPTIMIZER_POLY_TRAJ_OPTIMIZER_H_

#include <Eigen/Eigen>
#include <memory>
#include <path_searching/dyn_a_star.h>
#include <plan_env/grid_map.h>
#include <ros/ros.h>
#include "optimizer/lbfgs.hpp"
#include <traj_utils/plan_container.hpp>
#include "poly_traj_utils.hpp"

namespace diff_planner
{

  class ConstraintPoints
  {
  public:
    int control_point_count_; // deformation points_
    Eigen::MatrixXd points_;
    std::vector<std::vector<Eigen::Vector3d>> base_points_; // The point at the statrt of the directions_ vector (collision point)
    std::vector<std::vector<Eigen::Vector3d>> directions_;  // Direction vector, must be normalized.
    std::vector<bool> temporary_flags_;                          // A flag that used in many places. Initialize it everytime before using it.

    void Resize(const int size)
    {
      control_point_count_ = size;

      base_points_.clear();
      directions_.clear();
      temporary_flags_.clear();

      points_.resize(3, size);
      base_points_.resize(control_point_count_);
      directions_.resize(control_point_count_);
      temporary_flags_.resize(control_point_count_);
    }

    void Segment(ConstraintPoints &output, const int start, const int end)
    {
      if (start < 0 || end >= control_point_count_ || points_.rows() != 3)
      {
        ROS_ERROR("Wrong segment index! start=%d, end=%d", start, end);
        return;
      }

      output.Resize(end - start + 1);
      output.points_ = points_.block(0, start, 3, end - start + 1);
      output.control_point_count_ = end - start + 1;
      for (int i = start; i <= end; i++)
      {
        output.base_points_[i - start] = base_points_[i];
        output.directions_[i - start] = directions_[i];
      }
    }

    static inline int TwoThirdsIndex(Eigen::MatrixXd &points_, const bool touch_goal)
    {
      return touch_goal ? points_.cols() - 1 : points_.cols() - 1 - (points_.cols() - 2) / 3;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };

  class PolyTrajOptimizer
  {

  private:
    GridMap::Ptr grid_map_;
    AStar::Ptr a_star_;
    poly_traj::MinJerkOpt jerk_optimizer_;
    SwarmTrajectoryData *swarm_trajectories_{NULL}; // Can not use shared_ptr and no need to free
    ConstraintPoints constraint_points_;
    // PointsToCheck pts_check_;

    int drone_id_;
    int constraint_points_per_piece_;   // number of distinctive constraint points each piece
    int variable_count_;       // optimization variables
    int piece_count_;          // poly traj piece numbers
    int iteration_count_;           // iteration of the solver
    std::vector<double> minimum_ellipsoid_distances_squared_; // min trajectory distance in swarm
    bool touch_goal_;
    struct MultiTopologyData
    {
      bool use_multi_topology_trajectories{false};
      bool initial_obstacles_avoided{false};
    } multi_topology_data_;

    enum ForceStopType
    {
      kDoNotStop,
      kStopForRebound,
      kStopForError
    } force_stop_type_;

    /* optimization parameters */
    double obstacle_weight_, soft_obstacle_weight_;                               // obstacle weight
    double swarm_weight_, modified_swarm_weight_;                            // swarm weight
    double feasibility_weight_;                                             // feasibility weight
    double squared_variance_weight_;                                           // squared variance weight
    double time_weight_;                                             // time weight
    double obstacle_clearance_, soft_obstacle_clearance_, swarm_clearance_; // safe distance
    double max_velocity_, max_acceleration_, max_jerk_, velocity_tolerance_, acceleration_tolerance_;                          // dynamic limits

    double current_time_;

  public:
    PolyTrajOptimizer() {}
    ~PolyTrajOptimizer() {}

    enum CheckResult
    {
      kObstacleFree,
      kError,
      kFinished
    };

    /* set variables */
    void SetParameters(ros::NodeHandle &nh);
    void SetEnvironment(const GridMap::Ptr &map);
    void SetControlPoints(const Eigen::MatrixXd &points);
    void SetSwarmTrajectories(SwarmTrajectoryData *swarm_trajectories);
    void SetDroneId(const int drone_id);
    void SetTouchGoal(const bool touch_goal);
    void SetConstraintPoints(ConstraintPoints constraint_points);
    void SetUseMultiTopologyTrajectories(bool use_multi_topology_trajectories);

    /* helper functions */
    inline const ConstraintPoints &GetControlPoints(void) { return constraint_points_; }
    inline const poly_traj::MinJerkOpt &GetMinimumJerkOptimizer(void) { return jerk_optimizer_; }
    inline int GetConstraintPointsPerPiece(void) { return constraint_points_per_piece_; }
    inline double GetSwarmClearance(void) { return swarm_clearance_; }

    /* main planning API */
    bool OptimizeTrajectory(const Eigen::MatrixXd &initial_state,
                            const Eigen::MatrixXd &final_state,
                            const Eigen::MatrixXd &initial_inner_points,
                            const Eigen::VectorXd &initial_durations,
                            double &final_cost);

    bool ComputePointsToCheck(poly_traj::Trajectory &trajectory, int end_index,
                              PointsToCheck &points_to_check);

    bool CheckDynamicFeasibility(const poly_traj::MinJerkOpt &trajectory_optimizer);
    std::vector<std::pair<int, int>> FinelyCheckConstraintPointsOnly(Eigen::MatrixXd &init_points);

    /* check collision and set {p,v} pairs to constraint points */
    CheckResult FinelyCheckAndSetConstraintPoints(std::vector<std::pair<int, int>> &segments,
                                              const poly_traj::MinJerkOpt &trajectory_optimizer,
                                              const bool is_first_initialization /*= true*/);

    bool RoughlyCheckConstraintPoints(void);

    bool AllowRebound(void);

    /* multi-topo support */
    std::vector<ConstraintPoints> GenerateDistinctiveTrajectories(
        std::vector<std::pair<int, int>> segments);

  private:
    /* callbacks by the L-BFGS optimizer */
    static double CostFunctionCallback(void *func_data, const double *x, double *grad, const int n);

    static int EarlyExitCallback(void *func_data, const double *x, const double *g,
                                 const double fx, const double xnorm, const double gnorm,
                                 const double step, int n, int k, int ls);

    /* mappings between real world time and unconstrained virtual time */
    template <typename EigenVectorType>
    void RealTimeToVirtualTime(const Eigen::VectorXd &real_time,
                               EigenVectorType &virtual_time);

    template <typename EigenVectorType>
    void VirtualTimeToRealTime(const EigenVectorType &virtual_time,
                               Eigen::VectorXd &real_time);

    template <typename EigenVectorType, typename GradientVectorType>
    void CalculateVirtualTimeGradientCost(
        const Eigen::VectorXd &real_time, const EigenVectorType &virtual_time,
        const Eigen::VectorXd &real_time_gradient,
        GradientVectorType &virtual_time_gradient, double &time_cost);

    /* gradient and cost evaluation functions */
    template <typename EigenVectorType>
    void InitializeSmoothnessGradientCost(EigenVectorType &duration_gradient,
                                          double &cost);

    template <typename EigenVectorType>
    void AddDynamicGradientCost(EigenVectorType &duration_gradient,
                                Eigen::VectorXd &costs,
                                const int &samples_per_piece);

    bool CalculateObstacleGradientCost(const int i_dp,
                           const Eigen::Vector3d &p,
                           Eigen::Vector3d &gradp,
                           double &costp);

    bool CalculateSwarmGradientCost(const int i_dp,
                        const double t,
                        const Eigen::Vector3d &p,
                        const Eigen::Vector3d &v,
                        Eigen::Vector3d &gradp,
                        double &gradt,
                        double &grad_prev_t,
                        double &costp);

    bool CalculateVelocityFeasibilityGradientCost(const Eigen::Vector3d &v,
                              Eigen::Vector3d &gradv,
                              double &costv);

    bool CalculateAccelerationFeasibilityGradientCost(const Eigen::Vector3d &a,
                              Eigen::Vector3d &grada,
                              double &costa);

    bool CalculateJerkFeasibilityGradientCost(const Eigen::Vector3d &j,
                              Eigen::Vector3d &gradj,
                              double &costj);

    void CalculateSquaredDistanceVarianceGradientCost(const Eigen::MatrixXd &ps,
                                           Eigen::MatrixXd &gdp,
                                           double &var);

    void CalculateLengthVarianceGradientCost(const Eigen::MatrixXd &ps,
                                      const int n,
                                      Eigen::MatrixXd &gdp,
                                      double &var);

  public:
    using Ptr = std::unique_ptr<PolyTrajOptimizer>;
  };

} // namespace diff_planner
#endif  // DIFF_PLANNER_TRAJ_OPT_INCLUDE_OPTIMIZER_POLY_TRAJ_OPTIMIZER_H_
