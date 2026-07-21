#ifndef DIFF_PLANNER_TRAJ_UTILS_INCLUDE_TRAJ_UTILS_PLANNING_VISUALIZATION_H_
#define DIFF_PLANNER_TRAJ_UTILS_INCLUDE_TRAJ_UTILS_PLANNING_VISUALIZATION_H_

#include <eigen3/Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <stdlib.h>

namespace diff_planner
{
  class PlanningVisualization
  {
  private:
    ros::NodeHandle node_;

    ros::Publisher goal_point_pub_;
    ros::Publisher global_list_pub_;
    ros::Publisher initial_list_pub_;
    ros::Publisher optimal_list_pub_;
    ros::Publisher failed_list_pub_;
    ros::Publisher a_star_list_pub_;
    ros::Publisher guide_vector_pub_;

    ros::Publisher intermediate_point_0_pub_;
    ros::Publisher intermediate_point_1_pub_;
    ros::Publisher intermediate_gradient_0_pub_;
    ros::Publisher intermediate_gradient_1_pub_;
    ros::Publisher intermediate_smoothness_gradient_pub_;
    ros::Publisher intermediate_distance_gradient_pub_;
    ros::Publisher intermediate_feasibility_gradient_pub_;
    ros::Publisher intermediate_swarm_gradient_pub_;

  public:
    PlanningVisualization(/* args */) {}
    ~PlanningVisualization() {}
    PlanningVisualization(ros::NodeHandle &nh);

    using Ptr = std::shared_ptr<PlanningVisualization>;

    void DisplayMarkerList(ros::Publisher &publisher,
                           const std::vector<Eigen::Vector3d> &points, double scale,
                           Eigen::Vector4d color, int id, bool show_sphere = true);
    void GeneratePathDisplayArray(visualization_msgs::MarkerArray &array,
                                  const std::vector<Eigen::Vector3d> &points,
                                  double scale, Eigen::Vector4d color, int id);
    void GenerateArrowDisplayArray(visualization_msgs::MarkerArray &array,
                                   const std::vector<Eigen::Vector3d> &points,
                                   double scale, Eigen::Vector4d color, int id);
    void DisplayGoalPoint(Eigen::Vector3d goal_point, Eigen::Vector4d color,
                          const double scale, int id);
    void DisplayGlobalPathList(std::vector<Eigen::Vector3d> global_points,
                               const double scale, int id);
    void DisplayInitialPathList(std::vector<Eigen::Vector3d> initial_points,
                                const double scale, int id);
    void DisplayMultiInitialPathList(
        std::vector<std::vector<Eigen::Vector3d>> initial_trajectories,
        const double scale);
    void DisplayMultiOptimalPathList(
        std::vector<std::vector<Eigen::Vector3d>> optimal_trajectories,
        const double scale);
    void DisplayOptimalList(Eigen::MatrixXd optimal_points, int id);
    void DisplayFailedList(Eigen::MatrixXd failed_points, int id);
    void DisplayAStarList(std::vector<std::vector<Eigen::Vector3d>> a_star_paths, int id);
    void DisplayArrowList(ros::Publisher &publisher,
                          const std::vector<Eigen::Vector3d> &points, double scale,
                          Eigen::Vector4d color, int id);
    
    void DisplayIntermediatePoint(std::string type, Eigen::MatrixXd &points,
                                  int id, Eigen::Vector4d color);
    void DisplayIntermediateGradient(std::string type, Eigen::MatrixXd &points,
                                     Eigen::MatrixXd &gradient, int id,
                                     Eigen::Vector4d color);
    // void DisplayNewArrow(ros::Publisher& guide_vector_publisher,
    //                      diff_planner::PolyTrajOptimizer::Ptr optimizer);
  };
} // namespace diff_planner
#endif  // DIFF_PLANNER_TRAJ_UTILS_INCLUDE_TRAJ_UTILS_PLANNING_VISUALIZATION_H_
