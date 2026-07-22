#include <Eigen/Eigen>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <iostream>
#include <sensor_msgs/Joy.h>
#include <nav_msgs/Odometry.h>
#include <traj_utils/MINCOTraj.h>
#include <traj_utils/planning_visualization.h>
#include <optimizer/poly_traj_utils.hpp>

ros::Publisher obstacle_one_odometry_publisher;
ros::Publisher obstacle_two_odometry_publisher;
ros::Publisher trajectory_publisher;
ros::Publisher predicted_trajectory_publisher;

double obstacle_one_id;
double obstacle_two_id;

diff_planner::PlanningVisualization::Ptr visualization;

class MovingObstacle {
 private:
  Eigen::Vector2d position_{Eigen::Vector2d::Zero()};
  Eigen::Vector2d velocity_{Eigen::Vector2d::Zero()};
  double yaw_{0};
  ros::Time last_update_time_{ros::Time(0)};

 public:
  double desired_clearance_;

  MovingObstacle() = default;
  ~MovingObstacle() = default;

  void SetPosition(Eigen::Vector2d pos) {
    position_ = pos;
  }

  double GetYaw() { return yaw_; }

  void UpdateDynamics(const double delta_t, const double acc, const double dir,
                      double &yaw, Eigen::Vector2d &pos,
                      Eigen::Vector2d &vel) const {
    Eigen::Vector2d acc_vec = acc * Eigen::Vector2d(cos(yaw), sin(yaw));
    vel += acc_vec * delta_t;
    vel *= 0.9; // gradually stop like a real obstacle
    constexpr double kMaximumVelocity = 2.0;
    if (vel.norm() > kMaximumVelocity) {
      vel /= vel.norm() / kMaximumVelocity;
    }
    pos += vel * delta_t + 0.5 * acc_vec * delta_t * delta_t;
    yaw += dir * delta_t;
  }

  std::pair<Eigen::Vector2d, Eigen::Vector2d> Update(
      const double acc, const double dir) {
    ros::Time current_time = ros::Time::now();
    if (last_update_time_ == ros::Time(0)) {
      last_update_time_ = current_time;
    }

    double delta_t = (current_time - last_update_time_).toSec();
    UpdateDynamics(delta_t, acc, dir, yaw_, position_, velocity_);

    last_update_time_ = current_time;

    return {position_, velocity_};
  }

  std::pair<Eigen::Vector2d, Eigen::Vector2d> Predict(
      const double acc, const double dir, double predict_t) const {
    constexpr double kTimeStep = 0.1;
    double yaw = yaw_;
    Eigen::Vector2d pos = position_;
    Eigen::Vector2d vel = velocity_;

    for (double t = kTimeStep; t <= predict_t; t += kTimeStep) {
      UpdateDynamics(kTimeStep, acc, dir, yaw, pos, vel);
    }

    return {pos, vel};
  }
};

MovingObstacle obstacle_one;
MovingObstacle obstacle_two;

poly_traj::Trajectory PredictTrajectory(
    const double acc, const double dir, const Eigen::Vector3d p,
    const Eigen::Vector3d v, const MovingObstacle &obstacle,
    std::vector<Eigen::Vector3d> &visualization_points) {
  visualization_points.clear();
  constexpr double kPredictionTime = 5.0;
  constexpr int kSegmentCount = 10;
  poly_traj::MinJerkOpt predicted_traj;
  Eigen::Matrix<double, 3, 3> head_state, tail_state;
  head_state << p, v, Eigen::Vector3d::Zero();
  Eigen::MatrixXd inner_points(3, kSegmentCount - 1);
  Eigen::VectorXd durations(kSegmentCount);
  visualization_points.push_back(head_state.col(0));
  for (int i = 1; i < kSegmentCount; ++i) {
    auto predicted_state =
        obstacle.Predict(acc, dir, kPredictionTime / kSegmentCount * i);
    inner_points.col(i - 1) = Eigen::Vector3d(
        predicted_state.first(0), predicted_state.first(1), p(2));
    durations(i - 1) = kPredictionTime / kSegmentCount;
    visualization_points.push_back(inner_points.col(i - 1));
  }
  durations(kSegmentCount - 1) = kPredictionTime / kSegmentCount;
  auto final_state = obstacle.Predict(acc, dir, kPredictionTime);
  tail_state << Eigen::Vector3d(final_state.first(0), final_state.first(1),
                                p(2)),
      Eigen::Vector3d(final_state.second(0), final_state.second(1), v(2)),
      Eigen::Vector3d::Zero();
  visualization_points.push_back(tail_state.col(0));
  predicted_traj.Reset(head_state, tail_state, kSegmentCount);
  predicted_traj.Generate(inner_points, durations);
  return predicted_traj.GetTrajectory();
}

void ConvertTrajectoryToRosMessage(
    const poly_traj::Trajectory &traj, const double desired_clearance,
    const int obstacle_id, traj_utils::MINCOTraj &minco_message) {
  Eigen::VectorXd durations = traj.GetDurations();
  int piece_count = traj.GetPieceCount();
  double duration = durations.sum();

  minco_message.drone_id = obstacle_id;
  minco_message.traj_id = 0;
  minco_message.start_time = ros::Time::now();
  minco_message.order = 5; // todo, only support order = 5 now.
  minco_message.duration.resize(piece_count);
  minco_message.des_clearance = desired_clearance;
  Eigen::Vector3d vector;
  vector = traj.GetPosition(0);
  minco_message.start_p[0] = vector(0);
  minco_message.start_p[1] = vector(1);
  minco_message.start_p[2] = vector(2);
  vector = traj.GetVelocity(0);
  minco_message.start_v[0] = vector(0);
  minco_message.start_v[1] = vector(1);
  minco_message.start_v[2] = vector(2);
  vector = traj.GetAcceleration(0);
  minco_message.start_a[0] = vector(0);
  minco_message.start_a[1] = vector(1);
  minco_message.start_a[2] = vector(2);
  vector = traj.GetPosition(duration);
  minco_message.end_p[0] = vector(0);
  minco_message.end_p[1] = vector(1);
  minco_message.end_p[2] = vector(2);
  vector = traj.GetVelocity(duration);
  minco_message.end_v[0] = vector(0);
  minco_message.end_v[1] = vector(1);
  minco_message.end_v[2] = vector(2);
  vector = traj.GetAcceleration(duration);
  minco_message.end_a[0] = vector(0);
  minco_message.end_a[1] = vector(1);
  minco_message.end_a[2] = vector(2);
  minco_message.inner_x.resize(piece_count - 1);
  minco_message.inner_y.resize(piece_count - 1);
  minco_message.inner_z.resize(piece_count - 1);
  Eigen::MatrixXd pos = traj.GetPositions();
  for (int i = 0; i < piece_count - 1; i++) {
    minco_message.inner_x[i] = pos(0, i + 1);
    minco_message.inner_y[i] = pos(1, i + 1);
    minco_message.inner_z[i] = pos(2, i + 1);
  }
  for (int i = 0; i < piece_count; i++) {
    minco_message.duration[i] = durations[i];
  }
}

// #      ^                ^
// #    +1|              +4|
// # <-+0      ->     <-+3      ->
// #      |                |
// #      V                V

void JoyCallback(const sensor_msgs::Joy::ConstPtr &message) {
  ros::Time current_time = ros::Time::now();

  double acc1 = message->axes[1] * 2;
  double dir1 = message->axes[0] / 3;
  double acc2 = message->axes[4] * 2;
  double dir2 = message->axes[3] / 3;
  if (acc1 < 0) {
    dir1 = -dir1;
  }
  if (acc2 < 0) {
    dir2 = -dir2;
  }

  auto pv1 = obstacle_one.Update(acc1, dir1);
  auto pv2 = obstacle_two.Update(acc2, dir2);

  constexpr double kHeight = 1.0;

  // publish odometry
  nav_msgs::Odometry odometry_message;
  odometry_message.header.stamp = current_time;
  odometry_message.header.frame_id = "world";
  odometry_message.pose.pose.position.z = kHeight;
  odometry_message.twist.twist.linear.z = 0.0;
  odometry_message.pose.pose.orientation.x = 0.0;
  odometry_message.pose.pose.orientation.y = 0.0;

  Eigen::Quaterniond q1(Eigen::AngleAxisd(obstacle_one.GetYaw(),
                                         Eigen::Vector3d::UnitZ()));
  odometry_message.pose.pose.position.x = pv1.first(0);
  odometry_message.pose.pose.position.y = pv1.first(1);
  odometry_message.twist.twist.linear.x = pv1.second(0);
  odometry_message.twist.twist.linear.y = pv1.second(1);
  odometry_message.pose.pose.orientation.w = q1.w();
  odometry_message.pose.pose.orientation.z = q1.z();
  obstacle_one_odometry_publisher.publish(odometry_message);
  ros::Duration(0.005).sleep();

  Eigen::Quaterniond q2(Eigen::AngleAxisd(obstacle_two.GetYaw(),
                                         Eigen::Vector3d::UnitZ()));
  odometry_message.pose.pose.position.x = pv2.first(0);
  odometry_message.pose.pose.position.y = pv2.first(1);
  odometry_message.twist.twist.linear.x = pv2.second(0);
  odometry_message.twist.twist.linear.y = pv2.second(1);
  odometry_message.pose.pose.orientation.w = q2.w();
  odometry_message.pose.pose.orientation.z = q2.z();
  obstacle_two_odometry_publisher.publish(odometry_message);
  ros::Duration(0.005).sleep();

  // publish predicted trajectory
  traj_utils::MINCOTraj minco_message;
  std::vector<Eigen::Vector3d> visualization_points;
  poly_traj::Trajectory trajectory_one = PredictTrajectory(
      acc1, dir1, Eigen::Vector3d(pv1.first[0], pv1.first[1], kHeight),
      Eigen::Vector3d(pv1.second[0], pv1.second[1], 0), obstacle_one,
      visualization_points);
  ConvertTrajectoryToRosMessage(trajectory_one,
                                obstacle_one.desired_clearance_,
                                obstacle_one_id, minco_message);
  predicted_trajectory_publisher.publish(minco_message);
  ros::Duration(0.005).sleep();
  visualization->DisplayInitialPathList(visualization_points, 0.1, obstacle_one_id);
  ros::Duration(0.005).sleep();

  poly_traj::Trajectory trajectory_two = PredictTrajectory(
      acc2, dir2, Eigen::Vector3d(pv2.first[0], pv2.first[1], kHeight),
      Eigen::Vector3d(pv2.second[0], pv2.second[1], 0), obstacle_two,
      visualization_points);
  ConvertTrajectoryToRosMessage(trajectory_two,
                                obstacle_two.desired_clearance_,
                                obstacle_two_id, minco_message);
  predicted_trajectory_publisher.publish(minco_message);
  ros::Duration(0.005).sleep();
  visualization->DisplayInitialPathList(visualization_points, 0.1, obstacle_two_id);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "moving_obstacles");
  ros::NodeHandle node_handle("~");

  obstacle_one_odometry_publisher =
      node_handle.advertise<nav_msgs::Odometry>("odom_obs1", 10);
  obstacle_two_odometry_publisher =
      node_handle.advertise<nav_msgs::Odometry>("odom_obs2", 10);
  predicted_trajectory_publisher =
      node_handle.advertise<traj_utils::MINCOTraj>(
          "/broadcast_traj_to_planner", 10);
  ros::Subscriber joystick_subscriber =
      node_handle.subscribe<sensor_msgs::Joy>("joy", 10, JoyCallback);

  visualization.reset(new diff_planner::PlanningVisualization(node_handle));

  std::vector<double> initial_position;
  node_handle.getParam("obstacle1_init_pos", initial_position);
  obstacle_one.SetPosition(Eigen::Vector2d(initial_position[0], initial_position[1]));
  node_handle.getParam("desired_clearance1", obstacle_one.desired_clearance_);
  node_handle.getParam("obstacle2_init_pos", initial_position);
  obstacle_two.SetPosition(Eigen::Vector2d(initial_position[0], initial_position[1]));
  node_handle.getParam("desired_clearance2", obstacle_two.desired_clearance_);
  node_handle.getParam("obstacle1_id", obstacle_one_id);
  node_handle.getParam("obstacle2_id", obstacle_two_id);

  while (ros::ok()) {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  return 0;
}
