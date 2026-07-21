#include <nav_msgs/Odometry.h>
#include <traj_utils/PolyTraj.h>
#include <optimizer/poly_traj_utils.hpp>
#include <quadrotor_msgs/PositionCommand.h>
#include <std_msgs/Empty.h>
#include <visualization_msgs/Marker.h>
#include <ros/ros.h>

using namespace Eigen;

ros::Publisher position_command_publisher;

quadrotor_msgs::PositionCommand position_command;
// double pos_gain[3] = {0, 0, 0};
// double vel_gain[3] = {0, 0, 0};

#define FLIP_YAW_AT_END 0
#define TURN_YAW_TO_CENTER_AT_END 0

bool has_received_trajectory = false;
boost::shared_ptr<poly_traj::Trajectory> trajectory;
double trajectory_duration;
ros::Time trajectory_start_time;
int trajectory_id;
ros::Time heartbeat_time(0);
Eigen::Vector3d last_position;

// yaw control
double last_yaw, last_yaw_rate, flip_yaw_target, center_yaw_target;
double lookahead_time;
double custom_yaw;
double yaw_rate_limit = 2 * M_PI;
double yaw_acceleration_limit = 5 * M_PI;

bool has_received_yaw = false;
ros::Time yaw_receive_time(0);

void HeartbeatCallback(std_msgs::EmptyPtr message)
{
  heartbeat_time = ros::Time::now();
}

void YawCallback(const quadrotor_msgs::PositionCommandPtr message)
{
  has_received_yaw = true;
  yaw_receive_time = ros::Time::now();
  custom_yaw = message->yaw;
  // std::cout << "Received yaw:  " << custom_yaw << std::endl;
}

void PolynomialTrajectoryCallback(traj_utils::PolyTrajPtr message)
{
  if (message->order != 5)
  {
    ROS_ERROR("[traj_server] Only support trajectory order equals 5 now!");
    return;
  }
  if (message->duration.size() * (message->order + 1) != message->coef_x.size())
  {
    ROS_ERROR("[traj_server] WRONG trajectory parameters, ");
    return;
  }

  int piece_count = message->duration.size();
  std::vector<double> durations(piece_count);
  std::vector<poly_traj::CoefficientMatrix> coefficient_matrices(piece_count);
  for (int i = 0; i < piece_count; ++i)
  {
    int coefficient_offset = i * 6;
    coefficient_matrices[i].row(0) << message->coef_x[coefficient_offset + 0], message->coef_x[coefficient_offset + 1], message->coef_x[coefficient_offset + 2],
        message->coef_x[coefficient_offset + 3], message->coef_x[coefficient_offset + 4], message->coef_x[coefficient_offset + 5];
    coefficient_matrices[i].row(1) << message->coef_y[coefficient_offset + 0], message->coef_y[coefficient_offset + 1], message->coef_y[coefficient_offset + 2],
        message->coef_y[coefficient_offset + 3], message->coef_y[coefficient_offset + 4], message->coef_y[coefficient_offset + 5];
    coefficient_matrices[i].row(2) << message->coef_z[coefficient_offset + 0], message->coef_z[coefficient_offset + 1], message->coef_z[coefficient_offset + 2],
        message->coef_z[coefficient_offset + 3], message->coef_z[coefficient_offset + 4], message->coef_z[coefficient_offset + 5];

    durations[i] = message->duration[i];
  }

  trajectory.reset(new poly_traj::Trajectory(durations, coefficient_matrices));

  trajectory_start_time = message->start_time;
  trajectory_duration = trajectory->GetTotalDuration();
  trajectory_id = message->traj_id;

  has_received_trajectory = true;
}

std::pair<double, double> CalculateYaw(double current_time,
                                       Eigen::Vector3d &position,
                                       double time_step)
{
  std::pair<double, double> yaw_and_rate(0, 0);

  Eigen::Vector3d direction = current_time + lookahead_time <= trajectory_duration
                            ? trajectory->GetPosition(current_time + lookahead_time) - position
                            : trajectory->GetPosition(trajectory_duration) - position;
  double target_yaw = direction.norm() > 0.1
                        ? atan2(direction(1), direction(0))
                        : last_yaw;
  if (has_received_yaw && custom_yaw > -100.0)
  { 
    if ((ros::Time::now() - yaw_receive_time).toSec() < 0.5)
    {
      target_yaw = custom_yaw;
    }
    else
    {
      has_received_yaw = false;
    }
  }

  double yaw_rate = 0;
  double yaw_difference = target_yaw - last_yaw;
  if (yaw_difference >= M_PI)
  {
    yaw_difference -= 2 * M_PI;
  }
  if (yaw_difference <= -M_PI)
  {
    yaw_difference += 2 * M_PI;
  }

  const double maximum_yaw_rate = yaw_difference >= 0 ? yaw_rate_limit : -yaw_rate_limit;
  const double maximum_yaw_acceleration = yaw_difference >= 0 ? yaw_acceleration_limit : -yaw_acceleration_limit;
  double maximum_yaw_change;
  if (fabs(last_yaw_rate + time_step * maximum_yaw_acceleration) <= fabs(maximum_yaw_rate))
  {
    // yaw_rate = last_yaw_rate + time_step * maximum_yaw_acceleration;
    maximum_yaw_change = last_yaw_rate * time_step + 0.5 * maximum_yaw_acceleration * time_step * time_step;
  }
  else
  {
    // yaw_rate = maximum_yaw_rate;
    double acceleration_time = (maximum_yaw_rate - last_yaw_rate) / maximum_yaw_acceleration;
    maximum_yaw_change = ((time_step - acceleration_time) + time_step) * (maximum_yaw_rate - last_yaw_rate) / 2.0;
  }

  if (fabs(yaw_difference) > fabs(maximum_yaw_change))
  {
    yaw_difference = maximum_yaw_change;
  }
  yaw_rate = yaw_difference / time_step;

  double yaw = last_yaw + yaw_difference;
  if (yaw > M_PI)
    yaw -= 2 * M_PI;
  if (yaw < -M_PI)
    yaw += 2 * M_PI;
  yaw_and_rate.first = yaw;
  yaw_and_rate.second = yaw_rate;

  last_yaw = yaw_and_rate.first;
  last_yaw_rate = yaw_and_rate.second;

  yaw_and_rate.second = target_yaw;

  return yaw_and_rate;
}

void PublishCommand(Vector3d position, Vector3d velocity,
                    Vector3d acceleration, Vector3d jerk, double yaw,
                    double yaw_rate)
{

  position_command.header.stamp = ros::Time::now();
  position_command.header.frame_id = "world";
  position_command.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  position_command.trajectory_id = trajectory_id;

  position_command.position.x = position(0);
  position_command.position.y = position(1);
  position_command.position.z = position(2);
  position_command.velocity.x = velocity(0);
  position_command.velocity.y = velocity(1);
  position_command.velocity.z = velocity(2);
  position_command.acceleration.x = acceleration(0);
  position_command.acceleration.y = acceleration(1);
  position_command.acceleration.z = acceleration(2);
  position_command.jerk.x = jerk(0);
  position_command.jerk.y = jerk(1);
  position_command.jerk.z = jerk(2);
  position_command.yaw = yaw;
  position_command.yaw_dot = yaw_rate;
  position_command_publisher.publish(position_command);

  last_position = position;
}

void CommandTimerCallback(const ros::TimerEvent &event)
{
  /* no publishing before receive trajectory and have heartbeat */
  if (heartbeat_time.toSec() <= 1e-5)
  {
    // ROS_ERROR_ONCE("[traj_server] No heartbeat from the planner received");
    return;
  }
  if (!has_received_trajectory)
    return;

  ros::Time current_ros_time = ros::Time::now();

  if ((current_ros_time - heartbeat_time).toSec() > 0.5)
  {
    ROS_ERROR("[traj_server] Lost heartbeat from the planner, is it dead?");

    has_received_trajectory = false;
    PublishCommand(last_position, Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(), last_yaw, 0);
  }

  double current_time = (current_ros_time - trajectory_start_time).toSec();

  Eigen::Vector3d position(Eigen::Vector3d::Zero()), velocity(Eigen::Vector3d::Zero()), acceleration(Eigen::Vector3d::Zero()), jerk(Eigen::Vector3d::Zero());
  std::pair<double, double> yaw_and_rate(0, 0);

  static ros::Time last_update_time = ros::Time::now();
#if FLIP_YAW_AT_END or TURN_YAW_TO_CENTER_AT_END
  static bool finished = false;
#endif
  if (current_time < trajectory_duration && current_time >= 0.0)
  {
    position = trajectory->GetPosition(current_time);
    velocity = trajectory->GetVelocity(current_time);
    acceleration = trajectory->GetAcceleration(current_time);
    jerk = trajectory->GetJerk(current_time);

    /*** calculate yaw ***/
    yaw_and_rate = CalculateYaw(current_time, position, 0.01);
    /*** calculate yaw ***/

    last_update_time = current_ros_time;
    last_yaw = yaw_and_rate.first;
    last_position = position;

    flip_yaw_target = yaw_and_rate.first + M_PI;
    if (flip_yaw_target > M_PI)
      flip_yaw_target -= 2 * M_PI;
    if (flip_yaw_target < -M_PI)
      flip_yaw_target += 2 * M_PI;
    constexpr double kCenter[2] = {0.0, 0.0};
    center_yaw_target = atan2(kCenter[1] - position(1), kCenter[0] - position(0));

    // publish
    PublishCommand(position, velocity, acceleration, jerk, yaw_and_rate.first, yaw_and_rate.second);
#if FLIP_YAW_AT_END or TURN_YAW_TO_CENTER_AT_END
    finished = false;
#endif
  }

#if FLIP_YAW_AT_END
  else if (current_time >= trajectory_duration)
  {
    if (finished)
      return;

    /* hover when finished trajectory */
    position = trajectory->GetPosition(trajectory_duration);
    velocity.setZero();
    acceleration.setZero();
    jerk.setZero();

    if (flip_yaw_target > 0)
    {
      last_yaw += (current_ros_time - last_update_time).toSec() * M_PI / 2;
      yaw_and_rate.second = M_PI / 2;
      if (last_yaw >= flip_yaw_target)
      {
        finished = true;
      }
    }
    else
    {
      last_yaw -= (current_ros_time - last_update_time).toSec() * M_PI / 2;
      yaw_and_rate.second = -M_PI / 2;
      if (last_yaw <= flip_yaw_target)
      {
        finished = true;
      }
    }

    yaw_and_rate.first = last_yaw;
    last_update_time = current_ros_time;

    PublishCommand(position, velocity, acceleration, jerk, yaw_and_rate.first, yaw_and_rate.second);
  }
#endif

#if TURN_YAW_TO_CENTER_AT_END
  else if (current_time >= trajectory_duration)
  {
    if (finished)
      return;

    /* hover when finished trajectory */
    position = trajectory->GetPosition(trajectory_duration);
    velocity.setZero();
    acceleration.setZero();
    jerk.setZero();

    double yaw_difference = last_yaw - center_yaw_target;
    if (yaw_difference >= M_PI)
    {
      last_yaw += (current_ros_time - last_update_time).toSec() * M_PI / 2;
      yaw_and_rate.second = M_PI / 2;
      if (last_yaw > M_PI)
        last_yaw -= 2 * M_PI;
    }
    else if (yaw_difference <= -M_PI)
    {
      last_yaw -= (current_ros_time - last_update_time).toSec() * M_PI / 2;
      yaw_and_rate.second = -M_PI / 2;
      if (last_yaw < -M_PI)
        last_yaw += 2 * M_PI;
    }
    else if (yaw_difference >= 0)
    {
      last_yaw -= (current_ros_time - last_update_time).toSec() * M_PI / 2;
      yaw_and_rate.second = -M_PI / 2;
      if (last_yaw <= center_yaw_target)
        finished = true;
    }
    else
    {
      last_yaw += (current_ros_time - last_update_time).toSec() * M_PI / 2;
      yaw_and_rate.second = M_PI / 2;
      if (last_yaw >= center_yaw_target)
        finished = true;
    }

    yaw_and_rate.first = last_yaw;
    last_update_time = current_ros_time;

    PublishCommand(position, velocity, acceleration, jerk, yaw_and_rate.first, yaw_and_rate.second);
  }
#endif
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "traj_server");
  // ros::NodeHandle node;
  ros::NodeHandle node_handle("~");

  ros::Subscriber polynomial_trajectory_subscriber = node_handle.subscribe("planning/trajectory", 10, PolynomialTrajectoryCallback);
  ros::Subscriber yaw_subscriber = node_handle.subscribe("/planning/yaw", 10, YawCallback);
  ros::Subscriber heartbeat_subscriber = node_handle.subscribe("heartbeat", 10, HeartbeatCallback);
  
  position_command_publisher = node_handle.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);

  ros::Timer command_timer = node_handle.createTimer(ros::Duration(0.01), CommandTimerCallback);

  node_handle.param("traj_server/time_forward", lookahead_time, -1.0);
  node_handle.param("traj_server/yaw_dot_max", yaw_rate_limit, yaw_rate_limit);
  node_handle.param("traj_server/yaw_dot_dot_max", yaw_acceleration_limit, yaw_acceleration_limit);
  last_yaw = 0.0;
  last_yaw_rate = 0.0;

  ros::Duration(1.0).sleep();

  ROS_INFO("[Traj server]: ready.");

  ros::spin();

  return 0;
}
