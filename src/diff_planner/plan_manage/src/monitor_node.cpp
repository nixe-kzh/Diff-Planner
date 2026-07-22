#include <std_msgs/Empty.h>
#include <ros/ros.h>
#include <future>
#include <thread>
#include <Eigen/Eigen>
#include <quadrotor_msgs/GoalSet.h>

ros::Timer command_timer;
ros::Publisher waypoint_publisher;
ros::Time heartbeat_time(0);
bool publish_recorded_point = false;
bool has_received_waypoint = false;
Eigen::Vector3d recorded_waypoint(0, 0, 0);

void RestartProcess(const std::string &command) {
  std::cout << "restart process: " << command << std::endl;
  system((command + " &").c_str());
  std::cout << "restarted: " << command << std::endl;
}

void KillProcess(const std::string &process_name) {
  std::cout << "Killing process: " << process_name << std::endl;
  system(("pkill -9 " + process_name).c_str());
  std::cout << "Process killed: " << process_name << std::endl;
}

void HeartbeatCallback(std_msgs::EmptyPtr message) {
  heartbeat_time = ros::Time::now();
}

void MonitorCallback(const ros::TimerEvent &event) {
  if (heartbeat_time.toSec() <= 1e-5) {
    return;
  }
  static int publication_count = 0;
  static bool publish_switch_point = true;
  ros::Time current_time = ros::Time::now();
  if ((current_time - heartbeat_time).toSec() > 4 &&
      publish_switch_point) {
    publish_switch_point = false;
    command_timer.stop();
    ROS_ERROR("[monitor] Lost heartbeat from the planner, planner maybe dead !!! will restart diff_planner !!!");
    std::string process_name = "diff_planner";
    std::string start_command =
        "roslaunch diff_planner single_drone_interactive.launch";
    std::future<void> kill_result =
        std::async(std::launch::async, KillProcess, process_name);
    kill_result.wait();
    std::future<void> start_result =
        std::async(std::launch::async, RestartProcess, start_command);
    start_result.wait();
    ROS_INFO("cmd timer start");
    publish_recorded_point = true;
    command_timer.start();
  }
  if (publish_recorded_point) {
    publication_count++;
    if (publication_count == 3) {
      publication_count = 0;
      if (has_received_waypoint) {
        quadrotor_msgs::GoalSet message;
        message.goal[0] = recorded_waypoint(0);
        message.goal[1] = recorded_waypoint(1);
        message.goal[2] = recorded_waypoint(2);
        waypoint_publisher.publish(message);
        publish_recorded_point = false;
        has_received_waypoint = false;
        ROS_ERROR("publish record point msg: %f, %f, %f", message.goal[0],
                  message.goal[1], message.goal[2]);
        publish_switch_point = true;
      }
    }
  }
}

void WaypointCallback(const quadrotor_msgs::GoalSetPtr &message) {
  if (message->goal[2] < -0.1 || message->goal[1] > 20000 ||
      message->goal[0] > 20000) {
    return;
  }
  ROS_INFO("Monitor received goal: %f, %f, %f", message->goal[0],
           message->goal[1], message->goal[2]);
  recorded_waypoint(0) = message->goal[0];
  recorded_waypoint(1) = message->goal[1];
  recorded_waypoint(2) = message->goal[2];
  has_received_waypoint = true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "monitor_node");
  ros::NodeHandle node_handle("~");
  ros::Subscriber heartbeat_subscriber = node_handle.subscribe(
      "/drone_0_traj_server/heartbeat", 10, HeartbeatCallback);
  ros::Subscriber waypoint_subscriber =
      node_handle.subscribe("/goal_with_id", 1, WaypointCallback);
  waypoint_publisher =
      node_handle.advertise<quadrotor_msgs::GoalSet>("/goal_with_id", 10);
  ros::Timer monitor_timer =
      node_handle.createTimer(ros::Duration(1.0), MonitorCallback);
  ros::Duration(1.0).sleep();
  ROS_INFO("[monitor:] monitor node is ready.");
  ros::spin();
  return 0;
}
