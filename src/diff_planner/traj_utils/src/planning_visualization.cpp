#include <traj_utils/planning_visualization.h>

using std::cout;
using std::endl;
namespace diff_planner
{
  PlanningVisualization::PlanningVisualization(ros::NodeHandle &nh)
  {
    node_ = nh;

    goal_point_pub_ = nh.advertise<visualization_msgs::Marker>("goal_point", 2);
    global_list_pub_ = nh.advertise<visualization_msgs::Marker>("global_list", 2);
    initial_list_pub_ = nh.advertise<visualization_msgs::Marker>("init_list", 2);
    optimal_list_pub_ = nh.advertise<visualization_msgs::Marker>("optimal_list", 2);
    failed_list_pub_ = nh.advertise<visualization_msgs::Marker>("failed_list", 2);
    a_star_list_pub_ = nh.advertise<visualization_msgs::Marker>("a_star_list", 20);

    // intermediate_point_0_pub_ = nh.advertise<visualization_msgs::Marker>("pt0_dur_opt", 10);
    // intermediate_gradient_0_pub_ = nh.advertise<visualization_msgs::MarkerArray>("grad0_dur_opt", 10);
    // intermediate_point_1_pub_ = nh.advertise<visualization_msgs::Marker>("pt1_dur_opt", 10);
    // intermediate_gradient_1_pub_ = nh.advertise<visualization_msgs::MarkerArray>("grad1_dur_opt", 10);
    // intermediate_smoothness_gradient_pub_ = nh.advertise<visualization_msgs::MarkerArray>("smoo_grad_dur_opt", 10);
    // intermediate_distance_gradient_pub_ = nh.advertise<visualization_msgs::MarkerArray>("dist_grad_dur_opt", 10);
    // intermediate_feasibility_gradient_pub_ = nh.advertise<visualization_msgs::MarkerArray>("feas_grad_dur_opt", 10);
    // intermediate_swarm_gradient_pub_ = nh.advertise<visualization_msgs::MarkerArray>("swarm_grad_dur_opt", 10);
  }

  // // real ids used: {id, id+1000}
  void PlanningVisualization::DisplayMarkerList(ros::Publisher &publisher, const std::vector<Eigen::Vector3d> &points, double scale,
                                                Eigen::Vector4d color, int id, bool show_sphere /* = true */ )
  {
    visualization_msgs::Marker sphere, line_strip;
    sphere.header.frame_id = line_strip.header.frame_id = "world";
    sphere.header.stamp = line_strip.header.stamp = ros::Time::now();
    sphere.type = visualization_msgs::Marker::SPHERE_LIST;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
    sphere.id = id;
    line_strip.id = id + 1000;

    sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
    sphere.color.r = line_strip.color.r = color(0);
    sphere.color.g = line_strip.color.g = color(1);
    sphere.color.b = line_strip.color.b = color(2);
    sphere.color.a = line_strip.color.a = color(3) > 1e-5 ? color(3) : 1.0;
    sphere.scale.x = scale;
    sphere.scale.y = scale;
    sphere.scale.z = scale;
    line_strip.scale.x = scale / 2;
    geometry_msgs::Point pt;
    for (int i = 0; i < int(points.size()); i++)
    {
      pt.x = points[i](0);
      pt.y = points[i](1);
      pt.z = points[i](2);
      if (show_sphere) sphere.points.push_back(pt);
      line_strip.points.push_back(pt);
    }
    if (show_sphere) publisher.publish(sphere);
    publisher.publish(line_strip);
  }

  // real ids used: {id, id+1}
  void PlanningVisualization::GeneratePathDisplayArray(visualization_msgs::MarkerArray &array,
                                                       const std::vector<Eigen::Vector3d> &points, double scale, Eigen::Vector4d color, int id)
  {
    visualization_msgs::Marker sphere, line_strip;
    sphere.header.frame_id = line_strip.header.frame_id = "world";
    sphere.header.stamp = line_strip.header.stamp = ros::Time::now();
    sphere.type = visualization_msgs::Marker::SPHERE_LIST;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
    sphere.id = id;
    line_strip.id = id + 1;

    sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
    sphere.color.r = line_strip.color.r = color(0);
    sphere.color.g = line_strip.color.g = color(1);
    sphere.color.b = line_strip.color.b = color(2);
    sphere.color.a = line_strip.color.a = color(3) > 1e-5 ? color(3) : 1.0;
    sphere.scale.x = scale;
    sphere.scale.y = scale;
    sphere.scale.z = scale;
    line_strip.scale.x = scale / 3;
    geometry_msgs::Point pt;
    for (int i = 0; i < int(points.size()); i++)
    {
      pt.x = points[i](0);
      pt.y = points[i](1);
      pt.z = points[i](2);
      sphere.points.push_back(pt);
      line_strip.points.push_back(pt);
    }
    array.markers.push_back(sphere);
    array.markers.push_back(line_strip);
  }

  // real ids used: {1000*id ~ (arrow nums)+1000*id}
  void PlanningVisualization::GenerateArrowDisplayArray(visualization_msgs::MarkerArray &array,
                                                        const std::vector<Eigen::Vector3d> &points, double scale, Eigen::Vector4d color, int id)
  {
    visualization_msgs::Marker arrow;
    arrow.header.frame_id = "world";
    arrow.header.stamp = ros::Time::now();
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.action = visualization_msgs::Marker::ADD;

    // geometry_msgs::Point start, end;
    // arrow.points

    arrow.color.r = color(0);
    arrow.color.g = color(1);
    arrow.color.b = color(2);
    arrow.color.a = color(3) > 1e-5 ? color(3) : 1.0;
    arrow.scale.x = scale;
    arrow.scale.y = 2 * scale;
    arrow.scale.z = 2 * scale;

    geometry_msgs::Point start, end;
    for (int i = 0; i < int(points.size() / 2); i++)
    {
      // arrow.color.r = color(0) / (1+i);
      // arrow.color.g = color(1) / (1+i);
      // arrow.color.b = color(2) / (1+i);

      start.x = points[2 * i](0);
      start.y = points[2 * i](1);
      start.z = points[2 * i](2);
      end.x = points[2 * i + 1](0);
      end.y = points[2 * i + 1](1);
      end.z = points[2 * i + 1](2);
      arrow.points.clear();
      arrow.points.push_back(start);
      arrow.points.push_back(end);
      arrow.id = i + id * 1000;

      array.markers.push_back(arrow);
    }
  }

  void PlanningVisualization::DisplayGoalPoint(Eigen::Vector3d goal_point, Eigen::Vector4d color, const double scale, int id)
  {
    visualization_msgs::Marker sphere;
    sphere.header.frame_id = "world";
    sphere.header.stamp = ros::Time::now();
    sphere.type = visualization_msgs::Marker::SPHERE;
    sphere.action = visualization_msgs::Marker::ADD;
    sphere.id = id;

    sphere.pose.orientation.w = 1.0;
    sphere.color.r = color(0);
    sphere.color.g = color(1);
    sphere.color.b = color(2);
    sphere.color.a = color(3);
    sphere.scale.x = scale;
    sphere.scale.y = scale;
    sphere.scale.z = scale;
    sphere.pose.position.x = goal_point(0);
    sphere.pose.position.y = goal_point(1);
    sphere.pose.position.z = goal_point(2);

    goal_point_pub_.publish(sphere);
  }

  void PlanningVisualization::DisplayGlobalPathList(std::vector<Eigen::Vector3d> global_points, const double scale, int id)
  {

    if (global_list_pub_.getNumSubscribers() == 0)
    {
      return;
    }

    Eigen::Vector4d color(0, 0.5, 0.5, 1);
    DisplayMarkerList(global_list_pub_, global_points, scale, color, id);
  }

  void PlanningVisualization::DisplayMultiInitialPathList(std::vector<std::vector<Eigen::Vector3d>> initial_trajectories, const double scale)
  {

    if (initial_list_pub_.getNumSubscribers() == 0)
    {
      return;
    }

    static int previous_marker_count = 0;

    for ( int id=0; id<previous_marker_count; id++ )
    {
      Eigen::Vector4d color(0, 0, 0, 0);
      std::vector<Eigen::Vector3d> blank;
      DisplayMarkerList(initial_list_pub_, blank, scale, color, id, false);
      ros::Duration(0.001).sleep();
    }
    previous_marker_count = 0;

    for ( int id=0; id<(int)initial_trajectories.size(); id++ )
    {
      Eigen::Vector4d color(0, 0, 1, 0.7);
      DisplayMarkerList(initial_list_pub_, initial_trajectories[id], scale, color, id, false);
      ros::Duration(0.001).sleep();
      previous_marker_count++;
    }

  }

  void PlanningVisualization::DisplayInitialPathList(std::vector<Eigen::Vector3d> initial_points, const double scale, int id)
  {

    if (initial_list_pub_.getNumSubscribers() == 0)
    {
      return;
    }

    Eigen::Vector4d color(0, 0, 1, 1);
    DisplayMarkerList(initial_list_pub_, initial_points, scale, color, id);
  }

  void PlanningVisualization::DisplayMultiOptimalPathList(std::vector<std::vector<Eigen::Vector3d>> optimal_trajectories, const double scale) // zxzxzx
  {

    if (optimal_list_pub_.getNumSubscribers() == 0)
    {
      return;
    }

    static int previous_marker_count = 0;

    for ( int id=0; id<previous_marker_count; id++ )
    {
      Eigen::Vector4d color(0, 0, 0, 0);
      std::vector<Eigen::Vector3d> blank;
      DisplayMarkerList(optimal_list_pub_, blank, scale, color, id + 10, false);
      ros::Duration(0.001).sleep();
    }
    previous_marker_count = 0;

    for ( int id=0; id<(int)optimal_trajectories.size(); id++ )
    {
      Eigen::Vector4d color(1, 0, 0, 0.7);
      DisplayMarkerList(optimal_list_pub_, optimal_trajectories[id], scale, color, id + 10, false);
      ros::Duration(0.001).sleep();
      previous_marker_count++;
    }

  }

  void PlanningVisualization::DisplayOptimalList(Eigen::MatrixXd optimal_points, int id)
  {

    if (optimal_list_pub_.getNumSubscribers() == 0)
    {
      return;
    }

    std::vector<Eigen::Vector3d> points;
    for (int i = 0; i < optimal_points.cols(); i++)
    {
      Eigen::Vector3d pt = optimal_points.col(i).transpose();
      points.push_back(pt);
    }
    Eigen::Vector4d color(1, 0, 0, 1);
    DisplayMarkerList(optimal_list_pub_, points, 0.15, color, id);
  }

  void PlanningVisualization::DisplayFailedList(Eigen::MatrixXd failed_points, int id)
  {

    if (failed_list_pub_.getNumSubscribers() == 0)
    {
      return;
    }

    std::vector<Eigen::Vector3d> points;
    for (int i = 0; i < failed_points.cols(); i++)
    {
      Eigen::Vector3d pt = failed_points.col(i).transpose();
      points.push_back(pt);
    }
    Eigen::Vector4d color(0.3, 0, 0, 1);
    DisplayMarkerList(failed_list_pub_, points, 0.15, color, id);
  }

  void PlanningVisualization::DisplayAStarList(std::vector<std::vector<Eigen::Vector3d>> a_star_paths, int id /* = Eigen::Vector4d(0.5,0.5,0,1)*/)
  {

    if (a_star_list_pub_.getNumSubscribers() == 0)
    {
      return;
    }

    int i = 0;
    std::vector<Eigen::Vector3d> points;

    Eigen::Vector4d color = Eigen::Vector4d(0.5 + ((double)rand() / RAND_MAX / 2), 0.5 + ((double)rand() / RAND_MAX / 2), 0, 1); // make the A star pathes different every time.
    double scale = 0.05 + (double)rand() / RAND_MAX / 10;

    for (auto block : a_star_paths)
    {
      points.clear();
      for (auto pt : block)
      {
        points.push_back(pt);
      }
      //Eigen::Vector4d color(0.5,0.5,0,1);
      DisplayMarkerList(a_star_list_pub_, points, scale, color, id + i); // real ids used: [ id ~ id+a_star_paths.size() ]
      i++;
    }
  }

  void PlanningVisualization::DisplayArrowList(ros::Publisher &publisher, const std::vector<Eigen::Vector3d> &points, double scale, Eigen::Vector4d color, int id)
  {
    visualization_msgs::MarkerArray array;
    // clear
    publisher.publish(array);

    GenerateArrowDisplayArray(array, points, scale, color, id);

    publisher.publish(array);
  }

  void PlanningVisualization::DisplayIntermediatePoint(std::string type, Eigen::MatrixXd &points, int id, Eigen::Vector4d color)
  {
    std::vector<Eigen::Vector3d> point_list;
    point_list.reserve(points.cols());
    for ( int i=0; i<points.cols(); i++ )
    {
      point_list.emplace_back(points.col(i));
    }

    if ( !type.compare("0") )
    {
      DisplayMarkerList(intermediate_point_0_pub_, point_list, 0.1, color, id);
    }
    else if ( !type.compare("1") )
    {
      DisplayMarkerList(intermediate_point_1_pub_, point_list, 0.1, color, id);
    }
  }

  void PlanningVisualization::DisplayIntermediateGradient(std::string type, Eigen::MatrixXd &points, Eigen::MatrixXd &gradient, int id, Eigen::Vector4d color)
  {
    if ( points.cols() != gradient.cols() )
    {
      ROS_ERROR("pts.cols() != grad.cols()");
      return;
    }
    std::vector<Eigen::Vector3d> arrows;
    arrows.reserve(points.cols()*2);
    if ( !type.compare("swarm") )
    {
      for ( int i=0; i<points.cols(); i++ )
      {
        arrows.emplace_back(points.col(i));
        arrows.emplace_back(gradient.col(i));
      }
    }
    else
    {
      for ( int i=0; i<points.cols(); i++ )
      {
        arrows.emplace_back(points.col(i));
        arrows.emplace_back(points.col(i)+gradient.col(i));
      }
    }
    

    if ( !type.compare("grad0") )
    {
      DisplayArrowList(intermediate_gradient_0_pub_, arrows, 0.05, color, id);
    }
    else if ( !type.compare("grad1") )
    {
      DisplayArrowList(intermediate_gradient_1_pub_, arrows, 0.05, color, id);
    }
    else if ( !type.compare("dist") )
    {
      DisplayArrowList(intermediate_distance_gradient_pub_, arrows, 0.05, color, id);
    }
    else if ( !type.compare("smoo") )
    {
      DisplayArrowList(intermediate_smoothness_gradient_pub_, arrows, 0.05, color, id);
    }
    else if ( !type.compare("feas") )
    {
      DisplayArrowList(intermediate_feasibility_gradient_pub_, arrows, 0.05, color, id);
    }
    else if ( !type.compare("swarm") )
    {
      DisplayArrowList(intermediate_swarm_gradient_pub_, arrows, 0.02, color, id);
    }
    
  }

  // PlanningVisualization::
} // namespace diff_planner
