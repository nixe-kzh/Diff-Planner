#include "drone_detector/drone_detector.h"

// STD
#include <string>

namespace detect {

DroneDetector::DroneDetector(ros::NodeHandle& node_handle)
    : nh_(node_handle)
{
  ReadParameters();

  // depth_image_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(nh_, "depth", 50, ros::TransportHints().tcpNoDelay()));
  // color_depth_image_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(nh_, "colordepth", 50));
  // camera_pose_sub_.reset(new message_filters::Subscriber<geometry_msgs::PoseStamped>(nh_, "camera_pose", 50));

  odometry_sub_ = nh_.subscribe("odometry", 100,
                                &DroneDetector::OdometryCallback, this,
                                ros::TransportHints().tcpNoDelay());
  depth_image_sub_ = nh_.subscribe("depth", 50,
                                   &DroneDetector::DepthImageCallback, this,
                                   ros::TransportHints().tcpNoDelay());
  // depth_color_image_pose_synchronizer_->registerCallback(boost::bind(&DroneDetector::DepthColorCameraPoseCallback, this, _1, _2, _3));

  // drone0_odometry_sub_ = nh_.subscribe("drone0", 50, &DroneDetector::Drone0OdometryCallback, this);
  // drone1_odometry_sub_ = nh_.subscribe("drone1", 50, &DroneDetector::Drone1OdometryCallback, this);
  // drone2_odometry_sub_ = nh_.subscribe("drone2", 50, &DroneDetector::Drone2OdometryCallback, this);
  other_drone_odometry_sub_ = nh_.subscribe(
      "/others_odom", 100, &DroneDetector::OtherDroneOdometryCallback, this,
      ros::TransportHints().tcpNoDelay());

  new_depth_image_pub_ =
      nh_.advertise<sensor_msgs::Image>("new_depth_image", 50);
  debug_depth_image_pub_ =
      nh_.advertise<sensor_msgs::Image>("debug_depth_image", 50);

  debug_info_pub_ = nh_.advertise<std_msgs::String>("/debug_info", 50);

  camera_to_body_ << 0.0, 0.0, 1.0, 0.0,
      -1.0, 0.0, 0.0, 0.0,
      0.0, -1.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 1.0;

  // init drone_pose_err_pub
  for(int i = 0; i < kMaxDroneCount; i++) {
    if(i != my_id_)
      drone_pose_error_pub_[i] = nh_.advertise<geometry_msgs::PoseStamped>("drone"+std::to_string(i)+"to"+std::to_string(my_id_)+"_pose_err", 50);
  }

  ROS_INFO("Successfully launched node.");
}

DroneDetector::~DroneDetector()
{
}

void DroneDetector::ReadParameters()
{
  // camera params
  nh_.getParam("cam_width", image_width_);
  nh_.getParam("cam_height", image_height_);
  nh_.getParam("cam_fx", fx_);
  nh_.getParam("cam_fy", fy_);
  nh_.getParam("cam_cx", cx_);
  nh_.getParam("cam_cy", cy_);

  // 
  nh_.getParam("debug_flag", debug_flag_);
  nh_.getParam("pixel_ratio", pixel_ratio_);
  nh_.getParam("my_id", my_id_);
  nh_.getParam("estimate/drone_width", drone_width_);
  nh_.getParam("estimate/drone_height", drone_height_);
  nh_.getParam("estimate/max_pose_error", max_pose_error_);

  max_squared_pose_error_ = max_pose_error_*max_pose_error_;
}

// inline functions
inline double DroneDetector::SquaredDistance(const Eigen::Vector3d &p1,
                                             const Eigen::Vector3d &p2)
{
    double delta_x = p1(0)-p2(0);
    double delta_y = p1(1)-p2(1);
    double delta_z = p1(2)-p2(2);
    return delta_x*delta_x+delta_y*delta_y+delta_z*delta_z;
}

inline double DroneDetector::SquaredDistance(const Eigen::Vector4d &p1,
                                             const Eigen::Vector4d &p2)
{
    double delta_x = p1(0)-p2(0);
    double delta_y = p1(1)-p2(1);
    double delta_z = p1(2)-p2(2);
    return delta_x*delta_x+delta_y*delta_y+delta_z*delta_z;
}

inline Eigen::Vector4d DroneDetector::DepthToPosition(int u, int v,
                                                      float depth)
{
  Eigen::Vector4d pose_in_camera;
  pose_in_camera(0) = (u - cx_) * depth / fx_;
  pose_in_camera(1) = (v - cy_) * depth / fy_;
  pose_in_camera(2) = depth; 
  pose_in_camera(3) = 1.0;
  return pose_in_camera;
}

inline Eigen::Vector4d DroneDetector::DepthToPosition(
    const Eigen::Vector2i &pixel, float depth)
{
  Eigen::Vector4d pose_in_camera;
  pose_in_camera(0) = (pixel(0) - cx_) * depth / fx_;
  pose_in_camera(1) = (pixel(1) - cy_) * depth / fy_;
  pose_in_camera(2) = depth; 
  pose_in_camera(3) = 1.0;
  return pose_in_camera;
}

inline Eigen::Vector2i DroneDetector::PositionToDepthPixel(
    const Eigen::Vector4d &position_in_camera)
{
  float depth = position_in_camera(2);
  Eigen::Vector2i pixel;
  pixel(0) = position_in_camera(0) * fx_ / depth + cx_ + 0.5;
  pixel(1) = position_in_camera(1) * fy_ / depth + cy_ + 0.5;
  return pixel;
}

inline bool DroneDetector::IsInSensorRange(const Eigen::Vector2i &pixel)
{
  if (pixel(0)>=0 && pixel(1) >= 0 && pixel(0) <= image_width_ && pixel(1) <= image_height_) return true;
  else 
    return false;
}

void DroneDetector::OdometryCallback(const nav_msgs::Odometry& odometry)
{
  odometry_ = odometry;
  Eigen::Matrix4d body_to_world = Eigen::Matrix4d::Identity();

  my_pose_world_(0) = odometry.pose.pose.position.x;
  my_pose_world_(1) = odometry.pose.pose.position.y;
  my_pose_world_(2) = odometry.pose.pose.position.z;
  my_pose_world_(3) = 1.0;
  my_attitude_world_.x() = odometry.pose.pose.orientation.x;
  my_attitude_world_.y() = odometry.pose.pose.orientation.y;
  my_attitude_world_.z() = odometry.pose.pose.orientation.z;
  my_attitude_world_.w() = odometry.pose.pose.orientation.w;
  body_to_world.block<3,3>(0,0) = my_attitude_world_.toRotationMatrix();
  body_to_world(0,3) = my_pose_world_(0);
  body_to_world(1,3) = my_pose_world_(1);
  body_to_world(2,3) = my_pose_world_(2);

  //convert to cam pose
  camera_to_world_ = body_to_world * camera_to_body_;
  camera_to_world_quaternion_ = camera_to_world_.block<3,3>(0,0);

  // my_last_odom_stamp_ = odom.header.stamp;

  // my_last_pose_world_(0) = odom.pose.pose.position.x;
  // my_last_pose_world_(1) = odom.pose.pose.position.y;
  // my_last_pose_world_(2) = odom.pose.pose.position.z;
  // my_last_pose_world_(3) = 1.0;

  //publish tf
  // static tf::TransformBroadcaster br;
  // tf::Transform transform;
  // transform.setOrigin( tf::Vector3(cam2world(0,3), cam2world(1,3), cam2world(2,3) ));
  // transform.setRotation(tf::Quaternion(cam2world_quat.x(), cam2world_quat.y(), cam2world_quat.z(), cam2world_quat.w()));
  // br.sendTransform(tf::StampedTransform(transform, my_last_odom_stamp, "world", "camera")); 
  //publish transform from world frame to quadrotor frame.
}
void DroneDetector::DepthImageCallback(
    const sensor_msgs::ImageConstPtr& depth_image)
{
  /* get depth image */
  cv_bridge::CvImagePtr image;
  image = cv_bridge::toCvCopy(depth_image, depth_image->encoding);
  image->image.copyTo(depth_image_);

  debug_start_time_ = ros::Time::now();

  Eigen::Vector2i detected_pixels[kMaxDroneCount];
  for (int i = 0; i < kMaxDroneCount; i++) {
    if (in_depth_[i]) {
      Detect(i, detected_pixels[i]);
    }
  }   

  cv_bridge::CvImage output_image;
  for (int i = 0; i < kMaxDroneCount; i++) {
    if (in_depth_[i]) {
      // erase hit pixels in depth
      for(int k = 0; k < int(hit_pixels_[i].size()); k++) {
        // depth_image_.at<float>(hit_pixels_[i][k](1), hit_pixels_[i][k](0)) = 0;
        uint16_t *row_ptr;
        row_ptr = depth_image_.ptr<uint16_t>(hit_pixels_[i][k](1));
        (*(row_ptr+hit_pixels_[i][k](0))) = 0.0;
      } 
    }
  }  
  debug_end_time_ = ros::Time::now();
  // ROS_WARN("cost_total_time = %lf", (debug_end_time_ - debug_start_time_).toSec()*1000.0);
  output_image.header = depth_image->header;
  output_image.encoding = depth_image->encoding;
  output_image.image = depth_image_.clone();
  new_depth_image_pub_.publish(output_image.toImageMsg());

  std_msgs::String message;
  std::stringstream message_stream;
  if(debug_flag_) {
    for (int i = 0; i < kMaxDroneCount; i++) {
      if (in_depth_[i]) {
        // add bound box in colormap
        // cv::Rect rect(_bbox_lu.x, _bbox_lu.y, _bbox_rd.x, _bbox_rd.y);//左上坐标（x,y）和矩形的长(x)宽(y)
        cv::rectangle(depth_image_, cv::Rect(search_box_upper_left_[i], search_box_lower_right_[i]), cv::Scalar(0, 0, 0), 5, cv::LINE_8, 0);
        cv::rectangle(depth_image_, cv::Rect(bounding_box_upper_left_[i], bounding_box_lower_right_[i]), cv::Scalar(0, 0, 0), 5, cv::LINE_8, 0);
        if (debug_detection_result_[i] == 1) {
          message_stream << "no enough " << hit_pixels_[i].size();
        } else if(debug_detection_result_[i] == 2) {
          message_stream << "success";
        }
      } else {
        message_stream << "no detect";
      }
    } 
      output_image.header = depth_image->header;
      output_image.encoding = depth_image->encoding;
      output_image.image = depth_image_.clone();
      debug_depth_image_pub_.publish(output_image.toImageMsg());
      message.data = message_stream.str();
      debug_info_pub_.publish(message);
  }
}

void DroneDetector::DroneOdometryCallback(
    const nav_msgs::Odometry& odometry, int drone_id)
{
  if (drone_id == my_id_) {
    return;
  }
  Eigen::Matrix4d drone_to_world = Eigen::Matrix4d::Identity();
  drone_pose_world_[drone_id](0) = odometry.pose.pose.position.x;
  drone_pose_world_[drone_id](1) = odometry.pose.pose.position.y;
  drone_pose_world_[drone_id](2) = odometry.pose.pose.position.z;
  drone_pose_world_[drone_id](3) = 1.0;

  drone_attitude_world_[drone_id].x() = odometry.pose.pose.orientation.x;
  drone_attitude_world_[drone_id].y() = odometry.pose.pose.orientation.y;
  drone_attitude_world_[drone_id].z() = odometry.pose.pose.orientation.z;
  drone_attitude_world_[drone_id].w() = odometry.pose.pose.orientation.w;
  drone_to_world.block<3,3>(0,0) = drone_attitude_world_[drone_id].toRotationMatrix();
  
  drone_to_world(0,3) = drone_pose_world_[drone_id](0);
  drone_to_world(1,3) = drone_pose_world_[drone_id](1);
  drone_to_world(2,3) = drone_pose_world_[drone_id](2);

  drone_pose_camera_[drone_id] =
      camera_to_world_.inverse() * drone_pose_world_[drone_id];
  // if the drone is in sensor range
  drone_reference_pixel_[drone_id] =
      PositionToDepthPixel(drone_pose_camera_[drone_id]);
  if (drone_pose_camera_[drone_id](2) > 0 &&
      IsInSensorRange(drone_reference_pixel_[drone_id])) {
    in_depth_[drone_id] = true;
  } else {
    in_depth_[drone_id] = false;
    debug_detection_result_[drone_id] = 0;
  }
}

void DroneDetector::Drone0OdometryCallback(
    const nav_msgs::Odometry& odometry)
{
  DroneOdometryCallback(odometry, 0);
}

void DroneDetector::Drone1OdometryCallback(
    const nav_msgs::Odometry& odometry)
{
  DroneOdometryCallback(odometry, 1);
}

void DroneDetector::Drone2OdometryCallback(
    const nav_msgs::Odometry& odometry)
{
  DroneOdometryCallback(odometry, 2);
}

void DroneDetector::OtherDroneOdometryCallback(
    const nav_msgs::Odometry& odometry)
{
  std::string drone_id_text = odometry.child_frame_id.substr(6);
  try
  {
    int drone_id = std::stoi(drone_id_text);
    DroneOdometryCallback(odometry, drone_id);
  }
  catch(const std::exception& e)
  {
    std::cout << e.what() << '\n';
  }
}

bool DroneDetector::CountPixels(
    int drone_id, Eigen::Vector2i &detected_pixel,
    Eigen::Vector4d &detected_position_camera)
{
  bounding_box_upper_left_[drone_id].x = image_width_;
  bounding_box_lower_right_[drone_id].x = 0;
  bounding_box_upper_left_[drone_id].y = image_height_;
  bounding_box_lower_right_[drone_id].y = 0;

  valid_pixel_count_[drone_id] = 0;
  hit_pixels_[drone_id].clear();

  Eigen::Vector2i candidate_pixel;
  Eigen::Vector4d candidate_position_camera;
  int search_radius =
      2 * max_pose_error_ * fx_ / drone_pose_camera_[drone_id](2);
  float depth;
  search_box_upper_left_[drone_id].x =
      drone_reference_pixel_[drone_id](0) - search_radius;
  search_box_upper_left_[drone_id].y =
      drone_reference_pixel_[drone_id](1) - search_radius;
  search_box_lower_right_[drone_id].x =
      drone_reference_pixel_[drone_id](0) + search_radius;
  search_box_lower_right_[drone_id].y =
      drone_reference_pixel_[drone_id](1) + search_radius;
  // check the tmp_p around ref_pixel
  for(int i = -search_radius; i <= search_radius; i++)
    for(int j = -search_radius; j <= search_radius; j++)
    {
      candidate_pixel(0) = drone_reference_pixel_[drone_id](0) + j;
      candidate_pixel(1) = drone_reference_pixel_[drone_id](1) + i;
      if(candidate_pixel(0) < 0 || candidate_pixel(0) >= image_width_ ||
         candidate_pixel(1) < 0 || candidate_pixel(1) >= image_height_)
        continue;
      // depth = depth_image_.at<float>(candidate_pixel(1), candidate_pixel(0));
      uint16_t *row_ptr;
      row_ptr = depth_image_.ptr<uint16_t>(candidate_pixel(1));
      depth = (*(row_ptr+candidate_pixel(0))) / 1000.0;
      // ROS_WARN("depth = %lf", depth);
      // get tmp_pose in cam frame
      candidate_position_camera =
          DepthToPosition(candidate_pixel(0), candidate_pixel(1), depth);
      double squared_distance =
          SquaredDistance(candidate_position_camera,
                          drone_pose_camera_[drone_id]);
      // ROS_WARN("dist2 = %lf", dist2);
      if (squared_distance < max_squared_pose_error_) {
        valid_pixel_count_[drone_id]++;
        hit_pixels_[drone_id].push_back(candidate_pixel);
        bounding_box_upper_left_[drone_id].x = candidate_pixel(0) < bounding_box_upper_left_[drone_id].x ? candidate_pixel(0) : bounding_box_upper_left_[drone_id].x;
        bounding_box_upper_left_[drone_id].y = candidate_pixel(1) < bounding_box_upper_left_[drone_id].y ? candidate_pixel(1) : bounding_box_upper_left_[drone_id].y;
        bounding_box_lower_right_[drone_id].x = candidate_pixel(0) > bounding_box_lower_right_[drone_id].x ? candidate_pixel(0) : bounding_box_lower_right_[drone_id].x;
        bounding_box_lower_right_[drone_id].y = candidate_pixel(1) > bounding_box_lower_right_[drone_id].y ? candidate_pixel(1) : bounding_box_lower_right_[drone_id].y;
      }
    } 
  pixel_threshold_ = (drone_width_*fx_/drone_pose_camera_[drone_id](2)) * (drone_height_*fy_/drone_pose_camera_[drone_id](2))*pixel_ratio_;
  if (valid_pixel_count_[drone_id] > pixel_threshold_) {
    int step = 1, size = (bounding_box_lower_right_[drone_id].y-bounding_box_upper_left_[drone_id].y) < (bounding_box_lower_right_[drone_id].x-bounding_box_upper_left_[drone_id].x) ? (bounding_box_lower_right_[drone_id].y-bounding_box_upper_left_[drone_id].y) : (bounding_box_lower_right_[drone_id].x-bounding_box_upper_left_[drone_id].x);
    int initial_x = (bounding_box_upper_left_[drone_id].x+bounding_box_lower_right_[drone_id].x)/2, initial_y = (bounding_box_upper_left_[drone_id].y+bounding_box_lower_right_[drone_id].y)/2;
    int x_flag = 1, y_flag = 1;
    int x_idx = 0, y_idx = 0;
    uint16_t *row_ptr;
    row_ptr = depth_image_.ptr<uint16_t>(candidate_pixel(1));
    depth = (*(row_ptr+candidate_pixel(0))) / 1000.0;
    candidate_position_camera = DepthToPosition(initial_x, initial_y, depth);
    if (SquaredDistance(candidate_position_camera,
                        drone_pose_camera_[drone_id]) <
        max_squared_pose_error_){
      detected_pixel(0) = initial_x;
      detected_pixel(1) = initial_y;
      detected_position_camera = candidate_position_camera;
      return true;
    }
    while(step<size) {
        while(x_idx<step){
            initial_x = initial_x+x_flag;
            uint16_t *row_ptr;
            row_ptr = depth_image_.ptr<uint16_t>(candidate_pixel(1));
            depth = (*(row_ptr+candidate_pixel(0))) / 1000.0;
            candidate_position_camera = DepthToPosition(initial_x, initial_y, depth);
            if (SquaredDistance(candidate_position_camera,
                                drone_pose_camera_[drone_id]) <
                max_squared_pose_error_) {
              detected_pixel(0) = initial_x;
              detected_pixel(1) = initial_y;
              detected_position_camera = candidate_position_camera;
              return true;
            }
            x_idx++;
        }
        x_idx = 0;
        x_flag = -x_flag;
        while(y_idx<step){
            initial_y = initial_y+y_flag;
            uint16_t *row_ptr;
            row_ptr = depth_image_.ptr<uint16_t>(candidate_pixel(1));
            depth = (*(row_ptr+candidate_pixel(0))) / 1000.0;
            candidate_position_camera = DepthToPosition(initial_x, initial_y, depth);
            if (SquaredDistance(candidate_position_camera,
                                drone_pose_camera_[drone_id]) <
                max_squared_pose_error_){
              detected_pixel(0) = initial_x;
              detected_pixel(1) = initial_y;
              detected_position_camera = candidate_position_camera;
              return true;
            }
            y_idx++;
        }
        y_idx = 0;
        y_flag = -y_flag;
        step++;
    }
    while(x_idx<step-1){
        initial_x = initial_x+x_flag;
        uint16_t *row_ptr;
        row_ptr = depth_image_.ptr<uint16_t>(candidate_pixel(1));
        depth = (*(row_ptr+candidate_pixel(0))) / 1000.0;
        candidate_position_camera = DepthToPosition(initial_x, initial_y, depth);
        if (SquaredDistance(candidate_position_camera,
                            drone_pose_camera_[drone_id]) <
            max_squared_pose_error_){
          detected_pixel(0) = initial_x;
          detected_pixel(1) = initial_y;
          detected_position_camera = candidate_position_camera;
          return true;
        }
        x_idx++;
    }  
  }
  return false;
}

void DroneDetector::Detect(int drone_id, Eigen::Vector2i &detected_pixel)
{
  Eigen::Vector4d detected_position_camera;
  Eigen::Vector4d pose_error;
  bool found =
      CountPixels(drone_id, detected_pixel, detected_position_camera);
  if (found) {
    // ROS_WARN("FOUND");
    pose_error = camera_to_world_*detected_position_camera - drone_pose_world_[drone_id];
    debug_detection_result_[drone_id] = 2;

    geometry_msgs::PoseStamped output_message;
    output_message.header.stamp = my_last_camera_stamp_;
    output_message.header.frame_id = "/drone_detect";
    output_message.pose.position.x = pose_error(0);
    output_message.pose.position.y = pose_error(1);
    output_message.pose.position.z = pose_error(2);
    drone_pose_error_pub_[drone_id].publish(output_message);

  } else {
    // ROS_WARN("NOT FOUND");
    debug_detection_result_[drone_id] = 1;
  }
}

void DroneDetector::Test() {
  ROS_WARN("my_id = %d", my_id_);
}

} /* namespace */
