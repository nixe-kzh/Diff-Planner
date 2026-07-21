#ifndef DIFF_PLANNER_PLAN_ENV_INCLUDE_PLAN_ENV_RAYCAST_H_
#define DIFF_PLANNER_PLAN_ENV_INCLUDE_PLAN_ENV_RAYCAST_H_

#include <Eigen/Eigen>
#include <vector>

int Signum(int x);

double Mod(double value, double modulus);

double IntBound(double s, double ds);

// void Raycast(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const Eigen::Vector3d& min,
//              const Eigen::Vector3d& max, int& output_points_cnt, Eigen::Vector3d* output);

// void Raycast(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const Eigen::Vector3d& min,
//              const Eigen::Vector3d& max, std::vector<Eigen::Vector3d>* output);

class RayCaster {
private:
  /* data */
  Eigen::Vector3d start_;
  Eigen::Vector3d end_;
  Eigen::Vector3d direction_;
  Eigen::Vector3d min_;
  Eigen::Vector3d max_;
  int x_;
  int y_;
  int z_;
  int end_x_;
  int end_y_;
  int end_z_;
  double max_distance_;
  double dx_;
  double dy_;
  double dz_;
  int step_x_;
  int step_y_;
  int step_z_;
  double max_x_time_;
  double max_y_time_;
  double max_z_time_;
  double delta_x_time_;
  double delta_y_time_;
  double delta_z_time_;
  double dist_;

  int step_num_;

public:
  RayCaster(/* args */) {
  }
  ~RayCaster() {
  }

  bool SetInput(const Eigen::Vector3d& start,
                const Eigen::Vector3d& end /* , const Eigen::Vector3d& min,
                const Eigen::Vector3d& max */);

  bool Step(Eigen::Vector3d& ray_point);
};

#endif  // DIFF_PLANNER_PLAN_ENV_INCLUDE_PLAN_ENV_RAYCAST_H_
