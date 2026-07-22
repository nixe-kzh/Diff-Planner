#include <Eigen/Eigen>
#include <cmath>
#include <iostream>
#include <plan_env/raycast.h>

int Signum(int x) {
  return x == 0 ? 0 : x < 0 ? -1 : 1;
}

double Mod(double value, double modulus) {
  return fmod(fmod(value, modulus) + modulus, modulus);
}

double IntBound(double s, double ds) {
  // Find the smallest positive t such that s+t*ds is an integer.
  if (ds < 0) {
    return IntBound(-s, -ds);
  } else {
    s = Mod(s, 1);
    // problem is now s+t*ds = 1
    return (1 - s) / ds;
  }
}

// void Raycast(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const Eigen::Vector3d& min,
//              const Eigen::Vector3d& max, int& output_points_cnt, Eigen::Vector3d* output) {
//   //    std::cout << start << ' ' << end << std::endl;
//   // From "A Fast Voxel Traversal Algorithm for Ray Tracing"
//   // by John Amanatides and Andrew Woo, 1987
//   // <http://www.cse.yorku.ca/~amana/research/grid.pdf>
//   // <http://citeseer.ist.psu.edu/viewdoc/summary?doi=10.1.1.42.3443>
//   // Extensions to the described algorithm:
//   //   • Imposed a distance limit.
//   //   • The face passed through to reach the current cube is provided to
//   //     the callback.

//   // The foundation of this algorithm is a parameterized representation of
//   // the provided ray,
//   //                    origin + t * direction,
//   // except that t is not actually stored; rather, at any given point in the
//   // traversal, we keep track of the *greater* t values which we would have
//   // if we took a step sufficient to cross a cube boundary along that axis
//   // (i.e. change the integer part of the coordinate) in the variables
//   // tMaxX, tMaxY, and tMaxZ.

//   // Cube containing origin point.
//   int x = (int)std::floor(start.x());
//   int y = (int)std::floor(start.y());
//   int z = (int)std::floor(start.z());
//   int endX = (int)std::floor(end.x());
//   int endY = (int)std::floor(end.y());
//   int endZ = (int)std::floor(end.z());
//   Eigen::Vector3d direction = (end - start);
//   double maxDist = direction.squaredNorm();

//   // Break out direction vector.
//   double dx = endX - x;
//   double dy = endY - y;
//   double dz = endZ - z;

//   // Direction to increment x,y,z when stepping.
//   int stepX = (int)signum((int)dx);
//   int stepY = (int)signum((int)dy);
//   int stepZ = (int)signum((int)dz);

//   // See description above. The initial values depend on the fractional
//   // part of the origin.
//   double tMaxX = intbound(start.x(), dx);
//   double tMaxY = intbound(start.y(), dy);
//   double tMaxZ = intbound(start.z(), dz);

//   // The change in t when taking a step (always positive).
//   double tDeltaX = ((double)stepX) / dx;
//   double tDeltaY = ((double)stepY) / dy;
//   double tDeltaZ = ((double)stepZ) / dz;

//   // Avoids an infinite loop.
//   if (stepX == 0 && stepY == 0 && stepZ == 0) return;

//   double dist = 0;
//   while (true) {
//     if (x >= min.x() && x < max.x() && y >= min.y() && y < max.y() && z >= min.z() && z < max.z()) {
//       output[output_points_cnt](0) = x;
//       output[output_points_cnt](1) = y;
//       output[output_points_cnt](2) = z;

//       output_points_cnt++;
//       dist = sqrt((x - start(0)) * (x - start(0)) + (y - start(1)) * (y - start(1)) +
//                   (z - start(2)) * (z - start(2)));

//       if (dist > maxDist) return;

//       /*            if (output_points_cnt > 1500) {
//                       std::cerr << "Error, too many racyast voxels." <<
//          std::endl;
//                       throw std::out_of_range("Too many raycast voxels");
//                   }*/
//     }

//     if (x == endX && y == endY && z == endZ) break;

//     // tMaxX stores the t-value at which we cross a cube boundary along the
//     // X axis, and similarly for Y and Z. Therefore, choosing the least tMax
//     // chooses the closest cube boundary. Only the first case of the four
//     // has been commented in detail.
//     if (tMaxX < tMaxY) {
//       if (tMaxX < tMaxZ) {
//         // Update which cube we are now in.
//         x += stepX;
//         // Adjust tMaxX to the next X-oriented boundary crossing.
//         tMaxX += tDeltaX;
//       } else {
//         z += stepZ;
//         tMaxZ += tDeltaZ;
//       }
//     } else {
//       if (tMaxY < tMaxZ) {
//         y += stepY;
//         tMaxY += tDeltaY;
//       } else {
//         z += stepZ;
//         tMaxZ += tDeltaZ;
//       }
//     }
//   }
// }

// void Raycast(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const Eigen::Vector3d& min,
//              const Eigen::Vector3d& max, std::vector<Eigen::Vector3d>* output) {
//   //    std::cout << start << ' ' << end << std::endl;
//   // From "A Fast Voxel Traversal Algorithm for Ray Tracing"
//   // by John Amanatides and Andrew Woo, 1987
//   // <http://www.cse.yorku.ca/~amana/research/grid.pdf>
//   // <http://citeseer.ist.psu.edu/viewdoc/summary?doi=10.1.1.42.3443>
//   // Extensions to the described algorithm:
//   //   • Imposed a distance limit.
//   //   • The face passed through to reach the current cube is provided to
//   //     the callback.

//   // The foundation of this algorithm is a parameterized representation of
//   // the provided ray,
//   //                    origin + t * direction,
//   // except that t is not actually stored; rather, at any given point in the
//   // traversal, we keep track of the *greater* t values which we would have
//   // if we took a step sufficient to cross a cube boundary along that axis
//   // (i.e. change the integer part of the coordinate) in the variables
//   // tMaxX, tMaxY, and tMaxZ.

//   // Cube containing origin point.
//   int x = (int)std::floor(start.x());
//   int y = (int)std::floor(start.y());
//   int z = (int)std::floor(start.z());
//   int endX = (int)std::floor(end.x());
//   int endY = (int)std::floor(end.y());
//   int endZ = (int)std::floor(end.z());
//   Eigen::Vector3d direction = (end - start);
//   double maxDist = direction.squaredNorm();

//   // Break out direction vector.
//   double dx = endX - x;
//   double dy = endY - y;
//   double dz = endZ - z;

//   // Direction to increment x,y,z when stepping.
//   int stepX = (int)signum((int)dx);
//   int stepY = (int)signum((int)dy);
//   int stepZ = (int)signum((int)dz);

//   // See description above. The initial values depend on the fractional
//   // part of the origin.
//   double tMaxX = intbound(start.x(), dx);
//   double tMaxY = intbound(start.y(), dy);
//   double tMaxZ = intbound(start.z(), dz);

//   // The change in t when taking a step (always positive).
//   double tDeltaX = ((double)stepX) / dx;
//   double tDeltaY = ((double)stepY) / dy;
//   double tDeltaZ = ((double)stepZ) / dz;

//   output->clear();

//   // Avoids an infinite loop.
//   if (stepX == 0 && stepY == 0 && stepZ == 0) return;

//   double dist = 0;
//   while (true) {
//     if (x >= min.x() && x < max.x() && y >= min.y() && y < max.y() && z >= min.z() && z < max.z()) {
//       output->push_back(Eigen::Vector3d(x, y, z));

//       dist = (Eigen::Vector3d(x, y, z) - start).squaredNorm();

//       if (dist > maxDist) return;

//       if (output->size() > 1500) {
//         std::cerr << "Error, too many racyast voxels." << std::endl;
//         throw std::out_of_range("Too many raycast voxels");
//       }
//     }

//     if (x == endX && y == endY && z == endZ) break;

//     // tMaxX stores the t-value at which we cross a cube boundary along the
//     // X axis, and similarly for Y and Z. Therefore, choosing the least tMax
//     // chooses the closest cube boundary. Only the first case of the four
//     // has been commented in detail.
//     if (tMaxX < tMaxY) {
//       if (tMaxX < tMaxZ) {
//         // Update which cube we are now in.
//         x += stepX;
//         // Adjust tMaxX to the next X-oriented boundary crossing.
//         tMaxX += tDeltaX;
//       } else {
//         z += stepZ;
//         tMaxZ += tDeltaZ;
//       }
//     } else {
//       if (tMaxY < tMaxZ) {
//         y += stepY;
//         tMaxY += tDeltaY;
//       } else {
//         z += stepZ;
//         tMaxZ += tDeltaZ;
//       }
//     }
//   }
// }

bool RayCaster::SetInput(const Eigen::Vector3d& start,
                         const Eigen::Vector3d& end /* , const Eigen::Vector3d& min,
                         const Eigen::Vector3d& max */) {
  start_ = start;
  end_ = end;
  // max_ = max;
  // min_ = min;

  x_ = (int)std::floor(start_.x());
  y_ = (int)std::floor(start_.y());
  z_ = (int)std::floor(start_.z());
  end_x_ = (int)std::floor(end_.x());
  end_y_ = (int)std::floor(end_.y());
  end_z_ = (int)std::floor(end_.z());
  direction_ = (end_ - start_);
  max_distance_ = direction_.squaredNorm();

  // Break out direction vector.
  dx_ = end_x_ - x_;
  dy_ = end_y_ - y_;
  dz_ = end_z_ - z_;

  // Direction to increment x,y,z when stepping.
  step_x_ = Signum((int)dx_);
  step_y_ = Signum((int)dy_);
  step_z_ = Signum((int)dz_);

  // See description above. The initial values depend on the fractional
  // part of the origin.
  max_x_time_ = IntBound(start_.x(), dx_);
  max_y_time_ = IntBound(start_.y(), dy_);
  max_z_time_ = IntBound(start_.z(), dz_);

  // The change in t when taking a step (always positive).
  delta_x_time_ = ((double)step_x_) / dx_;
  delta_y_time_ = ((double)step_y_) / dy_;
  delta_z_time_ = ((double)step_z_) / dz_;

  dist_ = 0;

  step_num_ = 0;

  // Avoids an infinite loop.
  if (step_x_ == 0 && step_y_ == 0 && step_z_ == 0)
    return false;
  else
    return true;
}

bool RayCaster::Step(Eigen::Vector3d& ray_point) {
  // if (x_ >= min_.x() && x_ < max_.x() && y_ >= min_.y() && y_ < max_.y() &&
  // z_ >= min_.z() && z_ <
  // max_.z())
  ray_point = Eigen::Vector3d(x_, y_, z_);

  // step_num_++;

  // dist_ = (Eigen::Vector3d(x_, y_, z_) - start_).squaredNorm();

  if (x_ == end_x_ && y_ == end_y_ && z_ == end_z_) {
    return false;
  }

  // if (dist_ > maxDist_)
  // {
  //   return false;
  // }

  // tMaxX stores the t-value at which we cross a cube boundary along the
  // X axis, and similarly for Y and Z. Therefore, choosing the least tMax
  // chooses the closest cube boundary. Only the first case of the four
  // has been commented in detail.
  if (max_x_time_ < max_y_time_) {
    if (max_x_time_ < max_z_time_) {
      // Update which cube we are now in.
      x_ += step_x_;
      // Adjust tMaxX to the next X-oriented boundary crossing.
      max_x_time_ += delta_x_time_;
    } else {
      z_ += step_z_;
      max_z_time_ += delta_z_time_;
    }
  } else {
    if (max_y_time_ < max_z_time_) {
      y_ += step_y_;
      max_y_time_ += delta_y_time_;
    } else {
      z_ += step_z_;
      max_z_time_ += delta_z_time_;
    }
  }

  return true;
}
