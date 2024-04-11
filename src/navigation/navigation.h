//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    navigation.h
\brief   Interface for reference Navigation class.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <vector>

#include "eigen3/Eigen/Dense"
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include "vector_map/vector_map.h"
#include "simple_queue.h"

#ifndef NAVIGATION_H
#define NAVIGATION_H

namespace ros {
  class NodeHandle;
}  // namespace ros

namespace navigation {

struct Priority{
  static constexpr float Epsilon = 1e-6;
  float cost;
  float huer;

  Priority() {}

  Priority(float cost, float huer) : cost(cost), huer(huer) {}

  bool operator<(const Priority &other) const
  {
    float f = cost + huer;
    float f_other = other.cost + other.huer;
    if (f > f_other + Epsilon)
      return true;
    if (f < f_other - Epsilon)
      return false;
    return (cost < other.cost);
  }

  // bool operator=(const Priority &other) const
  // {
  //   return (other.cost == cost ) && (other.huer == huer);
  // }

  bool operator>(const Priority &other) const
  {
    float f = cost + huer;
    float f_other = other.cost + other.huer;
    if (f < f_other - Epsilon)
      return true;
    if (f > f_other + Epsilon)
      return false;
    return (cost > other.cost);
  }
};

struct PathOption {
  float curvature;
  float clearance;
  float free_path_length;
  Eigen::Vector2f obstruction;
  Eigen::Vector2f closest_point;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

class Navigation {
 public:

   // Constructor
  explicit Navigation(const std::string& map_file, ros::NodeHandle* n);

  // Used in callback from localization to update position.
  void UpdateLocation(const Eigen::Vector2f& loc, float angle);

  // Used in callback for odometry messages to update based on odometry.
  void UpdateOdometry(const Eigen::Vector2f& loc,
                      float angle,
                      const Eigen::Vector2f& vel,
                      float ang_vel);

  // Updates based on an observed laser scan
  void ObservePointCloud(const std::vector<Eigen::Vector2f>& cloud,
                         double time);

  // Main function called continously from main
  void Run();
  // Used to set the next target pose.
  void SetNavGoal(const Eigen::Vector2f& loc, float angle);
  bool PlanStillValid();
  void Plan();
  Eigen::Vector2f GetCarrot();  
  void ObstacleAvoidance();
  void OneDTOC(PathOption p, Eigen::Vector2f stg);

  Eigen::Vector2f getStatefromKey(int key);

  // Function for A*
  bool AStarPlanner(Eigen::Vector2f start_loc, Eigen::Vector2f end_loc, std::vector<Eigen::Vector2f> *path);

  int getKeyFromState(Eigen::Vector2f s);

  float getHeuristic( Eigen::Vector2f s1,  Eigen::Vector2f s2);
  float getEdgeCost( Eigen::Vector2f s1,  Eigen::Vector2f s2);
  void getNeighbors(int s_key, std::vector<int>* neighbors);
  bool checkCollision(int s_key, int s1_key);
  int boundLocation(Eigen::Vector2i location, bool isX);
  void populateNeighbors(int x, int y, int MAP_WIDTH_,int  MAP_HEIGHT_, int s_key, std::vector<int>* neighbors );
  void SetOffset(Eigen::Vector2f loc);
  // bool PlanStillValid();

  

 private:

  // Whether odometry has been initialized.
  bool odom_initialized_;
  // Whether localization has been initialized.
  bool localization_initialized_;
  // Current robot location.
  Eigen::Vector2f robot_loc_;
  // Current robot orientation.
  float robot_angle_;
  // Current robot velocity.
  Eigen::Vector2f robot_vel_;
  // Current robot angular speed.
  float robot_omega_;
  // Odometry-reported robot location.
  Eigen::Vector2f odom_loc_;
  // Odometry-reported robot angle.
  float odom_angle_;
  // Odometry-reported robot starting location.
  Eigen::Vector2f odom_start_loc_;
  // Odometry-reported robot starting angle.
  float odom_start_angle_;
  // Latest observed point cloud.
  std::vector<Eigen::Vector2f> point_cloud_;

  // Whether navigation is complete.
  bool nav_complete_;
  // Navigation goal location.
  Eigen::Vector2f nav_goal_loc_;
  // Navigation goal angle.
  float nav_goal_angle_;
  // Map of the environment.
  vector_map::VectorMap map_;


  Eigen::Vector2f short_term_goal;

  // Car parameters
  const float ROBOT_WIDTH_;
  const float ROBOT_LENGTH_;
  const float MAX_CLEARANCE_;
  const float WHEEL_BASE_;
  const float MAX_CURVATURE_;
  const float MAX_ACCEL_;
  const float MAX_DEACCL_;
  const float MAX_SHORT_TERM_GOAL_;
  const float STOPPING_DISTANCE_;
  const float MAX_SPEED_;
  const float DELTA_T_;
  const float SYSTEM_LATENCY_;
  const float OBSTACLE_MARGIN_;

  // Navigation parameters
  const float RESOLUTION_;
  const uint64_t SIZE_X_;
  const uint64_t SIZE_Y_;
  const float DIAGONAL_LENGTH_;
  Eigen::Vector2f NAV_OFFSET_;
  const float NAV_MARGIN_;
  const Eigen::Vector2f ORIGIN_;
  float NAV_TOLERANCE_;
  const int MAX_EDGE_EXPANSIONS_;
  const bool VISUALIZE_;
  const float CARROT_DIST_SQ_;

  std::vector<Eigen::Vector2f> plan_path_;
  PathOption GetFreePathLength(PathOption p, Eigen::Vector2f stg);
};

}  // namespace navigation

#endif  // NAVIGATION_H
