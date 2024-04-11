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
\file    navigation.cc
\brief   Starter code for navigation.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================
#include <algorithm>
#include <string>
#include <vector>
#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "amrl_msgs/Pose2Df.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "ros/package.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "navigation.h"
#include "visualization/visualization.h"
using Eigen::Vector2f;
using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using std::string;
using std::vector;
using std::max;
using std::min;
using std::swap;

using namespace math_util;
using namespace ros_helpers;

namespace {
ros::Publisher drive_pub_;
ros::Publisher viz_pub_;
VisualizationMsg local_viz_msg_;
VisualizationMsg global_viz_msg_;
AckermannCurvatureDriveMsg drive_msg_;
// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;
const bool visualize = false;
} 

namespace navigation {

string GetMapFileFromName(const string& map) {
  string maps_dir_ = ros::package::getPath("amrl_maps");
  return maps_dir_ + "/" + map + "/" + map + ".vectormap.txt";
}

Navigation::Navigation(const string& map_name, ros::NodeHandle* n) :
    odom_initialized_(false),
    localization_initialized_(false),
    robot_loc_(0, 0),
    robot_angle_(0),
    robot_vel_(0, 0),
    robot_omega_(0),
    nav_complete_(true),
    nav_goal_loc_(0, 0),
    nav_goal_angle_(0),
    ROBOT_WIDTH_(0.281),
    ROBOT_LENGTH_(0.535),
    MAX_CLEARANCE_(0.5),
    WHEEL_BASE_(0.324),
    MAX_CURVATURE_(1.0),
    MAX_ACCEL_(4.0),
    MAX_DEACCL_(9.0),
    MAX_SHORT_TERM_GOAL_(5.0),
    STOPPING_DISTANCE_(0.1),
    MAX_SPEED_(1.0),
    DELTA_T_(0.05),
    SYSTEM_LATENCY_(0.23),
    OBSTACLE_MARGIN_(0.1),
    RESOLUTION_(0.5),
    SIZE_X_(800),
    SIZE_Y_(800),
    DIAGONAL_LENGTH_(std::sqrt(2.0f)),
    NAV_OFFSET_(0,0),
    NAV_MARGIN_(0.1),
    ORIGIN_(0,0),
    NAV_TOLERANCE_(0.5),
    MAX_EDGE_EXPANSIONS_(5000), 
    VISUALIZE_(false), 
    CARROT_DIST_SQ_(4)
    {  
    map_.Load(GetMapFileFromName(map_name));
    drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>(
        "ackermann_curvature_drive", 1);
    viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
    local_viz_msg_ = visualization::NewVisualizationMessage(
        "base_link", "navigation_local");
    global_viz_msg_ = visualization::NewVisualizationMessage(
        "map", "navigation_global");
    InitRosHeader("base_link", &drive_msg_.header);
}


void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
    nav_goal_loc_ = loc;
    nav_goal_angle_ = angle;
    nav_complete_ = false;  
}

void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
  localization_initialized_ = true;
  robot_loc_ = loc;
  robot_angle_ = angle;
}

void Navigation::UpdateOdometry(const Vector2f& loc,
                                float angle,
                                const Vector2f& vel,
                                float ang_vel) {
  robot_omega_ = ang_vel;
  robot_vel_ = vel;
  if (!odom_initialized_) {
    odom_start_angle_ = angle;
    odom_start_loc_ = loc;
    odom_initialized_ = true;
    odom_loc_ = loc;
    odom_angle_ = angle;
    return;
  }
  odom_loc_ = loc;
  odom_angle_ = angle;
}

void Navigation::ObservePointCloud(const vector<Vector2f>& cloud,
                                   double time) {
  point_cloud_ = cloud;

}

int Navigation::boundLocation(Eigen::Vector2i location, bool isX)
        {
            float MAP_WIDTH_ = SIZE_X_/RESOLUTION_;
            float MAP_HEIGHT_ = SIZE_Y_/RESOLUTION_ ;           
            int boundary = isX ? MAP_WIDTH_ : MAP_HEIGHT_;
            int value = isX ? location.x() : location.y();

            // Apply boundary conditions
            if (value < 0) {
                return 0;
            } else if (value > boundary) {
                return boundary;
            } else {
                return value;
            } 
        }


Eigen::Vector2f Navigation::getStatefromKey(int key){
      float MAP_WIDTH_ = SIZE_X_/RESOLUTION_;
      // float MAP_HEIGHT_ = SIZE_Y_/RESOLUTION_ ; 
      float x = RESOLUTION_ * (float)(key % (int)MAP_WIDTH_ );
      float y = RESOLUTION_ * (float)(key / MAP_WIDTH_);
      Eigen::Vector2f newState = Eigen::Vector2f(x + ORIGIN_.x(), y + ORIGIN_.y());
      return newState+NAV_OFFSET_;
      }

int Navigation::getKeyFromState(Eigen::Vector2f s) {

    float MAP_WIDTH_ = SIZE_X_/RESOLUTION_;
    // float MAP_HEIGHT_ = SIZE_Y_/RESOLUTION_ ; 
    Eigen::Vector2i location = ((s-ORIGIN_- NAV_OFFSET_)/RESOLUTION_).cast<int>();
    int keyValue = boundLocation(location, false) * MAP_WIDTH_ + boundLocation(location, true);
    return keyValue;
}

float Navigation::getHeuristic( Eigen::Vector2f s1,  Eigen::Vector2f s2)  {
            return ((s1 - s2).norm() / RESOLUTION_);
        }

float Navigation::getEdgeCost( Eigen::Vector2f s1,  Eigen::Vector2f s2) {
    bool x_same = s1.x() == s2.x();
    bool y_same = s1.y() == s2.y();
    if (x_same || y_same){
        return 1;
    }
    else{
        return DIAGONAL_LENGTH_;
    }
}

void Navigation::SetOffset(Eigen::Vector2f loc)
{
  NAV_OFFSET_ = Eigen::Vector2f(0, 0);
  NAV_OFFSET_ = loc -(getStatefromKey(getKeyFromState(loc)));
}


bool Navigation::checkCollision(int s_key, int s1_key){
  Eigen::Vector2f s = Navigation::getStatefromKey(s_key);
  Eigen::Vector2f s1 = Navigation::getStatefromKey(s1_key);

  for (const auto& l : map_.lines) {
      if (l.CloserThan(s, s1, NAV_MARGIN_)) return false;
  }
  return true;
}

void Navigation::populateNeighbors(int x, int y, int MAP_WIDTH_,int  MAP_HEIGHT_, int s_key, std::vector<int>* neighbors )
{
            bool x_plus = (x + 1 < MAP_WIDTH_);
            bool x_minus = (x > 0);
            bool y_plus = (y + 1 < MAP_HEIGHT_);
            bool y_minus = (y > 0);

            if(x_plus)
            {
                if(Navigation::checkCollision(s_key, s_key+1))
                neighbors->push_back(s_key + 1);
                
                if(y_plus)
                {
                if(Navigation::checkCollision(s_key, s_key + MAP_WIDTH_ + 1))
                    neighbors->push_back(s_key + MAP_WIDTH_ + 1);
                }

                if(y_minus)
                {
                if(Navigation::checkCollision(s_key, s_key - MAP_WIDTH_ + 1))
                    neighbors->push_back(s_key - MAP_WIDTH_ + 1);
                }
            }
            if(x_minus)
            {
                if(Navigation::checkCollision(s_key, s_key - 1))
                neighbors->push_back(s_key-1);
                if (y_plus) {
                if(Navigation::checkCollision(s_key, s_key + MAP_WIDTH_ - 1))
                neighbors->push_back(s_key + MAP_WIDTH_ - 1);
                }
                if (y_minus) {
                if(Navigation::checkCollision(s_key, s_key - MAP_WIDTH_ - 1))
                neighbors->push_back(s_key - MAP_WIDTH_ - 1);
                }
            }
            if (y_plus){
                if(Navigation::checkCollision(s_key, s_key + MAP_WIDTH_))
              neighbors->push_back(s_key + MAP_WIDTH_);
            }
            if (y_minus) {
                if(Navigation::checkCollision(s_key, s_key - MAP_WIDTH_))
              neighbors->push_back(s_key - MAP_WIDTH_);  
            }
}

void Navigation::getNeighbors(int s_key, std::vector<int>* neighbors) {

            float MAP_WIDTH_ = SIZE_X_/RESOLUTION_;
            float MAP_HEIGHT_ = SIZE_Y_/RESOLUTION_;
            
            neighbors->clear();
            int x = s_key % (int)MAP_WIDTH_;
            int y = s_key/ MAP_WIDTH_;
            populateNeighbors(x, y,MAP_WIDTH_, MAP_HEIGHT_ , s_key, neighbors);


    }

bool Navigation::AStarPlanner(Eigen::Vector2f start_loc, Eigen::Vector2f end_loc, std::vector<Eigen::Vector2f> *path){
    std::unordered_map<int, int> parent_nodes;
    std::unordered_map<int, float> cost_to_go_values;
    std::unordered_set<int> optimal_set;
    
    // Initilize the queue
    SimpleQueue<int, Priority> queue;
    int start_key = Navigation::getKeyFromState(start_loc);
    int end_key = Navigation::getKeyFromState(end_loc);

    queue.Push(start_key, Priority(0, Navigation::getHeuristic(start_loc, end_loc)));
    cost_to_go_values[start_key] = 0;
    optimal_set.clear();


    std::vector<int> neighbors;
    int edge_num = 0;

    while (edge_num < MAX_EDGE_EXPANSIONS_ && !queue.Empty())
    {
      ++edge_num;
      int curr_key = queue.Pop();
      optimal_set.insert(curr_key);

      Eigen::Vector2f curr_loc = Navigation::getStatefromKey(curr_key);

      Navigation::getNeighbors(curr_key, &neighbors);
      for (auto neighb_key: neighbors)
      {
        Eigen::Vector2f neighb_loc = Navigation::getStatefromKey(neighb_key);
        if (optimal_set.find(neighb_key) == optimal_set.end())
        {
          float g = cost_to_go_values[curr_key] + Navigation::getEdgeCost(neighb_loc, curr_loc);
          float h = Navigation::getHeuristic(neighb_loc, end_loc);
          if (!queue.Exists(neighb_key) || cost_to_go_values[neighb_key] > g)
          {
            parent_nodes[neighb_key] = curr_key;
            cost_to_go_values[neighb_key] = g;
            queue.Push(neighb_key, Priority(g, h));
            visualization::DrawLine(Navigation::getStatefromKey(curr_key),
               Navigation::getStatefromKey(neighb_key), 0xC0C0C0, global_viz_msg_);
            viz_pub_.publish(global_viz_msg_);
          }
        }
      }
      if (parent_nodes.find(end_key) != parent_nodes.end())
      {
        path->clear();
        int current = end_key;
        do
        {
          path->push_back(Navigation::getStatefromKey(current));
          CHECK(parent_nodes.find(current) != parent_nodes.end());
          current = parent_nodes[current];
        } while (current != start_key);
        path->push_back(Navigation::getStatefromKey(start_key));
        return true;
      }
    }
    return false;
}

PathOption SelectOptimalPath(vector<PathOption> path_options, Vector2f short_term_goal){
  PathOption result;
  float w_dist_to_goal = -0.50;
  float w_clearance = 5;
  result.free_path_length = 0;
  result.clearance = 0;
  result.curvature = 0;
  float max_score = -100000.0;
  for(auto p : path_options){
    float distance = (p.closest_point - short_term_goal).norm();
    float score = p.free_path_length + w_clearance*p.clearance + w_dist_to_goal*distance;
    if (score > max_score){
      max_score = score;
      result = p;
    }
    //printf("Curvature %f, Distance to goal %f, Clearance %f, Free path length %f, score %f\n", p.curvature, distance, p.clearance, p.free_path_length, score);
  }
  return result;
}


bool Navigation::PlanStillValid()
  {
    if (plan_path_.size() < 2)
      return false;
    int k_current = Navigation::getKeyFromState(robot_loc_);
    int k_goal = Navigation::getKeyFromState(nav_goal_loc_);
    if (Navigation::getKeyFromState(plan_path_[0]) != k_goal)
      return false;
    for (const Vector2f &v : plan_path_)
    {
      int key = Navigation::getKeyFromState(v);
      if (key == k_current)
        return true;
    }
    return false;
  }


void Navigation::ObstacleAvoidance() {
  // Vector2f short_term_goal(10,0);
  
  // Set up initial parameters
  drive_msg_.header.stamp = ros::Time::now();
  drive_msg_.curvature = 0;
  drive_msg_.velocity = 0;

  // Time control delta
  // const float deltaT = 0.05;
  // Number of path options to take
  const float pathOptions = 20;
  vector<PathOption> path_array;

  // Iterate over all the paths
  for (float c = -MAX_CURVATURE_; c<= MAX_CURVATURE_; c+=MAX_CURVATURE_/pathOptions) {
    PathOption p;
    p.curvature = c;

    //Calculate the maximum possible free path length
    if(c < kEpsilon && c > -kEpsilon){
      // Straight path
      p.free_path_length = min(MAX_SHORT_TERM_GOAL_, short_term_goal.x());
    } else {
      // float turning_radius = 1/p.curvature;
      // Using the optimization mentioned in class where we take the free path 
      // only till the tangent.
      // p.free_path_length = fabs(turning_radius)*atan2(MAX_SHORT_TERM_GOAL_, fabs(turning_radius));

      float turn_radius = 1.0f / p.curvature;
      Vector2f turn_center(0, turn_radius);
      Vector2f target_radial = short_term_goal - turn_center;
      Vector2f middle_radial =
          fabs(turn_radius) * target_radial.normalized();
      float middle_angle =
          atan2(fabs(middle_radial.x()), fabs(middle_radial.y()));
      p.free_path_length = middle_angle * fabs(turn_radius);
    }
    p = GetFreePathLength(p, short_term_goal);
    visualization::DrawPathOption(p.curvature,
                                  p.free_path_length,
                                  p.clearance,
                                  0x0000FF,
                                  false,
                                  local_viz_msg_);
      visualization::DrawCross(p.closest_point, 0.05, 0x00A000, local_viz_msg_);
      path_array.push_back(p);
    }

    PathOption p = SelectOptimalPath(path_array, short_term_goal);
    visualization::DrawPathOption(p.curvature,
                                p.free_path_length,
                                p.clearance,
                                0xFF0000,
                                true,
                                local_viz_msg_);
    visualization::DrawCross(p.closest_point, 0.1, 0xFF0000, local_viz_msg_);
    OneDTOC(p, short_term_goal);
    
    // viz_pub_.publish(local_viz_msg_);
}

PathOption Navigation::GetFreePathLength(PathOption p, Eigen::Vector2f short_term_goal){
  float c = p.curvature;
  float ret_free_path_length = MAX_SHORT_TERM_GOAL_;
  float clearance = MAX_CLEARANCE_;
  Vector2f obstruction;
  if (fabs(c) < kEpsilon){
    float l = ROBOT_LENGTH_/2 + WHEEL_BASE_/2 + OBSTACLE_MARGIN_;
    float w = ROBOT_WIDTH_/2 + OBSTACLE_MARGIN_;
    for (auto &p : point_cloud_){
      if (fabs(p.y()) > w){
        continue;
      }
      float free_path_length = p.x() - l;
      if(ret_free_path_length > free_path_length){
        ret_free_path_length = free_path_length;
        obstruction = p;
      }
      ret_free_path_length = min(ret_free_path_length, p.x() - l);
    }
    for (auto &p : point_cloud_){
      if (p.x() - l > ret_free_path_length || p.x() < 0.0){
        continue;
      }
      clearance = min(clearance, fabs(p.y()));
    }
    p.free_path_length = max<float>(0.0, ret_free_path_length);
    p.clearance = clearance;
    p.obstruction = obstruction;

    Vector2f closest_point(min<float>(p.free_path_length, max<float>(short_term_goal.x(), 0)), 0);
    p.closest_point = closest_point;
    return p;
  }
  float r = 1/p.curvature;
  Vector2f center(0, r);
  float r1 = fabs(r) - ROBOT_WIDTH_/2 - OBSTACLE_MARGIN_;
  float r2 = sqrt(Sq(fabs(r)+ROBOT_WIDTH_/2 + OBSTACLE_MARGIN_) + Sq(ROBOT_LENGTH_/2 + WHEEL_BASE_/2 + OBSTACLE_MARGIN_));
  vector<float> angles(point_cloud_.size());
  vector<float> distances(point_cloud_.size());
  float min_angle = M_PI;

  for(size_t i=0; i<point_cloud_.size(); i++){
    Vector2f p_i = point_cloud_[i];
    float r_obs = (p_i-center).norm();
    float a = atan2(p_i.x(), Sign(c) * (center.y() - p_i.y()));
    angles[i] = a;
    distances[i] = r_obs;
    if (a < 0.0 || r_obs < r1 || r_obs > r2){
      continue;
    }
    float free_path_length = max<float>(0, a*fabs(r) - (ROBOT_LENGTH_/2 + WHEEL_BASE_/2 + OBSTACLE_MARGIN_));
    if (free_path_length < ret_free_path_length){
      ret_free_path_length = free_path_length;
      obstruction = p_i;
      min_angle = a;
    }
  }
  for (size_t i = 0; i < point_cloud_.size(); i++){
    if (angles[i] < min_angle && angles[i] > 0.0){
      float c = fabs(distances[i] - fabs(r));
      if (clearance > c){
          clearance = c;
      }
    }
  }

  p.clearance = max<float>(0.0, clearance);
  p.free_path_length = max<float>(0.0, ret_free_path_length);
  p.obstruction = obstruction;
  // printf("Clearance %f, Free Path Length %f, Curvature %f\n", p.clearance, p.free_path_length, p.curvature);

  float closest_angle_extended = atan2(short_term_goal.x(), fabs(r- short_term_goal.y()));
  float free_angle_extended = c*p.free_path_length;
  float len_arc = fabs(closest_angle_extended)*fabs(r);
  if(len_arc < p.free_path_length){
    Vector2f closest_point(Sign(c)*r*sin(closest_angle_extended), r-r*cos(closest_angle_extended));
    p.closest_point = closest_point;
  } else{
    Vector2f closest_point(r*sin(free_angle_extended), r-r*cos(free_angle_extended));
    p.closest_point = closest_point;
  }
  return p;
}

void Navigation::OneDTOC(PathOption p, Vector2f stg){
  float dist_to_travel = p.free_path_length - 0.2;
  // float dist_to_travel = (stg - p.closest_point).norm();
  float curvature = p.curvature;
  float speed = robot_vel_.norm();
  drive_msg_.curvature = curvature;
  if (dist_to_travel < STOPPING_DISTANCE_){
    // Decelerate
    drive_msg_.velocity = max<float>(0.0, speed - DELTA_T_*MAX_DEACCL_);
  } else if (speed < MAX_SPEED_) {
    if (SYSTEM_LATENCY_ * speed > dist_to_travel) {
      drive_msg_.velocity = speed;
    } else {
      drive_msg_.velocity = min<float>(MAX_SPEED_, speed + DELTA_T_*MAX_ACCEL_);
    }
  } else{
    drive_msg_.velocity = MAX_SPEED_;
  }
  visualization::DrawPathOption(p.curvature,
                                  p.free_path_length,
                                  p.clearance,
                                  0xFF0000,
                                  true,
                                  local_viz_msg_);
  drive_pub_.publish(drive_msg_);
  viz_pub_.publish(local_viz_msg_);
}

void Navigation::Plan()
  {

    Eigen::Vector2f start(robot_loc_);
    Eigen::Vector2f goal(nav_goal_loc_);

    // visualization::ClearVisualizationMsg(global_viz_msg_);

    visualization::DrawCross(goal, 0.2, 0xFF0000, global_viz_msg_);
    Navigation::SetOffset(robot_loc_);


    const bool found_path =
        Navigation::AStarPlanner(start, goal, &plan_path_);

    if (found_path)
    {
      Vector2f s1 = plan_path_[0];

      for (size_t i = 1; i < plan_path_.size(); ++i)
      {
        Vector2f s2 = plan_path_[i];
        visualization::DrawLine(s1, s2, 0x10E000, global_viz_msg_);
        s1 = s2;
      }
      viz_pub_.publish(global_viz_msg_);
    }
    else
    {
      printf("No path detected\n");
    }
  }

Eigen::Vector2f Navigation::GetCarrot()
{
    if (plan_path_.empty())
      return robot_loc_;

    size_t i = 0;
    while(i < plan_path_.size())
    {
        Eigen::Vector2f v = plan_path_[i];
        if ((v-robot_loc_).squaredNorm() < CARROT_DIST_SQ_)
        {
          return v;
        }
        i++;
    }
    return plan_path_[0];
}

void Navigation::Run() {
  // This function gets called 20 times a second to form the control loop.
  
  // Clear previous visualizations.
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);

  // If odometry has not been initialized, we can't do anything.
  if (!odom_initialized_) return;

  if (nav_complete_){
    // printf("Completed Navigation!\n");
    return;
  }
  
  nav_complete_ = false;

  if (!PlanStillValid())
    {
      Plan();
      viz_pub_.publish(global_viz_msg_);
    }
  // Plan();
   
  // Get Carrot.
  const Vector2f carrot = GetCarrot();
  auto msg_copy = global_viz_msg_;
  visualization::DrawCross(carrot, 0.2, 0x10E000, msg_copy);
  viz_pub_.publish(msg_copy);
  nav_complete_ = (robot_loc_ - carrot).norm() < NAV_TOLERANCE_;
  if (!nav_complete_)
  {
    short_term_goal = Eigen::Rotation2Df(-robot_angle_) * (carrot - robot_loc_);
    // short_term_goal = carrot;
    visualization::DrawCross(short_term_goal, 0.4, 0x10E000, local_viz_msg_);
    // printf("Robot: (%.2f, %.2f) -> Carrot: (%.2f, %.2f). SGoal: (%.2f, %.2f)\n", 
      // robot_loc_.x(), robot_loc_.y(), carrot.x(), carrot.y(), short_term_goal.x(), short_term_goal.y());
    if (short_term_goal.x() <= 0.0)
    {
      printf("\n");
      return;
    }
    ObstacleAvoidance();
  }
}


  
// ObstacleAvoidance();
// The latest observed point cloud is accessible via "point_cloud_"

// Eventually, you will have to set the control values to issue drive commands:
// drive_msg_.curvature = ...;
// drive_msg_.velocity = ...;

// Add timestamps to all messages.
// local_viz_msg_.header.stamp = ros::Time::now();
// global_viz_msg_.header.stamp = ros::Time::now();
// drive_msg_.header.stamp = ros::Time::now();
// // Publish messages.
// viz_pub_.publish(local_viz_msg_);
// viz_pub_.publish(global_viz_msg_);
// drive_pub_.publish(drive_msg_);
  
}  // namespace navigation
