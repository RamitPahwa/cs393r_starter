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
\file    particle-filter.cc
\brief   Particle Filter Starter Code
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <algorithm>
#include <cmath>
#include <iostream>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "shared/math/geometry.h"
#include "shared/math/line2d.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"

#include "config_reader/config_reader.h"
#include "particle_filter.h"

#include "vector_map/vector_map.h"

using geometry::line2f;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using Eigen::Vector2f;
using Eigen::Vector2i;
using vector_map::VectorMap;


// extra function we are using
using math_util::AngleMod;
using math_util::DegToRad;
using math_util::RadToDeg;
using math_util::AngleDist;

using std::max;
using std::min;

using Eigen::Rotation2Df;


namespace particle_filter {

config_reader::ConfigReader config_reader_({"config/particle_filter.lua"});

ParticleFilter::ParticleFilter() :
    // tracking variables
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    odom_initialized_(false),
    last_update_loc_(0, 0),
    last_update_angle_(0),    
    resampling_skipped_(0),
    // config parameters
    NUM_PARTICLES_(50),
    INIT_ANGLE_STDDEV_(DegToRad(10)),
    INIT_LOC_STDDEV_(0.1),
    K1_(0.3),
    K2_(0.3),
    K3_(0.5),
    K4_(0.5),
    LASER_LOC_(0.20),
    LASER_SAMPLE_FREQ_(1),
    GAMMA_(1/50.0),
    LASER_STDDEV_(0.4),
    LASER_SHORT_CUTOFF_(0.2),
    LASER_LONG_CUTOFF_(0.4),
    RESAMPLE_INTERVAL_(2),
    MIN_ANGLE_CHANGE_(DegToRad(0.03)),
    MIN_DIST_CHANGE_(0.03) {}

void ParticleFilter::GetParticles(vector<Particle>* particles) const {
  *particles = particles_;
}

void ParticleFilter::GetPredictedPointCloud(const Vector2f& loc,
                                            const float angle,
                                            int num_ranges,
                                            float range_min,
                                            float range_max,
                                            float angle_min,
                                            float angle_max,
                                            vector<Vector2f>* scan_ptr) {
  vector<Vector2f>& scan = *scan_ptr;
  // // Compute what the predicted point cloud would be, if the car was at the pose
  // // loc, angle, with the sensor characteristics defined by the provided
  // // parameters.
  // // This is NOT the motion model predict step: it is the prediction of the
  // // expected observations, to be used for the update step.

  // the robot laser location we used in obstacle avoidance, 
  // and in odometry is different from here, choose correct value
  Vector2f laser_base(LASER_LOC_, 0.0);
  Vector2f loc_laser = loc + Rotation2Df(angle) * laser_base;

  const float angle_delta = (angle_max - angle_min) / (static_cast<float>(num_ranges - 1));
  // float angle_delta = (angle_max - angle_min) / num_ranges;

  // for every ray, start point is laser_loc and end point is changing scan_end
  for (int i = 0; i < num_ranges; ++i) {
    // unsure if the angle min/max are relative to robot angle
    float angle_laser = angle + angle_min + i * angle_delta;
    Vector2f end_laser = loc_laser 
      + range_max * Vector2f(cos(angle_laser), sin(angle_laser));

    // find the closest intersection point with any line in map
    // for this laser_loc -> scan_end line
    Vector2f closest_p, temp_p;
    float closest_sqdist = __FLT_MAX__;
    for (size_t i = 0; i < map_.lines.size(); ++i) {
      const line2f map_line = map_.lines[i];
      if (map_line.Intersection(loc_laser, end_laser, &temp_p)) {
        float sqdist = (loc_laser - temp_p).squaredNorm();
        if (closest_sqdist > sqdist) {
          closest_p = temp_p;
          closest_sqdist = sqdist;
        }
      }
    }
    if (closest_sqdist < __FLT_MAX__) {
      scan.push_back(closest_p);
    }
  }
}

void ParticleFilter::Update(const vector<float>& ranges,
                            float range_min,
                            float range_max,
                            float angle_min,
                            float angle_max,
                            Particle* p_ptr) {
  // Implement the update step of the particle filter here.
  // You will have to use the `GetPredictedPointCloud` to predict the expected
  // observations for each particle, and assign weights to the particles based
  // on the observation likelihood computed by relating the observation to the
  // predicted point cloud.
  Particle& p = *p_ptr;
  Vector2f laser_base(LASER_LOC_, 0.0);
  Vector2f loc_laser = p.loc + Rotation2Df(p.angle) * laser_base;

  // this might be a optmization to update the true range_max from the data
  // not sure if this is needed
  float max_visible_range = 0;
  for (const float& r : ranges) {
    if (r >= range_max) continue;
    max_visible_range = max(r, max_visible_range);
  }

  // we see that 'GetPredictedScan' function is faster
  vector<float> predicted_scan;
  map_.GetPredictedScan(loc_laser,
                  range_min,
                  // range_max,
                  max_visible_range,
                  angle_min + p.angle,
                  angle_max + p.angle,
                  ranges.size(),
                  &predicted_scan);

  // vector<Vector2f> predicted_scan;
  // GetPredictedPointCloud(loc_laser,
  //                 p.angle,
  //                 ranges.size(),
  //                 range_min,
  //                 // range_max,
  //                 min<float>(10, max_visible_range),
  //                 angle_min + p.angle,
  //                 angle_max + p.angle,
  //                 &predicted_scan);
  
  // for each ray from the artificial laser scan, calculate the similarity
  // as a log laser weight and add as a single weight at the end
  // how did we arrive at the value of kGamma -> important to know num_rays for this
  // do we need to skip lasers to reduce compute? kNumSkipLaser
  double w_total = 0;
  // rays are independent, hence overall weight is addition of individual weights
  for (size_t i = 0; i < ranges.size(); i += LASER_SAMPLE_FREQ_) {
    // short hit - when observation was closer than prediction
    // long hit - when observation was farther than prediction    
    double w_short_hit = -Sq(LASER_SHORT_CUTOFF_ / LASER_STDDEV_);
    double w_long_hit = -Sq(LASER_LONG_CUTOFF_ / LASER_STDDEV_);

    // calculate the weight of the ray - difference in observation and prediction
    // which is there was no intersection?
    float actual_range = ranges[i];
    float predicted_range = predicted_scan[i];
    float diff = actual_range - predicted_range;
    double w_ray = -Sq(diff / LASER_STDDEV_);
    if (predicted_range >= range_max) {
      w_ray = w_short_hit;
    } else if (diff < 0) {
      w_ray = max(w_short_hit, w_ray);
    } else {
      w_ray = max(w_long_hit, w_ray);
    }

    // independent rays, log weights addition
    w_total += w_ray;
  }
  p.weight += w_total * GAMMA_;
}

void ParticleFilter::Resample() {
  // Resample the particles, proportional to their weights.
  // The current particles are in the `particles_` variable. 
  // Create a variable to store the new particles, and when done, replace the
  // old set of particles:
  // vector<Particle> new_particles';
  // During resampling: 
  //    new_particles.push_back(...)
  // After resampling:
  // particles_ = new_particles;

  // You will need to use the uniform random number generator provided. For
  // example, to generate a random number between 0 and 1:
  // float x = rng_.UniformRandom(0, 1);
  // printf("Random number drawn from uniform distribution between 0 and 1: %f\n",
        //  x);

  // finding the max log weight among all particles
  double max_log_w = -__DBL_MAX__;
  for (const Particle& p : particles_) {
    max_log_w = max(max_log_w, p.weight);
  }
  double total_weight = 0;
  vector<double> summed_weights;
  for (Particle& p : particles_) {
    // rescaling each particle weight in a way that max particle weight is 1
    p.weight = exp(p.weight - max_log_w);
    // we will use total weight for uniform sampling later
    total_weight += p.weight;
    summed_weights.push_back(total_weight);
  }
  // resample new particles based on weights of exisitng particles
  // if a particular particle has large weight, resampling will 
  // ensure that more duplicates of this particle are taken
  // reset the particle weights for the new particles
  vector<Particle> new_particles(NUM_PARTICLES_);
  for (Particle& p : new_particles) {
    const double r = rng_.UniformRandom(0, total_weight);
    // Find which particle index corresponds to r in summed_weights.
    // you can use binary search for this, since summed weights is a increasing array
    int idx = lower_bound(summed_weights.begin(), summed_weights.end(), r) 
                  - summed_weights.begin();
    p = particles_[idx];
    p.weight = 0;
  }
  particles_ = new_particles;
}

void ParticleFilter::ObserveLaser(const vector<float>& ranges,
                                  float range_min,
                                  float range_max,
                                  float angle_min,
                                  float angle_max) {
  // A new laser scan observation is available (in the laser frame)
  // Call the Update and Resample steps as necessary.
  static CumulativeFunctionTimer function_timer_(__FUNCTION__);
  CumulativeFunctionTimer::Invocation invoke(&function_timer_);
  // do not run the function if the change in angle and dist is low
  if (odom_initialized_ && (last_update_loc_ - prev_odom_loc_).norm() < MIN_DIST_CHANGE_ &&
        AngleDist(prev_odom_angle_, last_update_angle_) < MIN_ANGLE_CHANGE_) {
    return;
  }
  last_update_loc_ = prev_odom_loc_;
  last_update_angle_ = prev_odom_angle_;

  // here we are just removing out laser
  // readings that are out of the range
  vector<float> filtered_laser = ranges;
  for (auto& r : filtered_laser) {
    if (r < range_min || r > range_max) {
      r = 0;
    }
  }

  // for each particle, we need to add a weight, based on how close its 
  // laser observation based on map is similar to the actual laser observation
  for (auto& p : particles_) {
    Update(ranges, range_min, range_max, angle_min, angle_max, &p);
  }
  
  // to deal wtih loss of sampling variance, we have to resample less often
  if (resampling_skipped_ >= RESAMPLE_INTERVAL_) {
    Resample();
    resampling_skipped_ = 0;
  } else {
    ++resampling_skipped_;
  }
}

void ParticleFilter::Predict(const Vector2f& odom_loc,
                             const float odom_angle) {
  // Implement the predict step of the particle filter here.
  // A new odometry value is available (in the odom frame)
  // Implement the motion model predict step here, to propagate the particles
  // forward based on odometry.
  static CumulativeFunctionTimer function_timer_(__FUNCTION__);
  CumulativeFunctionTimer::Invocation invoke(&function_timer_);   
  // The very first callback goes here
  if (!odom_initialized_) {
    prev_odom_angle_ = odom_angle;
    prev_odom_loc_ = odom_loc;
    odom_initialized_ = true;
  }

  Vector2f loc_delta =
    Rotation2Df(-prev_odom_angle_) * (odom_loc - prev_odom_loc_);
  float angle_delta = odom_angle - prev_odom_angle_;

  prev_odom_angle_ = odom_angle;
  prev_odom_loc_ = odom_loc;

  float angle_diff = fabs(angle_delta - M_2PI * floor(angle_delta / M_2PI + 0.5));
  float loc_diff = loc_delta.norm();
  float angle_stddev = K1_ * angle_diff + K2_ * loc_diff;
  float loc_stddev = K3_ * angle_diff + K4_ * loc_diff;

  // move each particle based on angle and location change
  // with some gaussian noise
  for (auto& p : particles_) {
    p.angle += angle_delta + rng_.Gaussian(0, angle_stddev);
    p.loc += Rotation2Df(p.angle) * (loc_delta + Vector2f(rng_.Gaussian(0, loc_stddev), rng_.Gaussian(0, loc_stddev)));
  }
}

void ParticleFilter::Initialize(const string& map_file,
                                const Vector2f& loc,
                                const float angle) {
  // The "set_pose" button on the GUI was clicked, or an initialization message
  // was received from the log. Initialize the particles accordingly, e.g. with
  // some distribution around the provided location and angle.
  map_.Load(map_file);

  // why 50 particles? - it was given in starter code
  particles_.resize(NUM_PARTICLES_);


  // initialize the particle cloud as a distribution aroudn the first point
  // each particle is just a loc + angle with some noise added
  for (Particle& p : particles_) {
    p.angle = rng_.Gaussian(angle, INIT_ANGLE_STDDEV_);
    p.loc.x() = rng_.Gaussian(loc.x(), INIT_LOC_STDDEV_);
    p.loc.y() = rng_.Gaussian(loc.y(), INIT_LOC_STDDEV_);
    p.weight = 1;
  }
}

void ParticleFilter::GetLocation(Eigen::Vector2f* loc_ptr, 
                                 float* angle_ptr) const {

  // Compute the best estimate of the robot's location based on the current set
  // of particles. The computed values must be set to the `loc` and `angle`
  // variables to return them. Modify the following assignments:
  // loc = Vector2f(0, 0);
  // angle = 0;

  const Vector2f unit_vector(1.0, 0.0);
  Vector2f loc_mean(0, 0);
  Vector2f angle_mean(0, 0);

  // calculate the mean location over all points in the particle cloud
  // also mean angle, as a direction vector with unit distance
  // and then convert it back to an angle
  // why is it required to calculate the mean angle like this?
  for (auto & p : particles_) {
    loc_mean += p.loc;
    angle_mean += Rotation2Df(p.angle) * unit_vector;
  }
  loc_mean = loc_mean/NUM_PARTICLES_;
  angle_mean = angle_mean/NUM_PARTICLES_;
  
  // set the final values
  *loc_ptr = loc_mean;
  *angle_ptr = atan2(angle_mean.y(), angle_mean.x());
  // printf("Estimated Location: (%f, %f), %f\n", loc.x(), loc.y(), angle);
}


}  // namespace particle_filter
