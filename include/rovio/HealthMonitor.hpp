/*
* Copyright (c) 2018, Autonomous Systems Lab
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of the Autonomous Systems Lab, ETH Zurich nor the
* names of its contributors may be used to endorse or promote products
* derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#ifndef ROVIO_HEALTH_MONITOR_H_
#define ROVIO_HEALTH_MONITOR_H_
#include <memory>

#include <vector>

#include "rovio/CoordinateTransform/RovioOutput.hpp"
#include "rovio/RovioFilter.hpp"

#include <ros/node_handle.h>

namespace rovio {

class RovioHealthMonitor {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  RovioHealthMonitor(const ros::NodeHandle& nh,
                     const ros::NodeHandle& nh_private)
      : nh_(nh),
        nh_private_(nh_private),
        active_(false),
        velocity_to_consider_static_(0.1),
        max_subsequent_unhealthy_updates_(1),
        healthy_feature_distance_cov_(0.5),
        healthy_feature_distance_cov_increment_(0.3),
        unhealthy_feature_distance_cov_(1.0),
        unhealthy_velocity_(5.0),
        num_subsequent_unhealthy_updates_(0) {}

  // Access settings programmatically.
  bool active() const { return active_; }

  // Returns true if healthy; false if unhealthy and reset was triggered.
  bool shouldResetEstimator(const std::vector<float>& distance_covs_in,
                            const StandardOutput& imu_output) {
    float feature_distance_covariance_median = 0;
    std::vector<float> distance_covs = distance_covs_in;
    if (!distance_covs.empty()) {
      const size_t middle_index = distance_covs.size() / 2;
      std::nth_element(distance_covs.begin(),
                       distance_covs.begin() + middle_index,
                       distance_covs.end());
      feature_distance_covariance_median = distance_covs[middle_index];
    }

    const float BvB_norm = imu_output.BvB().norm();

    if ((BvB_norm > velocity_to_consider_static_) &&
        ((BvB_norm > unhealthy_velocity_) ||
         (feature_distance_covariance_median > unhealthy_feature_distance_cov_))) {
      ++num_subsequent_unhealthy_updates_;
      std::cout << "Estimator fault counter: "
                << num_subsequent_unhealthy_updates_ << "/"
                << max_subsequent_unhealthy_updates_ << ". Might reset soon.";

      if (num_subsequent_unhealthy_updates_ > max_subsequent_unhealthy_updates_) {
        std::cout << "Will reset ROVIOLI. Velocity norm: " << BvB_norm
                  << " (limit: " << unhealthy_velocity_
                  << "), median of feature distance covariances: "
                  << feature_distance_covariance_median
                  << " (limit: " << unhealthy_feature_distance_cov_ << ").";
        return true;
      }
    } else {
      if (feature_distance_covariance_median < healthy_feature_distance_cov_) {
        if (std::abs(feature_distance_covariance_median -
                     last_safe_pose_.feature_distance_covariance_median) <
            healthy_feature_distance_cov_increment_) {
          last_safe_pose_.failsafe_WrWB = imu_output.WrWB();
          last_safe_pose_.failsafe_qBW = imu_output.qBW();
          last_safe_pose_.feature_distance_covariance_median =
              feature_distance_covariance_median;
        }
      }
      num_subsequent_unhealthy_updates_ = 0;
    }
    return false;
  }

  Eigen::Vector3d failsafe_WrWB() { return last_safe_pose_.failsafe_WrWB; }

  kindr::RotationQuaternionPD failsafe_qBW() {
    return last_safe_pose_.failsafe_qBW;
  }

 private:
  struct RovioFailsafePose {
    RovioFailsafePose()
        : failsafe_WrWB(Eigen::Vector3d::Zero()),
          feature_distance_covariance_median(0.0) {
      failsafe_qBW.setIdentity();
    }

    Eigen::Vector3d failsafe_WrWB;
    kindr::RotationQuaternionPD failsafe_qBW;
    float feature_distance_covariance_median;
  };

  // ROS Stuff
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Parameters
  bool active_; // Whether the health checker is on or not.

  // The landmark covariance is not a good measure for divergence if we are
  // static.
  double velocity_to_consider_static_;
  int max_subsequent_unhealthy_updates_;
  double healthy_feature_distance_cov_;
  double healthy_feature_distance_cov_increment_;
  double unhealthy_feature_distance_cov_;
  double unhealthy_velocity_;

  // State
  RovioFailsafePose last_safe_pose_;

  int num_subsequent_unhealthy_updates_;
};

}  // namespace rovio

#endif  // ROVIO_HEALTH_MONITOR_H_
