// This file was stolen straight out of maplab

#ifndef ROVIO_HEALTH_MONITOR_H_
#define ROVIO_HEALTH_MONITOR_H_

#include <memory>
#include <vector>

#include "rovio/CoordinateTransform/RovioOutput.hpp"
#include "rovio/RovioFilter.hpp"

namespace rovio {
class RovioHealthMonitor {
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    RovioHealthMonitor() : num_subsequent_unhealthy_updates_(0) {}

    // Returns true if healthy; false if unhealthy and reset was triggered.
    bool shouldResetEstimator(const std::vector<float>& distance_covs_in, const StandardOutput& imu_output) {
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
        if ((BvB_norm > kVelocityToConsiderStatic) && ((BvB_norm > kUnhealthyVelocity) ||
         (feature_distance_covariance_median > kUnhealthyFeatureDistanceCov))) {
            ++num_subsequent_unhealthy_updates_;
            std::cout << "Estimator fault counter: "
                << num_subsequent_unhealthy_updates_ << "/"
                << kMaxSubsequentUnhealthyUpdates << ". Might reset soon.";
            if (num_subsequent_unhealthy_updates_ > kMaxSubsequentUnhealthyUpdates) {
                std::cout << "Will reset ROVIOLI. Velocity norm: " << BvB_norm
                  << " (limit: " << kUnhealthyVelocity
                  << "), median of feature distance covariances: "
                  << feature_distance_covariance_median
                  << " (limit: " << kUnhealthyFeatureDistanceCov << ").";
                return true;
            }
        } else {
            if (feature_distance_covariance_median < kHealthyFeatureDistanceCov) {
                if (std::abs(feature_distance_covariance_median -
                  last_safe_pose_.feature_distance_covariance_median) <
                  kHealthyFeatureDistanceCovIncrement) {
                    last_safe_pose_.failsafe_WrWB = imu_output.WrWB();
                    last_safe_pose_.failsafe_qBW = imu_output.qBW();
                    last_safe_pose_.feature_distance_covariance_median = feature_distance_covariance_median;
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
            : failsafe_WrWB(Eigen::Vector3d::Zero()), feature_distance_covariance_median(0.0) {
                failsafe_qBW.setIdentity();
            }
        Eigen::Vector3d failsafe_WrWB;
        kindr::RotationQuaternionPD failsafe_qBW;
        float feature_distance_covariance_median;
    };

    RovioFailsafePose last_safe_pose_;
    int num_subsequent_unhealthy_updates_;

    // The landmark covariance is not a good measure for divergence if we are static.
    static constexpr float kVelocityToConsiderStatic = 0.1f;
    static constexpr int kMaxSubsequentUnhealthyUpdates = 1;
    static constexpr float kHealthyFeatureDistanceCov = 0.5f;
    static constexpr float kHealthyFeatureDistanceCovIncrement = 0.3f;
    static constexpr float kUnhealthyFeatureDistanceCov = 1.0f;
    static constexpr float kUnhealthyVelocity = 5.0f;
};

}  // namespace rovio
 #endif  // ROVIO_HEALTH_MONITOR_H_ 