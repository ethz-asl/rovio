/*
* Copyright (c) 2014, Autonomous Systems Lab
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
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
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

#ifndef ROVIO_ROVIO_FILTER_HPP_
#define ROVIO_ROVIO_FILTER_HPP_

#include "kindr/rotations/RotationEigen.hpp"
#include <Eigen/Dense>
#include "lightweight_filtering/FilterBase.hpp"
#include "lightweight_filtering/State.hpp"
#include "rovio/FilterStates.hpp"
#include "rovio/ImgUpdate.hpp"
#include "rovio/ImuPrediction.hpp"
#include "rovio/Camera.hpp"

namespace rot = kindr::rotations::eigen_impl;

namespace rovio {
/** \brief Class, defining the Rovio Filter.
 *
 *  @tparam FILTERSTATE - \ref rovio::FilterState
 */
template<typename FILTERSTATE>
class RovioFilter:public LWF::FilterBase<ImuPrediction<FILTERSTATE>,ImgUpdate<FILTERSTATE>>{
 public:
  typedef LWF::FilterBase<ImuPrediction<FILTERSTATE>,ImgUpdate<FILTERSTATE>> Base;
  using Base::init_;
  using Base::reset;
  using Base::predictionTimeline_;
  using Base::safe_;
  using Base::front_;
  using Base::readFromInfo;
  using Base::boolRegister_;
  using Base::intRegister_;
  using Base::doubleRegister_;
  using Base::mUpdates_;
  using Base::mPrediction_;
  using Base::stringRegister_;
  typedef typename Base::mtFilterState mtFilterState;
  typedef typename Base::mtPrediction mtPrediction;
  typedef typename Base::mtState mtState;
  rovio::Camera cameras_[mtState::nCam_]; // TODO IMG
  std::string cameraCalibrationFile_[mtState::nCam_];

  /** \brief Constructor. Initializes the filter.
   */
  RovioFilter(){
    std::get<0>(mUpdates_).setCamera(cameras_); // TODO IMG
    boolRegister_.registerScalar("Common.doVECalibration",init_.state_.aux().doVECalibration_);
    intRegister_.registerScalar("Common.depthType",init_.state_.aux().depthTypeInt_);
    for(int camID=0;camID<FILTERSTATE::mtState::nCam_;camID++){
      cameraCalibrationFile_[camID] = "";
      stringRegister_.registerScalar("Camera" + std::to_string(camID) + ".CalibrationFile",cameraCalibrationFile_[camID]);
      doubleRegister_.registerVector("Camera" + std::to_string(camID) + ".MrMC",init_.state_.aux().MrMC_[camID]);
      doubleRegister_.registerQuaternion("Camera" + std::to_string(camID) + ".qCM",init_.state_.aux().qCM_[camID]);
      doubleRegister_.removeScalarByVar(init_.state_.MrMC(camID)(0));
      doubleRegister_.removeScalarByVar(init_.state_.MrMC(camID)(1));
      doubleRegister_.removeScalarByVar(init_.state_.MrMC(camID)(2));
      doubleRegister_.removeScalarByVar(init_.state_.qCM(camID).toImplementation().w());
      doubleRegister_.removeScalarByVar(init_.state_.qCM(camID).toImplementation().x());
      doubleRegister_.removeScalarByVar(init_.state_.qCM(camID).toImplementation().y());
      doubleRegister_.removeScalarByVar(init_.state_.qCM(camID).toImplementation().z());
      doubleRegister_.registerVector("Camera" + std::to_string(camID) + ".MrMC",init_.state_.MrMC(camID));
      doubleRegister_.registerQuaternion("Camera" + std::to_string(camID) + ".qCM",init_.state_.qCM(camID));
    }
    int ind;
    for(int i=0;i<FILTERSTATE::mtState::nMax_;i++){
      ind = mtState::template getId<mtState::_nor>(i);
      doubleRegister_.removeScalarByVar(init_.cov_(ind,ind));
      doubleRegister_.removeScalarByVar(init_.cov_(ind+1,ind+1));
      ind = mtState::template getId<mtState::_dep>(i);
      doubleRegister_.removeScalarByVar(init_.cov_(ind,ind));
      doubleRegister_.removeScalarByVar(init_.state_.dep(i));
      doubleRegister_.removeScalarByVar(init_.state_.CfP(i).q_.toImplementation().w());
      doubleRegister_.removeScalarByVar(init_.state_.CfP(i).q_.toImplementation().x());
      doubleRegister_.removeScalarByVar(init_.state_.CfP(i).q_.toImplementation().y());
      doubleRegister_.removeScalarByVar(init_.state_.CfP(i).q_.toImplementation().z());
    }
    std::get<0>(mUpdates_).doubleRegister_.removeScalarByVar(std::get<0>(mUpdates_).outlierDetection_.getMahalTh(0));
    std::get<0>(mUpdates_).doubleRegister_.registerScalar("MahalanobisTh",std::get<0>(mUpdates_).outlierDetection_.getMahalTh(0));
    std::get<0>(mUpdates_).outlierDetection_.setEnabledAll(true);
    boolRegister_.registerScalar("Common.verbose",std::get<0>(mUpdates_).verbose_);
    mPrediction_.doubleRegister_.removeScalarByStr("alpha");
    mPrediction_.doubleRegister_.removeScalarByStr("beta");
    mPrediction_.doubleRegister_.removeScalarByStr("kappa");
    boolRegister_.registerScalar("Groundtruth.doVisualization",init_.plotGroundtruth_);
    doubleRegister_.registerVector("Groundtruth.IrIJ",init_.groundtruth_IrIJ_);
    doubleRegister_.registerQuaternion("Groundtruth.qJI",init_.groundtruth_qJI_);
    doubleRegister_.registerVector("Groundtruth.BrBC",init_.groundtruth_BrBC_);
    doubleRegister_.registerQuaternion("Groundtruth.qCB",init_.groundtruth_qCB_);
    Eigen::Vector3d groundtruth_IrIJ_;
    Eigen::Vector3d groundtruth_BrBC_;
    reset(0.0);
  }

  /** \brief Reloads the camera calibration for all cameras and resets the depth map type.
   */
  void refreshProperties(){
    for(int camID = 0;camID<mtState::nCam_;camID++){
      if (!cameraCalibrationFile_[camID].empty()) {
        cameras_[camID].load(cameraCalibrationFile_[camID]); // TODO IMG
      }
    }
    init_.state_.aux().depthMap_.setType(init_.state_.aux().depthTypeInt_);
  };

  /** \brief Destructor
   */
  ~RovioFilter(){};
//  void resetToImuPose(V3D WrWM, QPD qMW, double t = 0.0){
//    init_.state_.initWithImuPose(WrWM,qMW);
//    reset(t);
//  }

  /** \brief Resets the filter with an accelerometer measurement.
   *
   *  @param fMeasInit - Accelerometer measurement.
   *  @param t         - Current time. @todo check this
   */
  void resetWithAccelerometer(const V3D& fMeasInit, double t = 0.0){
    init_.initWithAccelerometer(fMeasInit);
    reset(t);
  }

  /** \brief Sets the transformation between IMU and Camera.
   *
   *  @param R_VM  -  Rotation matrix, expressing the orientation of the IMU  in Camera Cooridinates (IMU Coordinates -> Camera Coordinates).
   *  @param VrVM  -  Vector, pointing from the camera frame to the IMU frame, expressed in IMU Coordinates.
   *  @param camID -  ID of the considered camera.
   */
  void setExtrinsics(const Eigen::Matrix3d& R_CM, const Eigen::Vector3d& CrCM, const int camID = 0){
    rot::RotationMatrixAD R(R_CM);
    init_.state_.aux().qCM_[camID] = QPD(R.getPassive());
    init_.state_.aux().MrMC_[camID] = -init_.state_.aux().qCM_[camID].inverseRotate(CrCM); // TODO: all filterstates
  }
//  void resetToKeyframe(double t = 0.0) {
//    std::cout << "Reseting to keyframe" << std::endl;
//    double imuMeasTime = 0.0;
//    if(predictionTimeline_.getNextTime(t,imuMeasTime)){  // Find close accelerometer measurement
//      resetWithAccelerometer(predictionTimeline_.measMap_[imuMeasTime].template get<mtPrediction::mtMeas::_acc>(),t); // Initialize with accelerometer
//    } else {
//      reset(t);
//    }
//    safe_.state_.cloneCurrentToKeyframe(safe_.cov_,t);
//    front_ = safe_;
//  }
};

}


#endif /* ROVIO_ROVIO_FILTER_HPP_ */
