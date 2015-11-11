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

#ifndef ROVIO_MULTICAMERA_HPP_
#define ROVIO_MULTICAMERA_HPP_

#include "lightweight_filtering/common.hpp"
#include "rovio/Camera.hpp"
#include "rovio/FeatureCoordinates.hpp"

namespace rovio{

template<int nCam>
class MultiCamera{
 public:
  V3D BrBC_[nCam]; //!< Translational extrinsic parameter
  QPD qCB_[nCam]; //!< Rotational extrinsic parameter
  Camera cameras_[nCam]; //!< Camera array

  /** \brief Constructor.
   *
   *  Initializes the multicamera object with all cameras at the same point
   * */
  MultiCamera(){
    for(unsigned int i=0;i<nCam;i++){
      qCB_[i].setIdentity();
      BrBC_[i].setZero();
    }
  };
  virtual ~MultiCamera(){};

  /** \brief Sets the extrinsics of the i'th camera
   *
   *   @param i - Camera index
   *   @param BrBC - Translational extrinsic parameter
   *   @param qCB - Rotational extrinsic parameter
   */
  void setExtrinsics(const int i, const V3D& BrBC, const QPD& qCB){
    BrBC_[i] = BrBC;
    qCB_[i] = qCB;
  }

  /** \brief Loads and sets the distortion model and the corresponding distortion coefficients from yaml-file for camera i
   *
   *   @param i - Camera index
   *   @param filename - Path to the yaml-file, containing the distortion model and distortion coefficient data.
   */
  void load(const int i, const std::string& filename){
    cameras_[i].load(filename);
  }

  /** \brief Transforms feature coordinates from one camera frame to another
   *
   *   @param u - Camera index of output frame
   *   @param cIn - Feature coordinates to be transformed
   *   @param dIn - Corresponding distance
   *   @param cOut - Transformed feature coordinates
   *   @param dOut - Corresponding distance of output
   *   @todo avoid double computation
   */
  void transformFeature(const int i, const FeatureCoordinates& vecIn, const FeatureDistance& dIn, FeatureCoordinates& vecOut, FeatureDistance& dOut) const{
    if(vecIn.camID_ != i){
      // D = Destination camera frame.
      // qDC = qDM*qCM^T
      // CrCD = qCM*(MrMD-MrMC)
      // DrDP = qDC*(d_in*nor_in-CrCD)
      // d_out = ||DrDP||
      // nor_out = DrDP/d_out
      const QPD qDC = qCB_[i]*qCB_[vecIn.camID_].inverted();
      const V3D CrCD = qCB_[vecIn.camID_].rotate(V3D(BrBC_[i]-BrBC_[vecIn.camID_]));
      const V3D CrCP = dIn.getDistance()*vecIn.get_nor().getVec();
      const V3D DrDP = qDC.rotate(V3D(CrCP-CrCD));
      dOut.setParameter(DrDP.norm());
      vecOut.nor_.setFromVector(DrDP);
      vecOut.valid_c_ = false;
      vecOut.valid_nor_ = true;
      vecOut.camID_ = i;
      vecOut.mpCamera_ = &cameras_[i];
      vecOut.trackWarping_ = vecIn.trackWarping_;
      vecOut.pixelCov_.setZero();
      vecOut.eigenVector1_.setZero();
      vecOut.eigenVector2_.setZero();
      vecOut.sigma1_ = 0.0;
      vecOut.sigma2_ = 0.0;
      vecOut.sigmaAngle_ = 0.0;
      if(vecIn.trackWarping_){ // Invalidate warping
        vecOut.valid_warp_c_ = false;
        vecOut.valid_warp_nor_ = false;
        vecOut.isWarpIdentity_ = false;
      } else {
        vecOut.warp_c_ = vecIn.warp_c_;
        vecOut.valid_warp_c_ = vecIn.valid_warp_c_;
        vecOut.warp_nor_ = vecIn.warp_nor_;
        vecOut.valid_warp_nor_ = vecIn.valid_warp_nor_;
        vecOut.isWarpIdentity_ = vecIn.isWarpIdentity_;
      }
    } else {
      vecOut = vecIn;
      dOut = dIn;
    }
  }
};

}


#endif /* ROVIO_MULTICAMERA_HPP_ */
