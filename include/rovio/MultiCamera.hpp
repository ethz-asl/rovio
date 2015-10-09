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
  ~MultiCamera(){};

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

  /** \brief Transforms a bearing vector from one camera frame to another
   *
   *   @param i - Camera index of input frame
   *   @param i - Camera index of output frame
   *   @param vecIn - Bearing vector to be transformed
   *   @param depth - Corresponding distance
   *   @param vecOut - Transformed bearing vector
   */
  void transformBearing(const int i, const int j, const Eigen::Vector3d& vecIn, const double& d, Eigen::Vector3d& vecOut) const{
    const QPD qDC = qCB_[j]*qCB_[i].inverted(); // TODO: avoid double computation
    const V3D CrCD = qCB_[i].rotate(V3D(BrBC_[j]-BrBC_[i]));
    const V3D CrCP = d*vecIn;
    vecOut = qDC.rotate(V3D(CrCP-CrCD));
  }
  /** \brief Transforms feature coordinates from one camera frame to another
   *
   *   @param i - Camera index of input frame
   *   @param i - Camera index of output frame
   *   @param cIn - Feature coordinates to be transformed
   *   @param depth - Corresponding distance
   *   @param cOut - Transformed feature coordinates
   */
  void transformBearing(const int i, const int j, const FeatureCoordinates& vecIn, const FeatureDistance& d, FeatureCoordinates& vecOut) const{
    const QPD qDC = qCB_[j]*qCB_[i].inverted(); // TODO: avoid double computation
    const V3D CrCD = qCB_[i].rotate(V3D(BrBC_[j]-BrBC_[i]));
    const V3D CrCP = d.getDistance()*vecIn.get_nor().getVec();
    vecOut.nor_.setFromVector(qDC.rotate(V3D(CrCP-CrCD)));
    vecOut.valid_c_ = false;
    vecOut.valid_nor_ = true;
  }
};

}


#endif /* ROVIO_MULTICAMERA_HPP_ */
