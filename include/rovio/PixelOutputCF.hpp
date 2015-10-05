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

#ifndef ROVIO_PIXELOUTPUTCF_HPP_
#define ROVIO_PIXELOUTPUTCF_HPP_

#include "rovio/FilterStates.hpp"
#include "lightweight_filtering/CoordinateTransform.hpp"

#include "FeatureBearingOutputCF.hpp"
#include "rovio/Camera.hpp"

namespace rovio {

class PixelOutput: public LWF::State<LWF::VectorElement<2>>{
 public:
  static constexpr unsigned int _pix = 0;
  PixelOutput(){
  }
  ~PixelOutput(){};
  cv::Point2f getPoint2f() const{
    return cv::Point2f(static_cast<float>(this->get<_pix>()(0)),static_cast<float>(this->get<_pix>()(1)));
  }
};

template<typename STATE>
class PixelOutputCF:public LWF::CoordinateTransform<STATE,PixelOutput,true>{
 public:
  typedef LWF::CoordinateTransform<STATE,PixelOutput,true> Base;
  typedef typename Base::mtInput mtInput;
  typedef typename Base::mtOutput mtOutput;
  typedef typename Base::mtMeas mtMeas;
  typedef typename Base::mtJacInput mtJacInput;
  int ID_;
  rovio::Camera* mpCameras_;
  PixelOutputCF(){
    ID_ = 0;
    mpCameras_ = nullptr;
  };
  void setCamera(Camera* mpCameras){
    mpCameras_ = mpCameras;
  }
  void setIndex(int ind){
    ID_ = ind;
  }
  ~PixelOutputCF(){};
  void eval(mtOutput& output, const mtInput& input, const mtMeas& meas, double dt = 0.0) const{
    // MrMC = MrMC
    // qCM = qCM
    cv::Point2f c;
    const int& camID = input.aux().camID_[ID_];
    mpCameras_[camID].bearingToPixel(input.CfP(ID_),c);
    output.template get<mtOutput::_pix>() = Eigen::Vector2d(c.x,c.y);
  }
  void jacInput(mtJacInput& J, const mtInput& input, const mtMeas& meas, double dt = 0.0) const{
    J.setZero();
    cv::Point2f c;
    Eigen::Matrix<double,2,2> J_temp;
    const int& camID = input.aux().camID_[ID_];
    mpCameras_[camID].bearingToPixel(input.CfP(ID_),c,J_temp);
    J.template block<2,2>(mtOutput::template getId<mtOutput::_pix>(),mtInput::template getId<mtInput::_nor>(ID_)) = J_temp;
  }
};

class PixelOutputFromNorCF:public LWF::CoordinateTransform<FeatureBearingOutput,PixelOutput,true>{
 public:
  typedef LWF::CoordinateTransform<FeatureBearingOutput,PixelOutput,true> Base;
  typedef typename Base::mtInput mtInput;
  typedef typename Base::mtOutput mtOutput;
  typedef typename Base::mtMeas mtMeas;
  typedef typename Base::mtJacInput mtJacInput;
  int camID_;
  rovio::Camera* mpCameras_;
  PixelOutputFromNorCF(){
    camID_ = 0;
    mpCameras_ = nullptr;
  };
  void setCamera(Camera* mpCameras){
    mpCameras_ = mpCameras;
  }
  void setCameraID(int ID){
    camID_ = ID;
  }
  ~PixelOutputFromNorCF(){};
  void eval(mtOutput& output, const mtInput& input, const mtMeas& meas, double dt = 0.0) const{
    // MrMC = MrMC
    // qCM = qCM
    cv::Point2f c;
    mpCameras_[camID_].bearingToPixel(input.CfP(),c);
    output.template get<mtOutput::_pix>() = Eigen::Vector2d(c.x,c.y);
  }
  void jacInput(mtJacInput& J, const mtInput& input, const mtMeas& meas, double dt = 0.0) const{
    J.setZero();
    cv::Point2f c;
    Eigen::Matrix<double,2,2> J_temp;
    mpCameras_[camID_].bearingToPixel(input.CfP(),c,J_temp);
    J.template block<2,2>(mtOutput::template getId<mtOutput::_pix>(),mtInput::template getId<mtInput::_nor>()) = J_temp;
  }
};

}


#endif /* ROVIO_PIXELOUTPUTCF_HPP_ */
