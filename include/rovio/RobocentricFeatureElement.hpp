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

#ifndef ROBOCENTRICFEATUREELEMENT_HPP_
#define ROBOCENTRICFEATUREELEMENT_HPP_

#include "lightweight_filtering/common.hpp"
#include "rovio/FeatureCoordinates.hpp"
#include "rovio/FeatureDistance.hpp"

namespace rovio{

class RobocentricFeatureElement: public LWF::ElementBase<RobocentricFeatureElement,RobocentricFeatureElement,3>{
 public:
  typedef LWF::ElementBase<RobocentricFeatureElement,RobocentricFeatureElement,3> Base;
  using typename Base::mtDifVec;
  using typename Base::mtGet;
  using Base::name_;
  mutable LWF::NormalVectorElement::mtDifVec norDifTemp_;
  mutable MXD norCovMatTemp_;
  mutable LWF::NormalVectorElement norTemp_;
  FeatureCoordinates coordinates_;
  FeatureDistance distance_;
  RobocentricFeatureElement():distance_(FeatureDistance::REGULAR), norCovMatTemp_(3,3){}
  RobocentricFeatureElement(const Camera* mpCameras, const FeatureDistance::Type depthType):coordinates_(mpCameras),distance_(depthType), norCovMatTemp_(3,3){}
  RobocentricFeatureElement(const RobocentricFeatureElement& other):distance_(other.distance_.type_), norCovMatTemp_(3,3){
    coordinates_ = other.coordinates_;
    distance_ = other.distance_;
  }

  /** \brief Destructor
   */
  virtual ~RobocentricFeatureElement(){};

  void boxPlus(const mtDifVec& vecIn, RobocentricFeatureElement& stateOut) const{
    if(&stateOut != this){
      stateOut.coordinates_ = coordinates_;
      stateOut.distance_ = distance_;
    }
    coordinates_.get_nor().boxPlus(vecIn.template block<2,1>(0,0),norTemp_);
    stateOut.coordinates_.set_nor(norTemp_,false);
    stateOut.distance_.p_ = distance_.p_ + vecIn(2);
  }
  void boxMinus(const RobocentricFeatureElement& stateIn, mtDifVec& vecOut) const{
    coordinates_.get_nor().boxMinus(stateIn.coordinates_.get_nor(),norDifTemp_);
    vecOut.template block<2,1>(0,0) = norDifTemp_;
    vecOut(2) = distance_.p_-stateIn.distance_.p_;
  }
  void boxMinusJac(const RobocentricFeatureElement& stateIn, MXD& matOut) const{
    matOut.setIdentity();
    coordinates_.get_nor().boxMinusJac(stateIn.coordinates_.get_nor(),norCovMatTemp_);
    matOut.template block<2,2>(0,0) = norCovMatTemp_;
  }
  void print() const{
    std::cout << "Bearing vector: " << coordinates_.get_nor().getVec().transpose() << " ,depth-parameter: " << distance_.p_ << std::endl;
  }
  void setIdentity(){
    norTemp_.setIdentity();
    coordinates_.set_nor(norTemp_);
    coordinates_.set_warp_identity();
    distance_.p_ = 1.0;
  }
  void setRandom(unsigned int& s){
    norTemp_.setRandom(s);
    coordinates_.set_nor(norTemp_);
    std::default_random_engine generator (s);
    std::normal_distribution<double> distribution (0.0,1.0);
    distance_.p_ = distribution(generator) + 1.0;
    s++;
  }
  void fix(){
    norTemp_ = coordinates_.get_nor();
    norTemp_.fix();
    coordinates_.set_nor(norTemp_,false);
  }
  mtGet& get(unsigned int i = 0){
    assert(i==0);
    return *this;
  }
  const mtGet& get(unsigned int i = 0) const{
    assert(i==0);
    return *this;
  }
  void registerElementToPropertyHandler(LWF::PropertyHandler* mpPropertyHandler, const std::string& str){
    coordinates_.nor_.registerElementToPropertyHandler(mpPropertyHandler,str + name_); // Does not work properly since when loading the info file the validity of nor_/c_ is not adapted
    mpPropertyHandler->doubleRegister_.registerScalar(str + name_ + "_d", distance_.p_);
  }
};

}


#endif /* ROBOCENTRICFEATUREELEMENT_HPP_ */
