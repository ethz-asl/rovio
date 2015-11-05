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

#ifndef POSEUPDATE_HPP_
#define POSEUPDATE_HPP_

#include "lightweight_filtering/common.hpp"
#include "lightweight_filtering/Update.hpp"
#include "lightweight_filtering/State.hpp"

namespace rovio {

class PoseInnovation: public LWF::State<LWF::VectorElement<3>,LWF::QuaternionElement>{
 public:
  static constexpr unsigned int _pos = 0;
  static constexpr unsigned int _att = _pos+1;
  PoseInnovation(){
    static_assert(_att+1==E_,"Error with indices");
  };
  virtual ~PoseInnovation(){};
  inline V3D& pos(){
    return this->template get<_pos>();
  }
  inline const V3D& pos() const{
    return this->template get<_pos>();
  }
  inline QPD& att(){
    return this->template get<_att>();
  }
  inline const QPD& att() const{
    return this->template get<_att>();
  }
};
class PoseUpdateMeas: public LWF::State<LWF::VectorElement<3>,LWF::QuaternionElement>{
 public:
  static constexpr unsigned int _pos = 0;
  static constexpr unsigned int _att = _pos+1;
  PoseUpdateMeas(){
    static_assert(_att+1==E_,"Error with indices");
  };
  virtual ~PoseUpdateMeas(){};
  inline V3D& pos(){
    return this->template get<_pos>();
  }
  inline const V3D& pos() const{
    return this->template get<_pos>();
  }
  inline QPD& att(){
    return this->template get<_att>();
  }
  inline const QPD& att() const{
    return this->template get<_att>();
  }
};
class PoseUpdateNoise: public LWF::State<LWF::VectorElement<3>,LWF::VectorElement<3>>{
 public:
  static constexpr unsigned int _pos = 0;
  static constexpr unsigned int _att = _pos+1;
  PoseUpdateNoise(){
    static_assert(_att+1==E_,"Error with indices");
    this->template getName<_pos>() = "pos";
    this->template getName<_att>() = "att";
  };
  virtual ~PoseUpdateNoise(){};
  inline V3D& pos(){
    return this->template get<_pos>();
  }
  inline const V3D& pos() const{
    return this->template get<_pos>();
  }
  inline V3D& att(){
    return this->template get<_att>();
  }
  inline const V3D& att() const{
    return this->template get<_att>();
  }
};
class PoseOutlierDetection: public LWF::OutlierDetection<LWF::ODEntry<PoseInnovation::template getId<PoseInnovation::_pos>(),6>>{
 public:
  virtual ~PoseOutlierDetection(){};
};

template<typename FILTERSTATE, int poseIndex>
class PoseUpdate: public LWF::Update<PoseInnovation,FILTERSTATE,PoseUpdateMeas,PoseUpdateNoise,PoseOutlierDetection,false>{
 public:
  typedef LWF::Update<PoseInnovation,FILTERSTATE,PoseUpdateMeas,PoseUpdateNoise,PoseOutlierDetection,false> Base;
  using Base::eval;
  using Base::intRegister_;
  using Base::doubleRegister_;
  typedef typename Base::mtState mtState;
  typedef typename Base::mtFilterState mtFilterState;
  typedef typename Base::mtInnovation mtInnovation;
  typedef typename Base::mtMeas mtMeas;
  typedef typename Base::mtNoise mtNoise;
  typedef typename Base::mtOutlierDetection mtOutlierDetection;
  static constexpr int poseIndex_ = poseIndex;
  PoseUpdate(){
    static_assert(mtState::nPose_>poseIndex_,"PoseUpdate requires enabling of addional poses in filter state");
    qVM_.setIdentity();
    MrMV_.setZero();
    doubleRegister_.registerVector("MrMV",MrMV_);
    doubleRegister_.registerQuaternion("qVM",qVM_);
    intRegister_.removeScalarByStr("maxNumIteration");
    doubleRegister_.removeScalarByStr("alpha");
    doubleRegister_.removeScalarByStr("beta");
    doubleRegister_.removeScalarByStr("kappa");
    doubleRegister_.removeScalarByStr("updateVecNormTermination");
  }
  virtual ~PoseUpdate(){}
  QPD qVM_;
  V3D MrMV_;

  void evalInnovation(mtInnovation& y, const mtState& state, const mtNoise& noise) const{
    y.pos() = state.poseLin(poseIndex_) + (state.poseRot(poseIndex_).inverted()*state.att()).rotate(V3D(state.pos()+MrMV_)) - meas_.pos() + noise.pos();
    QPD attNoise = attNoise.exponentialMap(noise.att());
    y.att() = attNoise*qVM_*state.att().inverted()*state.poseRot(poseIndex_)*meas_.att().inverted();
  }
  void jacState(MXD& F, const mtState& state) const{
    F.setZero();
    F.template block<3,3>(mtInnovation::template getId<mtInnovation::_pos>(),mtState::template getId<mtState::_pos>()) =
        MPD(state.poseRot(poseIndex_).inverted()*state.att()).matrix();
    F.template block<3,3>(mtInnovation::template getId<mtInnovation::_pos>(),mtState::template getId<mtState::_pop>(poseIndex_)) =
              M3D::Identity();
    F.template block<3,3>(mtInnovation::template getId<mtInnovation::_pos>(),mtState::template getId<mtState::_att>()) =
        MPD(state.poseRot(poseIndex_).inverted()).matrix()*gSM(state.att().rotate(V3D(state.pos()+MrMV_)));
    F.template block<3,3>(mtInnovation::template getId<mtInnovation::_pos>(),mtState::template getId<mtState::_poa>(poseIndex_)) =
        -gSM((state.poseRot(poseIndex_).inverted()*state.att()).rotate(V3D(state.pos()+MrMV_)))*MPD(state.poseRot(poseIndex_).inverted()).matrix();
    F.template block<3,3>(mtInnovation::template getId<mtInnovation::_att>(),mtState::template getId<mtState::_att>()) =
        -MPD(qVM_*state.att().inverted()).matrix();
    F.template block<3,3>(mtInnovation::template getId<mtInnovation::_att>(),mtState::template getId<mtState::_poa>(poseIndex_)) =
        MPD(qVM_*state.att().inverted()).matrix();
  }
  void jacNoise(MXD& G, const mtState& state) const{
    G.setZero();
    G.template block<3,3>(mtInnovation::template getId<mtInnovation::_pos>(),mtNoise::template getId<mtNoise::_pos>()) = M3D::Identity();
    G.template block<3,3>(mtInnovation::template getId<mtInnovation::_att>(),mtNoise::template getId<mtNoise::_att>()) = M3D::Identity();
  }
  void preProcess(mtFilterState& filterstate, const mtMeas& meas, bool& isFinished){
    isFinished = false;
  }
  void postProcess(mtFilterState& filterstate, const mtMeas& meas, const mtOutlierDetection& outlierDetection, bool& isFinished){
    isFinished = true;
  }
};

}


#endif /* POSEUPDATE_HPP_ */
