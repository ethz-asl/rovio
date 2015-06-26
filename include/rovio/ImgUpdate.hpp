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

#ifndef ROVIO_IMGUPDATE_HPP_
#define ROVIO_IMGUPDATE_HPP_

#include "kindr/rotations/RotationEigen.hpp"
#include <Eigen/Dense>
#include "lightweight_filtering/Update.hpp"
#include "lightweight_filtering/State.hpp"
#include "rovio/FilterStates.hpp"
#include "rovio/Camera.hpp"
#include "rovio/PixelOutputCF.hpp"
#include "rovio/FeatureLocationOutputCF.hpp"

namespace rot = kindr::rotations::eigen_impl;

namespace rovio {

template<typename STATE>
class ImgInnovation: public LWF::State<LWF::VectorElement<2>>{
 public:
  typedef LWF::State<LWF::VectorElement<2>> Base;
  using Base::E_;
  static constexpr unsigned int _nor = 0;
  ImgInnovation(){
    static_assert(_nor+1==E_,"Error with indices");
    this->template getName<_nor>() = "nor";
  };
  ~ImgInnovation(){};
};
template<typename STATE>
class ImgUpdateMeasAuxiliary: public LWF::AuxiliaryBase<ImgUpdateMeasAuxiliary<STATE>>{
 public:
  ImgUpdateMeasAuxiliary(){
    reset(0.0);
  };
  ~ImgUpdateMeasAuxiliary(){};
  void reset(const double t){
    imgTime_ = t;
    for(int i=0;i<STATE::nCam_;i++){
      isValidPyr_[i] = false;
    }
  }
  bool areAllValid(){
    for(int i=0;i<STATE::nCam_;i++){
      if(isValidPyr_[i] == false) return false;
    }
    return true;
  }
  ImagePyramid<STATE::nLevels_> pyr_[STATE::nCam_];
  bool isValidPyr_[STATE::nCam_];
  double imgTime_;
};
template<typename STATE>
class ImgUpdateMeas: public LWF::State<ImgUpdateMeasAuxiliary<STATE>>{
 public:
  typedef LWF::State<ImgUpdateMeasAuxiliary<STATE>> Base;
  using Base::E_;
  static constexpr unsigned int _aux = 0;
  ImgUpdateMeas(){
    static_assert(_aux+1==E_,"Error with indices");
  };
  ~ImgUpdateMeas(){};
};
template<typename STATE>
class ImgUpdateNoise: public LWF::State<LWF::VectorElement<2>>{
 public:
  typedef LWF::State<LWF::VectorElement<2>> Base;
  using Base::E_;
  static constexpr unsigned int _nor = 0;
  ImgUpdateNoise(){
    static_assert(_nor+1==E_,"Error with indices");
    this->template getName<_nor>() = "nor";
  };
  ~ImgUpdateNoise(){};
};
template<typename STATE>
class ImgOutlierDetection: public LWF::OutlierDetection<LWF::ODEntry<ImgInnovation<STATE>::template getId<ImgInnovation<STATE>::_nor>(),2>>{
};

template<typename FILTERSTATE>
class ImgUpdate: public LWF::Update<ImgInnovation<typename FILTERSTATE::mtState>,FILTERSTATE,ImgUpdateMeas<typename FILTERSTATE::mtState>,ImgUpdateNoise<typename FILTERSTATE::mtState>,
                                    ImgOutlierDetection<typename FILTERSTATE::mtState>,false>{
 public:
  typedef LWF::Update<ImgInnovation<typename FILTERSTATE::mtState>,FILTERSTATE,ImgUpdateMeas<typename FILTERSTATE::mtState>,ImgUpdateNoise<typename FILTERSTATE::mtState>,
                      ImgOutlierDetection<typename FILTERSTATE::mtState>,false> Base;
  using typename Base::eval;
  using Base::doubleRegister_;
  using Base::intRegister_;
  using Base::boolRegister_;
  using Base::updnoiP_;
  using Base::useSpecialLinearizationPoint_;
  using Base::useImprovedJacobian_;
  typedef typename Base::mtState mtState;
  typedef typename Base::mtFilterState mtFilterState;
  typedef typename Base::mtInnovation mtInnovation;
  typedef typename Base::mtMeas mtMeas;
  typedef typename Base::mtNoise mtNoise;
  typedef typename Base::mtJacInput mtJacInput;
  typedef typename Base::mtJacNoise mtJacNoise;
  typedef typename Base::mtOutlierDetection mtOutlierDetection;
  M3D initCovFeature_;
  double initDepth_;
  Camera* mpCameras_;
  PixelOutputCF<typename FILTERSTATE::mtState> pixelOutputCF_; // TODO delete/unify
  PixelOutput pixelOutput_;
  typename PixelOutputCF<typename FILTERSTATE::mtState>::mtOutputCovMat pixelOutputCov_;
  PixelOutputFromNorCF pixelOutputFromNorCF_;
  mutable rovio::FeatureLocationOutputCF<mtState> featureLocationOutputCF_;
  mutable FeatureLocationOutput featureLocationOutput_;
  mutable typename rovio::FeatureLocationOutputCF<mtState>::mtOutputCovMat featureLocationCov_;
  mutable typename rovio::FeatureLocationOutputCF<mtState>::mtJacInput featureLocationOutputJac_;
  int startLevel_;
  int endLevel_;
  double startDetectionTh_;
  int nDetectionBuckets_;
  int fastDetectionThreshold_;
  double scoreDetectionExponent_;
  double penaltyDistance_;
  double zeroDistancePenalty_;
  double trackingLocalRange_,trackingLocalVisibilityRange_;
  double trackingUpperBound_,trackingLowerBound_;
  double minTrackedAndFreeFeatures_;
  double minRelativeSTScore_;
  double minAbsoluteSTScore_;
  double matchingPixelThreshold_;
  double patchRejectionTh_;
  bool doPatchWarping_;
  bool useDirectMethod_;
  bool doFrameVisualisation_;
  bool verbose_;
  bool removeNegativeFeatureAfterUpdate_;
  double specialLinearizationThreshold_;
  ImgUpdate(){
    mpCameras_ = nullptr;
    initCovFeature_.setIdentity();
    initDepth_ = 0;
    startLevel_ = 3;
    endLevel_ = 1;
    startDetectionTh_ = 0.9;
    nDetectionBuckets_ = 100;
    fastDetectionThreshold_ = 10;
    scoreDetectionExponent_ = 0.5;
    penaltyDistance_ = 20;
    zeroDistancePenalty_ = nDetectionBuckets_*1.0;
    doPatchWarping_ = true;
    useDirectMethod_ = true;
    doFrameVisualisation_ = true;
    verbose_ = false;
    trackingLocalRange_ = 20;
    trackingLocalVisibilityRange_ = 200;
    trackingUpperBound_ = 0.9;
    trackingLowerBound_ = 0.1;
    minTrackedAndFreeFeatures_ = 0.5;
    minRelativeSTScore_ = 0.2;
    minAbsoluteSTScore_ = 0.2;
    matchingPixelThreshold_ = 4.0;
    patchRejectionTh_ = 10.0;
    removeNegativeFeatureAfterUpdate_ = true;
    specialLinearizationThreshold_ = 1.0/400;
    doubleRegister_.registerDiagonalMatrix("initCovFeature",initCovFeature_);
    doubleRegister_.registerScalar("initDepth",initDepth_);
    doubleRegister_.registerScalar("startDetectionTh",startDetectionTh_);
    doubleRegister_.registerScalar("scoreDetectionExponent",scoreDetectionExponent_);
    doubleRegister_.registerScalar("penaltyDistance",penaltyDistance_);
    doubleRegister_.registerScalar("zeroDistancePenalty",zeroDistancePenalty_);
    doubleRegister_.registerScalar("trackingLocalRange",trackingLocalRange_);
    doubleRegister_.registerScalar("trackingLocalVisibilityRange",trackingLocalVisibilityRange_);
    doubleRegister_.registerScalar("trackingUpperBound",trackingUpperBound_);
    doubleRegister_.registerScalar("trackingLowerBound",trackingLowerBound_);
    doubleRegister_.registerScalar("minTrackedAndFreeFeatures",minTrackedAndFreeFeatures_);
    doubleRegister_.registerScalar("minRelativeSTScore",minRelativeSTScore_);
    doubleRegister_.registerScalar("minAbsoluteSTScore",minAbsoluteSTScore_);
    doubleRegister_.registerScalar("matchingPixelThreshold",matchingPixelThreshold_);
    doubleRegister_.registerScalar("patchRejectionTh",patchRejectionTh_);
    doubleRegister_.registerScalar("specialLinearizationThreshold",specialLinearizationThreshold_);
    intRegister_.registerScalar("fastDetectionThreshold",fastDetectionThreshold_);
    intRegister_.registerScalar("startLevel",startLevel_);
    intRegister_.registerScalar("endLevel",endLevel_);
    intRegister_.registerScalar("nDetectionBuckets",nDetectionBuckets_);
    boolRegister_.registerScalar("doPatchWarping",doPatchWarping_);
    boolRegister_.registerScalar("useDirectMethod",useDirectMethod_);
    boolRegister_.registerScalar("doFrameVisualisation",doFrameVisualisation_);
    boolRegister_.registerScalar("removeNegativeFeatureAfterUpdate",removeNegativeFeatureAfterUpdate_);
    doubleRegister_.removeScalarByVar(updnoiP_(0,0));
    doubleRegister_.removeScalarByVar(updnoiP_(1,1));
    doubleRegister_.registerScalar("UpdateNoise.nor",updnoiP_(0,0));
    doubleRegister_.registerScalar("UpdateNoise.nor",updnoiP_(1,1));
    useImprovedJacobian_ = false; // TODO: adapt/test
  };
  ~ImgUpdate(){};
  void refreshProperties(){
    useSpecialLinearizationPoint_ = true; // TODO: make dependent
  };
  void setCamera(Camera* mpCameras){
    mpCameras_ = mpCameras;
    pixelOutputCF_.setCamera(mpCameras);
    pixelOutputFromNorCF_.setCamera(mpCameras);
  }
  void eval(mtInnovation& y, const mtState& state, const mtMeas& meas, const mtNoise noise, double dt = 0.0) const{
    const int& ID = state.template get<mtState::_aux>().activeFeature_;
    const int& camID = state.template get<mtState::_aux>().camID_[ID];
    const int& activeCamCounter = state.template get<mtState::_aux>().activeCameraCounter_;
    const int activeCamID = (activeCamCounter + camID)%mtState::nCam_;
    if(verbose_){
      std::cout << "    \033[32mMaking update with feature " << ID << " from camera " << camID << " in camera " << activeCamID << "\033[0m" << std::endl;
    }
    if(useDirectMethod_){
      y.template get<mtInnovation::_nor>() = state.template get<mtState::_aux>().b_red_[ID]+noise.template get<mtNoise::_nor>();
    } else {
      featureLocationOutputCF_.setFeatureID(ID);
      featureLocationOutputCF_.setOutputCameraID(activeCamID);
      featureLocationOutputCF_.eval(featureLocationOutput_,state,state);
      featureLocationOutput_.template get<FeatureLocationOutput::_nor>().boxMinus(state.template get<mtState::_aux>().bearingMeas_[ID],y.template get<mtInnovation::_nor>()); // 0 = m - m_meas + n
      y.template get<mtInnovation::_nor>() += noise.template get<mtNoise::_nor>();
    }
  }
  void jacInput(mtJacInput& F, const mtState& state, const mtMeas& meas, double dt = 0.0) const{
    const int& ID = state.template get<mtState::_aux>().activeFeature_;
    const int& camID = state.template get<mtState::_aux>().camID_[ID];
    const int& activeCamCounter = state.template get<mtState::_aux>().activeCameraCounter_;
    const int activeCamID = (activeCamCounter + camID)%mtState::nCam_;
    F.setZero();
    featureLocationOutputCF_.setFeatureID(ID);
    featureLocationOutputCF_.setOutputCameraID(activeCamID);
    featureLocationOutputCF_.eval(featureLocationOutput_,state,state);
    featureLocationOutputCF_.jacInput(featureLocationOutputJac_,state,state);
    if(useDirectMethod_){
      cv::Point2f c_temp;
      Eigen::Matrix2d c_J;
      mpCameras_[activeCamID].bearingToPixel(featureLocationOutput_.template get<FeatureLocationOutput::_nor>(),c_temp,c_J);
      F = -state.template get<mtState::_aux>().A_red_[ID]*c_J*featureLocationOutputJac_.template block<2,mtState::D_>(0,0);
    } else {
      F = state.template get<mtState::_aux>().bearingMeas_[ID].getN().transpose()
            *-LWF::NormalVectorElement::getRotationFromTwoNormalsJac(featureLocationOutput_.template get<FeatureLocationOutput::_nor>(),state.template get<mtState::_aux>().bearingMeas_[ID])
            *featureLocationOutput_.template get<FeatureLocationOutput::_nor>().getM()*featureLocationOutputJac_.template block<2,mtState::D_>(0,0);
    }
  }
  void jacNoise(mtJacNoise& G, const mtState& state, const mtMeas& meas, double dt = 0.0) const{
    G.setZero();
    G.template block<2,2>(mtInnovation::template getId<mtInnovation::_nor>(),mtNoise::template getId<mtNoise::_nor>()) = Eigen::Matrix2d::Identity();
  }
  void commonPreProcess(mtFilterState& filterState, const mtMeas& meas){
    assert(filterState.t_ == meas.template get<mtMeas::_aux>().imgTime_);
    for(int i=0;i<mtState::nCam_;i++){
      if(doFrameVisualisation_){
        cvtColor(meas.template get<mtMeas::_aux>().pyr_[i].imgs_[0], filterState.img_[i], CV_GRAY2RGB);
      }
    }
    filterState.imgTime_ = filterState.t_;
    filterState.imageCounter_++;
    filterState.patchDrawing_ = cv::Mat::zeros(mtState::nMax_*pow(2,mtState::nLevels_-1),mtState::nMax_*pow(2,mtState::nLevels_-1),CV_8UC1); // TODO
    filterState.state_.template get<mtState::_aux>().activeFeature_ = 0;
    filterState.state_.template get<mtState::_aux>().activeCameraCounter_ = 0;

    // TODO sort feature by covariance and use more accurate ones first
  }
  void preProcess(mtFilterState& filterState, const mtMeas& meas, bool& isFinished){
    if(isFinished){ // gets called if this is the first call
      commonPreProcess(filterState,meas);
      isFinished = false;
    }
    bool foundValidMeasurement = false;
    MultilevelPatchFeature<mtState::nLevels_,mtState::patchSize_>* mpFeature;
    MultilevelPatchFeature<mtState::nLevels_,mtState::patchSize_> patchInTargetFrame(mpCameras_);
    Eigen::Vector2d bearingError;
    typename mtFilterState::mtState& state = filterState.state_;
    typename mtFilterState::mtFilterCovMat& cov = filterState.cov_;
    int& ID = filterState.state_.template get<mtState::_aux>().activeFeature_;
    int& activeCamCounter = filterState.state_.template get<mtState::_aux>().activeCameraCounter_;

    // Actualize camera extrinsics
    for(int i=0;i<mtState::nCam_;i++){
      mpCameras_[i].setExtrinsics(state.template get<mtState::_vep>(i),state.template get<mtState::_vea>(i));
    }

    while(ID < mtState::nMax_ && foundValidMeasurement == false){
      if(filterState.mlps_.isValid_[ID]){
        // Data handling stuff
        mpFeature = &filterState.mlps_.features_[ID];
        mpFeature->fromState();
        mpFeature->setDepth(state.get_depth(ID));
        const int camID = mpFeature->camID_;
        if(activeCamCounter==0){
          mpFeature->increaseStatistics(filterState.t_);
          if(verbose_){
            std::cout << "=========== Feature " << ID << " ==================================================== " << std::endl;
          }
        }
        const int activeCamID = (activeCamCounter + camID)%mtState::nCam_;
        if(verbose_){
          std::cout << "  ========== Camera  " << activeCamID << " ================= " << std::endl;
          std::cout << "  Normal in feature frame: " << mpFeature->get_nor().getVec().transpose() << std::endl;
          std::cout << "  with depth: " << state.get_depth(ID) << std::endl;
        }

        // Get normal in target frame
        featureLocationOutputCF_.setFeatureID(ID);
        featureLocationOutputCF_.setOutputCameraID(activeCamID);
        featureLocationOutputCF_.transformState(state,featureLocationOutput_);
        featureLocationOutputCF_.transformCovMat(state,cov,featureLocationCov_);
        if(verbose_) std::cout << "    Normal in camera frame: " << featureLocationOutput_.template get<FeatureLocationOutput::_nor>().getVec().transpose() << std::endl;

        // Make patch feature in target frame
        patchInTargetFrame = *mpFeature; // TODO: make less costly
        patchInTargetFrame.set_nor(featureLocationOutput_.template get<FeatureLocationOutput::_nor>()); // TODO: do warping
        patchInTargetFrame.camID_ = activeCamID;
        bool isInActiveFrame = isMultilevelPatchInFrame(patchInTargetFrame,meas.template get<mtMeas::_aux>().pyr_[activeCamID],startLevel_,false,doPatchWarping_);
        mpFeature->status_.inFrame_ = mpFeature->status_.inFrame_ || isInActiveFrame;

        if(isInActiveFrame){
          pixelOutputFromNorCF_.setCameraID(activeCamID);
          pixelOutputFromNorCF_.transformState(featureLocationOutput_,pixelOutput_);
          pixelOutputFromNorCF_.transformCovMat(featureLocationOutput_,featureLocationCov_,pixelOutputCov_);

          // Visualization
          if(doFrameVisualisation_){
            FeatureCoordinates featureCoordinates(mpCameras_);
            featureCoordinates.set_c(pixelOutput_.getPoint2f());
            featureCoordinates.camID_ = activeCamID;
            featureCoordinates.setSigmaFromCov(pixelOutputCov_);
            if(activeCamID==camID){
              drawEllipse(filterState.img_[activeCamID], featureCoordinates, cv::Scalar(0,175,175), 2.0, false);
              drawText(filterState.img_[activeCamID],featureCoordinates,std::to_string(ID),cv::Scalar(0,175,175));
            } else {
              drawEllipse(filterState.img_[activeCamID], featureCoordinates, cv::Scalar(175,175,0), 2.0, false);
              drawText(filterState.img_[activeCamID],featureCoordinates,std::to_string(ID),cv::Scalar(175,175,0));
            }
          }

          // Logging (TODO: remove eventually)
          if(activeCamID==camID){
            pixelOutputCF_.setIndex(ID);
            pixelOutputCF_.transformCovMat(state,cov,pixelOutputCov_);
            mpFeature->setSigmaFromCov(pixelOutputCov_);
            mpFeature->log_prediction_ = static_cast<FeatureCoordinates>(*mpFeature);
            const PixelCorners& pixelCorners = mpFeature->get_pixelCorners();
            mpFeature->log_predictionC0_.set_c(mpFeature->get_c() - 4*pixelCorners[0] - 4*pixelCorners[1]);
            mpFeature->log_predictionC1_.set_c(mpFeature->get_c() + 4*pixelCorners[0] - 4*pixelCorners[1]);
            mpFeature->log_predictionC2_.set_c(mpFeature->get_c() - 4*pixelCorners[0] + 4*pixelCorners[1]);
            mpFeature->log_predictionC3_.set_c(mpFeature->get_c() + 4*pixelCorners[0] + 4*pixelCorners[1]);
          }

          // Search patch
          if(!useDirectMethod_ || true){ // TODO: make adaptive || pixelOutputCov_.operatorNorm() > matchingPixelThreshold_
            align2DComposed(patchInTargetFrame,meas.template get<mtMeas::_aux>().pyr_[activeCamID],startLevel_,endLevel_,startLevel_-endLevel_,doPatchWarping_);
          }
          if(patchInTargetFrame.status_.matchingStatus_ == FOUND){
            if(patchInTargetFrame.computeAverageDifferenceReprojection(meas.template get<mtMeas::_aux>().pyr_[activeCamID],endLevel_,startLevel_) > patchRejectionTh_){
              patchInTargetFrame.status_.matchingStatus_ = NOTFOUND;
              if(verbose_) std::cout << "    \033[31mNOT FOUND (error too large)\033[0m" << std::endl;
            } else {
              if(doFrameVisualisation_){
                drawPoint(filterState.img_[activeCamID], patchInTargetFrame, cv::Scalar(255,0,255));
              }
              if(activeCamID==camID) mpFeature->log_meas_.set_nor(patchInTargetFrame.get_nor());
              mpFeature->status_.matchingStatus_ = FOUND; // TODO: rethink status handling
              if(verbose_) std::cout << "    Found match: " << patchInTargetFrame.get_nor().getVec().transpose() << std::endl;
            }
          } else {
            if(verbose_) std::cout << "    \033[31mNOT FOUND (matching failed)\033[0m" << std::endl;
          }

          if(patchInTargetFrame.status_.matchingStatus_ == FOUND){
            // Compute deviation of expected
            patchInTargetFrame.get_nor().boxMinus(featureLocationOutput_.template get<FeatureLocationOutput::_nor>(),bearingError);
            const double weightedBearingError = (bearingError.transpose()*featureLocationCov_.template block<2,2>(FeatureLocationOutput::template getId<FeatureLocationOutput::_nor>(),FeatureLocationOutput::template getId<FeatureLocationOutput::_nor>()).inverse()*bearingError)(0,0);

            // Determine linearization mode
            useSpecialLinearizationPoint_ = bearingError.norm() > specialLinearizationThreshold_;

            if(weightedBearingError < 5.886){ // TODO: param
              if(useSpecialLinearizationPoint_) featureLocationOutput_.template get<FeatureLocationOutput::_nor>() = patchInTargetFrame.get_nor();
              mtState linearizationPoint = state;
              if(verbose_) std::cout << "    useSpecialLinearizationPoint: " << useSpecialLinearizationPoint_ << std::endl;
              if(!useSpecialLinearizationPoint_ || featureLocationOutputCF_.solveInverseProblemRelaxed(linearizationPoint,cov,featureLocationOutput_,Eigen::Matrix2d::Identity()*1e-5,1e-4,199)){ // TODO: make noide dependent on patch
                if(verbose_) std::cout << "    Backprojection: " << linearizationPoint.template get<mtState::_nor>(ID).getVec().transpose() << std::endl;
                if(useDirectMethod_ && !useSpecialLinearizationPoint_) patchInTargetFrame.set_nor(featureLocationOutput_.template get<FeatureLocationOutput::_nor>());
                if(!useDirectMethod_ || getLinearAlignEquationsReduced(patchInTargetFrame,meas.template get<mtMeas::_aux>().pyr_[activeCamID],endLevel_,startLevel_,doPatchWarping_,
                                                  state.template get<mtState::_aux>().A_red_[ID],state.template get<mtState::_aux>().b_red_[ID])){
                  if(useSpecialLinearizationPoint_) linearizationPoint.boxMinus(state,filterState.difVecLin_);
                  state.template get<mtState::_aux>().bearingMeas_[ID] = patchInTargetFrame.get_nor();
                  foundValidMeasurement = true;
                  if(doFrameVisualisation_){
                    drawPoint(filterState.img_[activeCamID], patchInTargetFrame, cv::Scalar(0,255,0));
                    if(activeCamID!=camID){
                      FeatureCoordinates featureCoordinates(mpCameras_);
                      featureCoordinates.set_nor(linearizationPoint.template get<mtState::_nor>(ID));
                      featureCoordinates.camID_ = camID;
                      drawPoint(filterState.img_[camID], featureCoordinates, cv::Scalar(255,0,0));
                    }
                  }
                } else {
                  if(verbose_) std::cout << "    \033[31mFailed construction of linear equation!\033[0m" << std::endl;
                }
              } else {
                if(verbose_) std::cout << "    \033[31mFailed backprojection!\033[0m" << std::endl;
              }
            } else {
              if(verbose_) std::cout << "    Match too far!" << std::endl;
            }
          }
        }
      }
      if(foundValidMeasurement == false){
        activeCamCounter++;
        if(activeCamCounter == mtState::nCam_){
          activeCamCounter = 0;
          ID++;
        }
      }
    }
    if(ID >= mtState::nMax_){
      isFinished = true;
    }
  };
  void postProcess(mtFilterState& filterState, const mtMeas& meas, const mtOutlierDetection& outlierDetection, bool& isFinished){
    int& ID = filterState.state_.template get<mtState::_aux>().activeFeature_;
    int& activeCamCounter = filterState.state_.template get<mtState::_aux>().activeCameraCounter_;

    if(isFinished){
      commonPostProcess(filterState,meas);
    } else {
      MultilevelPatchFeature<mtState::nLevels_,mtState::patchSize_> drawPatch(mpCameras_);
      const int camID = filterState.mlps_.features_[ID].camID_;
      const int activeCamID = (activeCamCounter + camID)%mtState::nCam_;
      if(filterState.mlps_.features_[ID].status_.trackingStatus_ == NOTTRACKED){
        filterState.mlps_.features_[ID].status_.trackingStatus_ = FAILED;
      }
      drawPatch.set_pixelCorners(filterState.mlps_.features_[ID].get_pixelCorners());
      drawPatch.set_nor(filterState.state_.template get<mtState::_nor>(ID));
      drawPatch.camID_ = activeCamID;
      if(doFrameVisualisation_ && activeCamID != camID){
        featureLocationOutputCF_.setFeatureID(ID);
        featureLocationOutputCF_.setOutputCameraID(activeCamID);
        featureLocationOutputCF_.transformState(filterState.state_,featureLocationOutput_);
        drawPatch.set_nor(featureLocationOutput_.template get<FeatureLocationOutput::_nor>());
      }
      if(!outlierDetection.isOutlier(0)){
        filterState.mlps_.features_[ID].status_.trackingStatus_ = TRACKED;
        if(doFrameVisualisation_) drawPatchBorder(filterState.img_[activeCamID],drawPatch,4.0,cv::Scalar(0,255,0));
      } else {
        if(doFrameVisualisation_) drawPatchBorder(filterState.img_[activeCamID],drawPatch,4.0,cv::Scalar(0,0,255));
      }

      // Remove negative feature
      if(removeNegativeFeatureAfterUpdate_){
        for(unsigned int i=0;i<mtState::nMax_;i++){
          if(filterState.mlps_.isValid_[i]){
            if(filterState.state_.template get<mtState::_dep>(i) < 1e-8){
              if(verbose_) std::cout << "    \033[33mRemoved feature " << i << " with invalid depth " << filterState.state_.get_depth(i) << "!\033[0m" << std::endl;
                filterState.mlps_.isValid_[i] = false;
                filterState.removeFeature(i);
            }
          }
        }
      }

      activeCamCounter++;
      if(activeCamCounter == mtState::nCam_){
        activeCamCounter = 0;
        ID++;
      }
    }
  };
  void commonPostProcess(mtFilterState& filterState, const mtMeas& meas){
    // Temps
    float averageScore;
    int countTracked;
    int requiredFreeFeature;
    double removalFactor;
    int featureIndex;
    typename mtFilterState::mtState& state = filterState.state_;
    typename mtFilterState::mtFilterCovMat& cov = filterState.cov_;
    MultilevelPatchFeature<mtState::nLevels_,mtState::patchSize_>* mpFeature;
    MultilevelPatchFeature<mtState::nLevels_,mtState::patchSize_> testFeature(mpCameras_);

    // Actualize camera extrinsics
    for(int i=0;i<mtState::nCam_;i++){
      mpCameras_[i].setExtrinsics(state.template get<mtState::_vep>(i),state.template get<mtState::_vea>(i));
    }

    // Compute the median depth parameters for each camera, using the state features.
    std::array<double, mtState::nCam_> medianDepthParameters;
    filterState.getMedianDepthParameters(initDepth_, &medianDepthParameters);

    countTracked = 0;
    for(unsigned int i=0;i<mtState::nMax_;i++){
      if(filterState.mlps_.isValid_[i]){
        mpFeature = &filterState.mlps_.features_[i];
        mpFeature->fromState(); // Has to be done for every valid feature
        mpFeature->setDepth(state.get_depth(i));
        const int camID = mpFeature->camID_;
        if(mpFeature->status_.inFrame_){
          // Logging
          pixelOutputCF_.setIndex(i);
          pixelOutputCF_.transformCovMat(state,cov,pixelOutputCov_);
          mpFeature->setSigmaFromCov(pixelOutputCov_);
          mpFeature->log_current_ = static_cast<FeatureCoordinates>(*mpFeature);

          // Count Tracked
          if(mpFeature->status_.trackingStatus_ == TRACKED){
            countTracked++;
          }

          // Extract feature patches
          if(mpFeature->status_.trackingStatus_ == TRACKED){
            if(isMultilevelPatchInFrame(*mpFeature,meas.template get<mtMeas::_aux>().pyr_[camID],startLevel_,true,false)){
              testFeature.set_c(mpFeature->get_c());
              extractMultilevelPatchFromImage(testFeature,meas.template get<mtMeas::_aux>().pyr_[camID],startLevel_,true,false);
              testFeature.computeMultilevelShiTomasiScore(endLevel_,startLevel_);
              if(testFeature.s_ >= static_cast<float>(minAbsoluteSTScore_) || testFeature.s_ >= static_cast<float>(minRelativeSTScore_)*mpFeature->s_){
                extractMultilevelPatchFromImage(*mpFeature,meas.template get<mtMeas::_aux>().pyr_[camID],startLevel_,true,false);
                mpFeature->computeMultilevelShiTomasiScore(endLevel_,startLevel_);
                mpFeature->toState();
              }
            }
          }
        }
      }
    }

    // Remove bad feature
    averageScore = filterState.mlps_.getAverageScore(); // TODO improve
    for(unsigned int i=0;i<mtState::nMax_;i++){
      if(filterState.mlps_.isValid_[i]){
        mpFeature = &filterState.mlps_.features_[i];
        if(!mpFeature->isGoodFeature(trackingLocalRange_,trackingLocalVisibilityRange_,trackingUpperBound_,trackingLowerBound_)){
          //          || fManager.features_[ind].s_ < static_cast<float>(minAbsoluteSTScore_) + static_cast<float>(minRelativeSTScore_)*averageScore){ //TODO: debug and fix
          filterState.mlps_.isValid_[i] = false;
          filterState.removeFeature(i);
        }
      }
    }

    // Check if enough free features
    requiredFreeFeature = mtState::nMax_*minTrackedAndFreeFeatures_-countTracked;
    removalFactor = 1.1; // TODO: param
    featureIndex = 0;
    while((int)(mtState::nMax_) - (int)(filterState.mlps_.getValidCount()) < requiredFreeFeature){
      if(filterState.mlps_.isValid_[featureIndex]){
        mpFeature = &filterState.mlps_.features_[featureIndex];
        if(mpFeature->status_.trackingStatus_ != TRACKED &&
          !mpFeature->isGoodFeature(trackingLocalRange_,trackingLocalVisibilityRange_,trackingUpperBound_*removalFactor,trackingLowerBound_*removalFactor)){ // TODO: improve
          filterState.mlps_.isValid_[featureIndex] = false;
          filterState.removeFeature(featureIndex);
        }
      }
      featureIndex++;
      if(featureIndex == mtState::nMax_){
        featureIndex = 0;
        removalFactor = removalFactor*1.1; // TODO: param
      }
    }

    // Get new features // TODO IMG do for both images
    const int searchCamID = rand()%mtState::nCam_;
    averageScore = filterState.mlps_.getAverageScore(); // TODO
    if(filterState.mlps_.getValidCount() < startDetectionTh_*mtState::nMax_){
      std::list<cv::Point2f> candidates;
      if(verbose_) std::cout << "Adding keypoints" << std::endl;
      const double t1 = (double) cv::getTickCount();
      for(int l=endLevel_;l<=startLevel_;l++){
        detectFastCorners(meas.template get<mtMeas::_aux>().pyr_[searchCamID],candidates,l,fastDetectionThreshold_);
      }
      const double t2 = (double) cv::getTickCount();
      if(verbose_) std::cout << "== Detected " << candidates.size() << " on levels " << endLevel_ << "-" << startLevel_ << " (" << (t2-t1)/cv::getTickFrequency()*1000 << " ms)" << std::endl;
      for(unsigned int camID=0;camID<mtState::nCam_;camID++){
        pruneCandidates(filterState.mlps_,candidates,searchCamID);
      }
      const double t3 = (double) cv::getTickCount();
      if(verbose_) std::cout << "== Selected " << candidates.size() << " candidates (" << (t3-t2)/cv::getTickFrequency()*1000 << " ms)" << std::endl;
      std::unordered_set<unsigned int> newSet = addBestCandidates(filterState.mlps_,candidates,meas.template get<mtMeas::_aux>().pyr_[searchCamID],searchCamID,filterState.t_,
                                                                  endLevel_,startLevel_,mtState::nMax_-filterState.mlps_.getValidCount(),nDetectionBuckets_, scoreDetectionExponent_,
                                                                  penaltyDistance_, zeroDistancePenalty_,false,0.0);
      const double t4 = (double) cv::getTickCount();
      if(verbose_) std::cout << "== Got " << filterState.mlps_.getValidCount() << " after adding " << newSet.size() << " features (" << (t4-t3)/cv::getTickFrequency()*1000 << " ms)" << std::endl;
      for(auto it = newSet.begin();it != newSet.end();++it){
        filterState.mlps_.features_[*it].setCamera(mpCameras_);
        filterState.mlps_.features_[*it].status_.inFrame_ = true;
        filterState.mlps_.features_[*it].status_.matchingStatus_ = FOUND;
        filterState.mlps_.features_[*it].status_.trackingStatus_ = TRACKED;
        filterState.initializeFeatureState(*it, filterState.mlps_.features_[*it].get_nor().getVec(), medianDepthParameters[searchCamID], initCovFeature_);
        filterState.mlps_.features_[*it].linkToState(&state.template get<mtState::_aux>().camID_[*it],&state.template get<mtState::_nor>(*it),&state.template get<mtState::_aux>().bearingCorners_[*it]);
        filterState.mlps_.features_[*it].toState();
      }
    }
    for(unsigned int i=0;i<mtState::nMax_;i++){
      if(filterState.mlps_.isValid_[i] && filterState.mlps_.features_[i].status_.inFrame_){
        filterState.mlps_.features_[i].log_previous_ = static_cast<FeatureCoordinates>(filterState.mlps_.features_[i]);
      }
    }
    if (doFrameVisualisation_){
      drawVirtualHorizon(filterState,0);
      drawVirtualHorizon(filterState,1);
    }

    if(verbose_){
      for(int i=0;i<mtState::nCam_;i++){
        std::cout << filterState.state_.get_qVM(i) << std::endl;
        std::cout << filterState.state_.get_MrMV(i).transpose() << std::endl;
      }
    }
  }
  void drawVirtualHorizon(mtFilterState& filterState, const int camID = 0){
    typename mtFilterState::mtState& state = filterState.state_;
    cv::rectangle(filterState.img_[camID],cv::Point2f(0,0),cv::Point2f(82,92),cv::Scalar(50,50,50),-1,8,0);
    cv::rectangle(filterState.img_[camID],cv::Point2f(0,0),cv::Point2f(80,90),cv::Scalar(100,100,100),-1,8,0);
    cv::putText(filterState.img_[camID],std::to_string(filterState.imageCounter_),cv::Point2f(5,85),cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255,0,0));
    cv::Point2f rollCenter = cv::Point2f(40,40);
    cv::Scalar rollColor1(50,50,50);
    cv::Scalar rollColor2(200,200,200);
    cv::Scalar rollColor3(120,120,120);
    cv::circle(filterState.img_[camID],rollCenter,32,rollColor1,-1,8,0);
    cv::circle(filterState.img_[camID],rollCenter,30,rollColor2,-1,8,0);
    Eigen::Vector3d Vg = (state.template get<mtState::_vea>(camID)*state.template get<mtState::_att>().inverted()).rotate(Eigen::Vector3d(0,0,-1));
    double roll = atan2(Vg(1),Vg(0))-0.5*M_PI;
    double pitch = acos(Vg.dot(Eigen::Vector3d(0,0,1)))-0.5*M_PI;
    double pixelFor10Pitch = 5.0;
    double pitchOffsetAngle = -asin(pitch/M_PI*180.0/10.0*pixelFor10Pitch/30.0);
    cv::Point2f rollVector1 = 30*cv::Point2f(cos(roll),sin(roll));
    cv::Point2f rollVector2 = cv::Point2f(25,0);
    cv::Point2f rollVector3 = cv::Point2f(10,0);
    std::vector<cv::Point> pts;
    cv::ellipse2Poly(rollCenter,cv::Size(30,30),0,(roll-pitchOffsetAngle)/M_PI*180,(roll+pitchOffsetAngle)/M_PI*180+180,1,pts);
    cv::Point *points;
    points = &pts[0];
    int nbtab = pts.size();
    cv::fillPoly(filterState.img_[camID],(const cv::Point**)&points,&nbtab,1,rollColor3);
    cv::line(filterState.img_[camID],rollCenter+rollVector2,rollCenter+rollVector3,rollColor1, 2);
    cv::line(filterState.img_[camID],rollCenter-rollVector2,rollCenter-rollVector3,rollColor1, 2);
    cv::ellipse(filterState.img_[camID],rollCenter,cv::Size(10,10),0,0,180,rollColor1,2,8,0);
    cv::circle(filterState.img_[camID],rollCenter,2,rollColor1,-1,8,0);
  }
};

}


#endif /* ROVIO_IMGUPDATE_HPP_ */
