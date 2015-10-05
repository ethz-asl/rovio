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
#include "FeatureBearingOutputCF.hpp"
#include "rovio/FilterStates.hpp"
#include "rovio/Camera.hpp"
#include "rovio/PixelOutputCF.hpp"
#include "rovio/FeatureLocationOutputCF.hpp"
#include "rovio/ZeroVelocityUpdate.hpp"

namespace rot = kindr::rotations::eigen_impl;

namespace rovio {

/** \brief Class, defining the innovation.
 */
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** \brief Class, holding the image pyramids of the different cameras. @todo complete
 */
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**  \brief @todo
 */
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

  //@{
  /** \brief Get the auxiliary state of the ImgUpdateMeas.
   *
   *  \see ImgUpdateMeasAuxiliary
   *  @return the the auxiliary state of the ImgUpdateMeas.
   */
  inline ImgUpdateMeasAuxiliary<STATE>& aux(){
    return this->template get<_aux>();
  }
  inline const ImgUpdateMeasAuxiliary<STATE>& aux() const{
    return this->template get<_aux>();
  }
  //@}
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**  \brief Class holding the update noise. @todo complete
 */
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** \brief Outlier Detection.
 */
template<typename STATE>
class ImgOutlierDetection: public LWF::OutlierDetection<LWF::ODEntry<ImgInnovation<STATE>::template getId<ImgInnovation<STATE>::_nor>(),2>>{
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** \brief Class, holding image update routines for the filter.
 */
template<typename FILTERSTATE>
class ImgUpdate: public LWF::Update<ImgInnovation<typename FILTERSTATE::mtState>,FILTERSTATE,ImgUpdateMeas<typename FILTERSTATE::mtState>,ImgUpdateNoise<typename FILTERSTATE::mtState>,
ImgOutlierDetection<typename FILTERSTATE::mtState>,false>{
 public:
  typedef LWF::Update<ImgInnovation<typename FILTERSTATE::mtState>,FILTERSTATE,ImgUpdateMeas<typename FILTERSTATE::mtState>,ImgUpdateNoise<typename FILTERSTATE::mtState>,
      ImgOutlierDetection<typename FILTERSTATE::mtState>,false> Base;
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
  mutable rovio::FeatureBearingOutputCF<mtState> featureBearingOutputCF_;
  mutable FeatureBearingOutput featureBearingOutput_;
  mutable typename rovio::FeatureBearingOutputCF<mtState>::mtOutputCovMat featureBearingCov_;
  mutable typename rovio::FeatureBearingOutputCF<mtState>::mtJacInput featureBearingOutputJac_;
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
  bool doVisualMotionDetection_; /**<Do visual motion detection*/
  double rateOfMovingFeaturesTh_; /**<What percentage of feature must be moving for image motion detection*/
  double pixelCoordinateMotionTh_; /**<Threshold for detecting feature motion*/
  int minFeatureCountForNoMotionDetection_; /**<Minimum amount of feature for detecting NO image motion*/
  double patchRejectionTh_;
  bool doPatchWarping_;
  bool useDirectMethod_;  /**<If true, the innovation term is based directly on pixel intensity errors.
                              If false, the reprojection error is used for the innovation term. @todo check this*/
  bool doFrameVisualisation_;
  bool verbose_;
  bool removeNegativeFeatureAfterUpdate_;
  double specialLinearizationThreshold_;
  bool isZeroVelocityUpdateEnabled_; /**<Should zero velocity updates be performed*/
  double minTimeForZeroVelocityUpdate_;  /**<Time until zero velocity update get performed if there is no motion*/
  ZeroVelocityUpdate<FILTERSTATE> zeroVelocityUpdate_; /**<Zero velocity update, directly integrated into the img update*/
  double maxUncertaintyToDepthRatioForDepthInitialization_;

  /** \brief Constructor.
   *
   *   Loads and sets the needed parameters.
   */
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
    doVisualMotionDetection_ = false;
    rateOfMovingFeaturesTh_ = 0.5;
    pixelCoordinateMotionTh_ = 1.0;
    minFeatureCountForNoMotionDetection_ = 5;
    minTimeForZeroVelocityUpdate_ = 1.0;
    maxUncertaintyToDepthRatioForDepthInitialization_ = 0.3;
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
    doubleRegister_.registerScalar("MotionDetection.rateOfMovingFeaturesTh",rateOfMovingFeaturesTh_);
    doubleRegister_.registerScalar("MotionDetection.pixelCoordinateMotionTh",pixelCoordinateMotionTh_);
    doubleRegister_.registerScalar("maxUncertaintyToDepthRatioForDepthInitialization",maxUncertaintyToDepthRatioForDepthInitialization_);
    intRegister_.registerScalar("fastDetectionThreshold",fastDetectionThreshold_);
    intRegister_.registerScalar("startLevel",startLevel_);
    intRegister_.registerScalar("endLevel",endLevel_);
    intRegister_.registerScalar("nDetectionBuckets",nDetectionBuckets_);
    intRegister_.registerScalar("MotionDetection.minFeatureCountForNoMotionDetection",minFeatureCountForNoMotionDetection_);
    boolRegister_.registerScalar("MotionDetection.isEnabled",doVisualMotionDetection_);
    boolRegister_.registerScalar("doPatchWarping",doPatchWarping_);
    boolRegister_.registerScalar("useDirectMethod",useDirectMethod_);
    boolRegister_.registerScalar("doFrameVisualisation",doFrameVisualisation_);
    boolRegister_.registerScalar("removeNegativeFeatureAfterUpdate",removeNegativeFeatureAfterUpdate_);
    doubleRegister_.removeScalarByVar(updnoiP_(0,0));
    doubleRegister_.removeScalarByVar(updnoiP_(1,1));
    doubleRegister_.registerScalar("UpdateNoise.nor",updnoiP_(0,0));
    doubleRegister_.registerScalar("UpdateNoise.nor",updnoiP_(1,1));
    useImprovedJacobian_ = false; // TODO: adapt/test
    isZeroVelocityUpdateEnabled_ = false;
    Base::PropertyHandler::registerSubHandler("ZeroVelocityUpdate",zeroVelocityUpdate_);
    zeroVelocityUpdate_.outlierDetection_.registerToPropertyHandler(&zeroVelocityUpdate_,"MahalanobisTh");
    zeroVelocityUpdate_.doubleRegister_.registerScalar("minNoMotionTime",minTimeForZeroVelocityUpdate_);
    zeroVelocityUpdate_.boolRegister_.registerScalar("isEnabled",isZeroVelocityUpdateEnabled_);
    intRegister_.removeScalarByStr("maxNumIteration");
    doubleRegister_.removeScalarByStr("alpha");
    doubleRegister_.removeScalarByStr("beta");
    doubleRegister_.removeScalarByStr("kappa");
    doubleRegister_.removeScalarByStr("updateVecNormTermination");
  };

  /** \brief Destructor
   */
  ~ImgUpdate(){};

  /** \brief @todo
   */
  void refreshProperties(){
    useSpecialLinearizationPoint_ = true; // TODO: make dependent
  };

  /** \brief @todo
   */
  void setCamera(Camera* mpCameras){
    mpCameras_ = mpCameras;
    pixelOutputCF_.setCamera(mpCameras);
    pixelOutputFromNorCF_.setCamera(mpCameras);
  }

  /** \brief Sets the innovation term.
   *
   *  \note If \ref useDirectMethod_ is set true, the innovation term is based directly on pixel intensity errors.
   *  \note If \ref useDirectMethod_ is set false, the reprojection error is used for the innovation term.
   *  @param mtInnovation - Class, holding innovation data.
   *  @param state        - Filter %State.
   *  @param meas         - Not Used.
   *  @param noise        - Additive discrete Gaussian noise.
   *  @param dt           - Not used.
   *  @todo check this
   */
  void eval(mtInnovation& y, const mtState& state, const mtMeas& meas, const mtNoise noise, double dt = 0.0) const{
    const int& ID = state.aux().activeFeature_;  // Feature ID.
    const int& camID = state.aux().camID_[ID];   // Camera ID of the feature.
    const int& activeCamCounter = state.aux().activeCameraCounter_;
    const int activeCamID = (activeCamCounter + camID)%mtState::nCam_;
    if(verbose_){
      std::cout << "    \033[32mMaking update with feature " << ID << " from camera " << camID << " in camera " << activeCamID << "\033[0m" << std::endl;
    }
    if(useDirectMethod_){
      y.template get<mtInnovation::_nor>() = state.aux().b_red_[ID]+noise.template get<mtNoise::_nor>();
    } else {
      featureBearingOutputCF_.setFeatureID(ID);
      featureBearingOutputCF_.setOutputCameraID(activeCamID);
      featureBearingOutputCF_.eval(featureBearingOutput_,state,state);
      featureBearingOutput_.CfP().boxMinus(state.aux().bearingMeas_[ID],y.template get<mtInnovation::_nor>()); // 0 = m - m_meas + n
      y.template get<mtInnovation::_nor>() += noise.template get<mtNoise::_nor>();
    }
  }

  /** \brief Computes the Jacobian for the update step of the filter.
   *
   *  \note If \ref useDirectMethod_ is set true, the jacobian is set w.r.t. the intensity errors.
   *  \note If \ref useDirectMethod_ is set false, the jacobian is set w.r.t. the reprojection error.
   *  @param F     - Jacobian for the update step of the filter.
   *  @param state - Filter state.
   *  @param meas  - Not used.
   *  @param dt    - Not used.
   *  @todo check this
   */
  void jacInput(mtJacInput& F, const mtState& state, const mtMeas& meas, double dt = 0.0) const{
    const int& ID = state.aux().activeFeature_;
    const int& camID = state.aux().camID_[ID];
    const int& activeCamCounter = state.aux().activeCameraCounter_;
    const int activeCamID = (activeCamCounter + camID)%mtState::nCam_;
    F.setZero();
    featureBearingOutputCF_.setFeatureID(ID);
    featureBearingOutputCF_.setOutputCameraID(activeCamID);
    featureBearingOutputCF_.eval(featureBearingOutput_,state,state);
    featureBearingOutputCF_.jacInput(featureBearingOutputJac_,state,state);
    if(useDirectMethod_){
      cv::Point2f c_temp;
      Eigen::Matrix2d c_J;
      mpCameras_[activeCamID].bearingToPixel(featureBearingOutput_.CfP(),c_temp,c_J);
      F = -state.aux().A_red_[ID]*c_J*featureBearingOutputJac_.template block<2,mtState::D_>(0,0);
    } else {
      F = state.aux().bearingMeas_[ID].getN().transpose()
                *-LWF::NormalVectorElement::getRotationFromTwoNormalsJac(featureBearingOutput_.CfP(),state.aux().bearingMeas_[ID])
      *featureBearingOutput_.CfP().getM()*featureBearingOutputJac_.template block<2,mtState::D_>(0,0);
    }
  }

  /** \brief @todo
   */
  void jacNoise(mtJacNoise& G, const mtState& state, const mtMeas& meas, double dt = 0.0) const{
    G.setZero();
    G.template block<2,2>(mtInnovation::template getId<mtInnovation::_nor>(),mtNoise::template getId<mtNoise::_nor>()) = Eigen::Matrix2d::Identity();
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  /** \brief Prepares the filter state for the update.
   *
   *   @param filterState - Filter state.
   *   @param meas        - Update measurement.
   */
  void commonPreProcess(mtFilterState& filterState, const mtMeas& meas){
    assert(filterState.t_ == meas.aux().imgTime_);
    for(int i=0;i<mtState::nCam_;i++){
      if(doFrameVisualisation_){
        cvtColor(meas.aux().pyr_[i].imgs_[0], filterState.img_[i], CV_GRAY2RGB);
      }
    }
    filterState.imgTime_ = filterState.t_;
    filterState.imageCounter_++;
    filterState.patchDrawing_ = cv::Mat::zeros(mtState::nMax_*pow(2,mtState::nLevels_-1),mtState::nMax_*pow(2,mtState::nLevels_-1),CV_8UC1); // TODO
    filterState.state_.aux().activeFeature_ = 0;
    filterState.state_.aux().activeCameraCounter_ = 0;

    // TODO sort feature by covariance and use more accurate ones first

    /* Detect Image changes by looking at the feature patches between curren and previous image (both at the current feature location)
     * The maximum change of intensity is obtained if the pixel is moved along the strongest gradient.
     * The maximal singularvalue, which is equivalent to the root of the larger eigenvalue of the Hessian,
     * gives us range in which intensity change is allowed to be.
     */
    if(doVisualMotionDetection_ && filterState.imageCounter_>1){
      int totCountInFrame = 0;
      int totCountInMotion = 0;
      MultilevelPatchFeature<mtState::nLevels_,mtState::patchSize_> mlp(mpCameras_);
      for(unsigned int i=0;i<mtState::nMax_;i++){
        if(filterState.mlps_.isValid_[i]){
          mlp.set_nor(filterState.state_.CfP(i));
          mlp.camID_ = filterState.mlps_.features_[i].camID_;
          if(isMultilevelPatchInFrame(mlp,filterState.prevPyr_[mlp.camID_],startLevel_,true,false)){
            extractMultilevelPatchFromImage(mlp,filterState.prevPyr_[mlp.camID_],startLevel_,true,false);
            mlp.computeMultilevelShiTomasiScore(endLevel_,startLevel_);
            const float avgError = mlp.computeAverageDifferenceReprojection(meas.aux().pyr_[mlp.camID_],endLevel_,startLevel_,false);
            if(avgError/std::sqrt(mlp.e1_) > static_cast<float>(pixelCoordinateMotionTh_)) totCountInMotion++;
            totCountInFrame++;
          }
        }
      }
      if(rateOfMovingFeaturesTh_/totCountInMotion*totCountInFrame < 1.0 || totCountInFrame < minFeatureCountForNoMotionDetection_){
        filterState.state_.aux().timeSinceLastImageMotion_ = 0.0;
      }
    } else {
      filterState.state_.aux().timeSinceLastImageMotion_ = 0.0;
    }
  }

  /** \brief Pre-Processing for the image update.
   *
   *  Summary:
   *  1. Searches a valid MultilevelPatchFeature from the filter state.
   *  2. Transforms the MultilevelPatchFeature into the target frame.
   *  3. Executes a 2D patch alignment in the target frame. If unsuccessful go back to step 1.
   *  4. If bearing error between aligned patch and estimated patch too large, alter linearization point.
   *     Bearing vector in the state is directly altered to the new aligned position.
   *
   *  @param filterState - Filter state.
   *  @param meas        - Update measurement.
   *  @param isFinished  - True, if process has finished.
   *  @todo check and complete.
   */
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
    int& ID = filterState.state_.aux().activeFeature_;   // ID of the current updated feature!!! Initially set to 0.
    int& activeCamCounter = filterState.state_.aux().activeCameraCounter_;

    // Actualize camera extrinsics
    for(int i=0;i<mtState::nCam_;i++){
      mpCameras_[i].setExtrinsics(state.MrMC(i),state.qCM(i));
    }

    while(ID < mtState::nMax_ && foundValidMeasurement == false){
      if(filterState.mlps_.isValid_[ID]){
        // Data handling stuff
        mpFeature = &filterState.mlps_.features_[ID];  // Set the feature pointer.
        mpFeature->fromState();                        // Read the linked variable from the filter state.
        mpFeature->setDepth(state.get_depth(ID));      // Depth value.
        const int camID = mpFeature->camID_;           // Camera ID of the feature.
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
        featureBearingOutputCF_.setFeatureID(ID);
        featureBearingOutputCF_.setOutputCameraID(activeCamID);
        featureBearingOutputCF_.transformState(state,featureBearingOutput_);
        featureBearingOutputCF_.transformCovMat(state,cov,featureBearingCov_);
        if(verbose_) std::cout << "    Normal in camera frame: " << featureBearingOutput_.CfP().getVec().transpose() << std::endl;

        // Make patch feature in target frame
        patchInTargetFrame = *mpFeature; // TODO: make less costly
        patchInTargetFrame.set_nor(featureBearingOutput_.CfP()); // TODO: do warping
        patchInTargetFrame.camID_ = activeCamID;
        bool isInActiveFrame = isMultilevelPatchInFrame(patchInTargetFrame,meas.aux().pyr_[activeCamID],startLevel_,false,doPatchWarping_);
        mpFeature->status_.inFrame_ = mpFeature->status_.inFrame_ || isInActiveFrame;

        if(isInActiveFrame){
          pixelOutputFromNorCF_.setCameraID(activeCamID);
          pixelOutputFromNorCF_.transformState(featureBearingOutput_,pixelOutput_);
          pixelOutputFromNorCF_.transformCovMat(featureBearingOutput_,featureBearingCov_,pixelOutputCov_);

          // Visualization
          if(doFrameVisualisation_){
            FeatureCoordinates featureCoordinates(mpCameras_);
            featureCoordinates.set_c(pixelOutput_.getPoint2f());
            featureCoordinates.camID_ = activeCamID;
            featureCoordinates.setSigmaFromCov(pixelOutputCov_);
            if(activeCamID==camID){
              drawEllipse(filterState.img_[activeCamID], featureCoordinates, cv::Scalar(0,175,175), 2.0, false);
              drawText(filterState.img_[activeCamID],featureCoordinates,std::to_string(mpFeature->totCount_),cv::Scalar(0,175,175));
            } else {
              drawEllipse(filterState.img_[activeCamID], featureCoordinates, cv::Scalar(175,175,0), 2.0, false);
              drawText(filterState.img_[activeCamID],featureCoordinates,std::to_string(mpFeature->totCount_),cv::Scalar(175,175,0));
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
            align2DComposed(patchInTargetFrame,meas.aux().pyr_[activeCamID],startLevel_,endLevel_,startLevel_-endLevel_,doPatchWarping_);
          }
          if(patchInTargetFrame.status_.matchingStatus_ == FOUND){
            if(patchInTargetFrame.computeAverageDifferenceReprojection(meas.aux().pyr_[activeCamID],endLevel_,startLevel_,true) > patchRejectionTh_){
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
            patchInTargetFrame.get_nor().boxMinus(featureBearingOutput_.CfP(),bearingError);
            const double weightedBearingError = (bearingError.transpose()*featureBearingCov_.template block<2,2>(FeatureBearingOutput::template getId<FeatureBearingOutput::_nor>(),FeatureBearingOutput::template getId<FeatureBearingOutput::_nor>()).inverse()*bearingError)(0,0);

            // Determine linearization mode
            useSpecialLinearizationPoint_ = bearingError.norm() > specialLinearizationThreshold_;

            if(weightedBearingError < 5.886){ // TODO: param
              if(useSpecialLinearizationPoint_) featureBearingOutput_.CfP() = patchInTargetFrame.get_nor();
              mtState linearizationPoint = state;
              if(verbose_) std::cout << "    useSpecialLinearizationPoint: " << useSpecialLinearizationPoint_ << std::endl;
              if(activeCamID == camID && useSpecialLinearizationPoint_) linearizationPoint.CfP(ID) = patchInTargetFrame.get_nor();
              if(activeCamID == camID || !useSpecialLinearizationPoint_ || featureBearingOutputCF_.solveInverseProblemRelaxed(linearizationPoint,cov,featureBearingOutput_,Eigen::Matrix2d::Identity()*1e-5,1e-4,199)){ // TODO: make noide dependent on patch
                if(verbose_) std::cout << "    Backprojection: " << linearizationPoint.CfP(ID).getVec().transpose() << std::endl;
                if(useDirectMethod_ && !useSpecialLinearizationPoint_) patchInTargetFrame.set_nor(featureBearingOutput_.CfP());
                if(!useDirectMethod_ || getLinearAlignEquationsReduced(patchInTargetFrame,meas.aux().pyr_[activeCamID],endLevel_,startLevel_,doPatchWarping_,
                                                                       state.aux().A_red_[ID],state.aux().b_red_[ID])){
                  if(useSpecialLinearizationPoint_) linearizationPoint.boxMinus(state,filterState.difVecLin_);
                  state.aux().bearingMeas_[ID] = patchInTargetFrame.get_nor();
                  foundValidMeasurement = true;
                  if(doFrameVisualisation_){
                    drawPoint(filterState.img_[activeCamID], patchInTargetFrame, cv::Scalar(0,255,0));
                    if(activeCamID!=camID){
                      FeatureCoordinates featureCoordinates(mpCameras_);
                      featureCoordinates.set_nor(linearizationPoint.CfP(ID));
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
    }  // while end
    if(ID >= mtState::nMax_){
      isFinished = true;
    }
  };

  /** \brief Post-Processing for the image update.
   *
   *  Summary:
   *  1. Some drawing tasks.
   *  2. Removal of features with negative depth.
   *
   *  @param filterState      - Filter state.
   *  @param meas             - Update measurement.
   *  @param outlierDetection - Outlier detection.
   *  @param isFinished       - True, if process has finished.
   *  @todo check and complete.
   */
  void postProcess(mtFilterState& filterState, const mtMeas& meas, const mtOutlierDetection& outlierDetection, bool& isFinished){
    int& ID = filterState.state_.aux().activeFeature_;  // Get the ID of the updated feature.
    int& activeCamCounter = filterState.state_.aux().activeCameraCounter_;

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
      drawPatch.set_nor(filterState.state_.CfP(ID));
      drawPatch.camID_ = activeCamID;
      if(doFrameVisualisation_ && activeCamID != camID){
        featureBearingOutputCF_.setFeatureID(ID);
        featureBearingOutputCF_.setOutputCameraID(activeCamID);
        featureBearingOutputCF_.transformState(filterState.state_,featureBearingOutput_);
        drawPatch.set_nor(featureBearingOutput_.CfP());
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
            if(filterState.state_.dep(i) < 1e-8){
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

  /** \brief Final Post-Processing step for the image update.
   *
   *  Summary:
   *  1. For each feature in the state: Extract patches and compute Shi-Tomasi Score.
   *  2. Removal of bad features from the state.
   *  3. Get new features and add them to the state.
   *
   *  @param filterState      - Filter state.
   *  @param meas             - Update measurement.
   *  @todo check and complete.
   */
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
      mpCameras_[i].setExtrinsics(state.MrMC(i),state.qCM(i));
    }

    countTracked = 0;
    // For all features in the state.
    for(unsigned int i=0;i<mtState::nMax_;i++){
      if(filterState.mlps_.isValid_[i]){
        mpFeature = &filterState.mlps_.features_[i];
        mpFeature->fromState();                       // Has to be done for every valid feature.
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

          // Extract feature patches and update Shi-Tomasi score.
          if(mpFeature->status_.trackingStatus_ == TRACKED){
            if(isMultilevelPatchInFrame(*mpFeature,meas.aux().pyr_[camID],startLevel_,true,false)){
              testFeature.set_c(mpFeature->get_c());
              extractMultilevelPatchFromImage(testFeature,meas.aux().pyr_[camID],startLevel_,true,false);
              testFeature.computeMultilevelShiTomasiScore(endLevel_,startLevel_);
              if(testFeature.s_ >= static_cast<float>(minAbsoluteSTScore_) || testFeature.s_ >= static_cast<float>(minRelativeSTScore_)*mpFeature->s_){
                extractMultilevelPatchFromImage(*mpFeature,meas.aux().pyr_[camID],startLevel_,true,false);
                mpFeature->computeMultilevelShiTomasiScore(endLevel_,startLevel_);
                mpFeature->toState();
              }
            }
          }
        }
      }
    }

    // Remove bad feature.
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
      // Compute the median depth parameters for each camera, using the state features.
      std::array<double, mtState::nCam_> medianDepthParameters;
      if(maxUncertaintyToDepthRatioForDepthInitialization_>0){
        filterState.getMedianDepthParameters(initDepth_, &medianDepthParameters,maxUncertaintyToDepthRatioForDepthInitialization_);
      } else {
        medianDepthParameters.fill(initDepth_);
      }

      // Get Candidates
      std::list<cv::Point2f> candidates;
      if(verbose_) std::cout << "Adding keypoints" << std::endl;
      const double t1 = (double) cv::getTickCount();
      for(int l=endLevel_;l<=startLevel_;l++){
        detectFastCorners(meas.aux().pyr_[searchCamID],candidates,l,fastDetectionThreshold_);
      }
      const double t2 = (double) cv::getTickCount();
      if(verbose_) std::cout << "== Detected " << candidates.size() << " on levels " << endLevel_ << "-" << startLevel_ << " (" << (t2-t1)/cv::getTickFrequency()*1000 << " ms)" << std::endl;
      for(unsigned int camID=0;camID<mtState::nCam_;camID++){
        pruneCandidates(filterState.mlps_,candidates,searchCamID);
      }
      const double t3 = (double) cv::getTickCount();
      if(verbose_) std::cout << "== Selected " << candidates.size() << " candidates (" << (t3-t2)/cv::getTickFrequency()*1000 << " ms)" << std::endl;
      std::unordered_set<unsigned int> newSet = addBestCandidates(filterState.mlps_,candidates,meas.aux().pyr_[searchCamID],searchCamID,filterState.t_,
                                                                  endLevel_,startLevel_,mtState::nMax_-filterState.mlps_.getValidCount(),nDetectionBuckets_, scoreDetectionExponent_,
                                                                  penaltyDistance_, zeroDistancePenalty_,false,0.0);
      const double t4 = (double) cv::getTickCount();
      if(verbose_) std::cout << "== Got " << filterState.mlps_.getValidCount() << " after adding " << newSet.size() << " features (" << (t4-t3)/cv::getTickFrequency()*1000 << " ms)" << std::endl;
      for(auto it = newSet.begin();it != newSet.end();++it){
        filterState.mlps_.features_[*it].setCamera(mpCameras_);
        filterState.mlps_.features_[*it].status_.inFrame_ = true;
        filterState.mlps_.features_[*it].status_.matchingStatus_ = FOUND;
        filterState.mlps_.features_[*it].status_.trackingStatus_ = TRACKED;
        filterState.initializeFeatureState(*it, filterState.mlps_.features_[*it].get_nor().getVec(), medianDepthParameters[searchCamID], initCovFeature_); // TODO: adapt covariance to initial depth
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
        std::cout << filterState.state_.qCM(i) << std::endl;
        std::cout << filterState.state_.MrMC(i).transpose() << std::endl;
      }
    }

    // Copy image pyramid to state
    for(int i=0;i<mtState::nCam_;i++){
      filterState.prevPyr_[i] = meas.aux().pyr_[i];
    }

    // Zero Velocity updates if appropriate
    if(isZeroVelocityUpdateEnabled_ && filterState.state_.aux().timeSinceLastImageMotion_ > minTimeForZeroVelocityUpdate_ && filterState.state_.aux().timeSinceLastInertialMotion_ > minTimeForZeroVelocityUpdate_){
      cv::putText(filterState.img_[0],"Performing Zero Velocity Updates!",cv::Point2f(150,25),cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0,255,255));
      zeroVelocityUpdate_.performUpdateEKF(filterState,ZeroVelocityUpdateMeas<mtState>());
    }
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  /** \brief Draws a virtual horizon into the current image of the camera with ID camID
   *
   *  @param filterState - Filter state.
   *  @param camID       - ID of the camera, in which image the horizon should be drawn.
   */
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
    Eigen::Vector3d Vg = (state.qCM(camID)*state.qWM().inverted()).rotate(Eigen::Vector3d(0,0,-1));
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
