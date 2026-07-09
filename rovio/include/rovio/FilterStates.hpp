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

#ifndef ROVIO_FILTERSTATES_HPP_
#define ROVIO_FILTERSTATES_HPP_

#include "lightweight_filtering/common.hpp"
#include "lightweight_filtering/FilterState.hpp"
#include <map>
#include <unordered_set>
#include "CoordinateTransform/FeatureOutput.hpp"
#include "rovio/RobocentricFeatureElement.hpp"
#include "rovio/FeatureManager.hpp"
#include "rovio/MultiCamera.hpp"

namespace rovio {

/** \brief Class, defining the auxiliary state of the filter.
 *
 *  \see State
 *
 *  @tparam nMax      - Maximal number of considered features in the filter state.
 *  @tparam nLevels   - Total number of pyramid levels considered.
 *  @tparam patchSize - Edge length of the patches (in pixel). Must be a multiple of 2!
 *  @tparam nCam      - Used total number of cameras.
 */
template<unsigned int nMax, int nLevels, int patchSize, int nCam>
class StateAuxiliary: public LWF::AuxiliaryBase<StateAuxiliary<nMax,nLevels,patchSize,nCam>>{
 public:
  /** \brief Constructor
   */
  StateAuxiliary(){
    MwWMest_.setZero();
    MwWMmeas_.setZero();
    wMeasCov_.setIdentity();
    for(unsigned int i=0;i<nMax;i++){
      A_red_[i].setIdentity();
      b_red_[i].setZero();
    }
    doVECalibration_ = true;
    activeFeature_ = 0;
    activeCameraCounter_ = 0;
    timeSinceLastInertialMotion_ = 0.0;
    timeSinceLastImageMotion_ = 0.0;
    for(unsigned int i=0;i<nCam;i++){
      qCM_[i].setIdentity();
      MrMC_[i].setZero();
    }
    poseMeasRot_.setIdentity();
    poseMeasLin_.setZero();
  };

  /** \brief Destructor
   */
  virtual ~StateAuxiliary(){};

  V3D MwWMest_;  /**<Estimated rotational rate.*/
  V3D MwWMmeas_;  /**<Measured rotational rate.*/
  M3D wMeasCov_;  /**<Covariance of the measured rotational rate.*/
  Eigen::Matrix2d A_red_[nMax];  /**<Reduced Jacobian of the pixel intensities w.r.t. to pixel coordinates, needed for the multilevel patch alignment. \see rovio::MultilevelPatchFeature::A_ \see rovio::getLinearAlignEquationsReduced()*/
  Eigen::Vector2d b_red_[nMax];  /**<Reduced intensity errors, needed for the multilevel patch alignment. \see rovio::MultilevelPatchFeature::A_ \see rovio::getLinearAlignEquationsReduced()*/
  FeatureCoordinates feaCoorMeas_[nMax];  /**<Intermediate variable for storing the measured feature location.*/
  QPD qCM_[nCam];  /**<Quaternion Array: IMU coordinates to camera coordinates.*/
  V3D MrMC_[nCam];  /**<Position Vector Array: Vectors pointing from IMU to the camera frame, expressed in the IMU frame.*/
  bool doVECalibration_;  /**<Do Camera-IMU extrinsic parameter calibration?*/
  int activeFeature_;  /**< Active Feature ID. ID of the currently updated feature. Needed in the image update procedure.*/
  int activeCameraCounter_;  /**<Counter for iterating through the cameras, used such that when updating a feature we always start with the camId where the feature is expressed in.*/
  double timeSinceLastInertialMotion_;  /**<Time since the IMU showed motion last.*/
  double timeSinceLastImageMotion_;  /**<Time since the Image showed motion last.*/
  QPD poseMeasRot_; /**<Groundtruth attitude measurement. qMI.*/
  Eigen::Vector3d poseMeasLin_; /**<Groundtruth position measurement. IrIM*/
  FeatureManager<nLevels,patchSize,nCam>* mpCurrentFeature_; /**<Pointer to active feature*/
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** \brief Filter State.
 *
 *  @tparam nMax      - Maximal number of considered features in the filter state.
 *  @tparam nLevels   - Total number of pyramid levels considered.
 *  @tparam patchSize - Edge length of the patches (in pixel). Must be a multiple of 2!
 *  @tparam nCam      - Used total number of cameras.
 *  @tparam nPose     - Additional 6D pose in state.
 */
template<unsigned int nMax, int nLevels, int patchSize, int nCam, int nPose>
class State: public LWF::State<
LWF::TH_multiple_elements<LWF::VectorElement<3>,4>,
LWF::QuaternionElement,
LWF::ArrayElement<LWF::VectorElement<3>,nCam>,
LWF::ArrayElement<LWF::QuaternionElement,nCam>,
LWF::ArrayElement<RobocentricFeatureElement,nMax>,
LWF::ArrayElement<LWF::VectorElement<3>,nPose>,
LWF::ArrayElement<LWF::QuaternionElement,nPose>,
StateAuxiliary<nMax,nLevels,patchSize,nCam>>{
 public:
  typedef LWF::State<
      LWF::TH_multiple_elements<LWF::VectorElement<3>,4>,
      LWF::QuaternionElement,
      LWF::ArrayElement<LWF::VectorElement<3>,nCam>,
      LWF::ArrayElement<LWF::QuaternionElement,nCam>,
      LWF::ArrayElement<RobocentricFeatureElement,nMax>,
      LWF::ArrayElement<LWF::VectorElement<3>,nPose>,
      LWF::ArrayElement<LWF::QuaternionElement,nPose>,
      StateAuxiliary<nMax,nLevels,patchSize,nCam>> Base;  /**<State definition.*/
  using Base::D_;
  using Base::E_;
  static constexpr int nMax_ = nMax;            /**<Max number of features.*/
  static constexpr int nLevels_ = nLevels;      /**<Max number of image levels.*/
  static constexpr int patchSize_ = patchSize;  /**<Patch size.*/
  static constexpr int nCam_ = nCam;            /**<Total number of cameras.*/
  static constexpr int nPose_ = nPose;          /**<Total number of addtional pose states.*/
  static constexpr unsigned int _pos = 0;       /**<Idx. Position Vector WrWM: Pointing from the World-Frame to the IMU-Frame, expressed in World-Coordinates.*/
  static constexpr unsigned int _vel = _pos+1;  /**<Idx. Velocity Vector MvM: Absolute velocity of the of the IMU-Frame, expressed in IMU-Coordinates.*/
  static constexpr unsigned int _acb = _vel+1;  /**<Idx. Additive bias on accelerometer.*/
  static constexpr unsigned int _gyb = _acb+1;  /**<Idx. Additive bias on gyroscope.*/
  static constexpr unsigned int _att = _gyb+1;  /**<Idx. Quaternion qWM: IMU coordinates to World coordinates.*/
  static constexpr unsigned int _vep = _att+1;  /**<Idx. Position Vector MrMC: Pointing from the IMU-Frame to the Camera-Frame, expressed in IMU-Coordinates.*/
  static constexpr unsigned int _vea = _vep+1;  /**<Idx. Quaternion qCM: IMU-Coordinates to Camera-Coordinates.*/
  static constexpr unsigned int _fea = _vea+1;  /**<Idx. Robocentric feature parametrization.*/
  static constexpr unsigned int _pop = _fea+1;  /**<Idx. Additonial pose in state, linear part. IrIW.*/
  static constexpr unsigned int _poa = _pop+1;  /**<Idx. Additonial pose in state, rotational part. qWI.*/
  static constexpr unsigned int _aux = _poa+1;  /**<Idx. Auxiliary state.*/

  /** \brief Constructor
   */
  State(){
    static_assert(_aux+1==E_,"Error with indices");
    this->template getName<_pos>() = "pos";
    this->template getName<_vel>() = "vel";
    this->template getName<_acb>() = "acb";
    this->template getName<_gyb>() = "gyb";
    this->template getName<_att>() = "att";
    this->template getName<_vep>() = "vep";
    this->template getName<_vea>() = "vea";
    this->template getName<_fea>() = "fea";
    this->template getName<_pop>() = "pop";
    this->template getName<_poa>() = "poa";
    this->template getName<_aux>() = "auxiliary";
  }

  /** \brief Destructor
   */
  virtual ~State(){};

  /** \brief Initializes the feature manager pointer as far as possible
   *
   *  @param featureManager* A pointer to a featureManagerArray
   */
  void initFeatureManagers(FeatureManager<nLevels,patchSize,nCam>** mpFeatureManager){
    for(int i=0;i<nMax;i++){
      mpFeatureManager[i]->mpCoordinates_ = &CfP(i);
      mpFeatureManager[i]->mpDistance_ = &dep(i);
    }
  }

  /** \brief Initializes the feature manager pointer as far as possible
   *
   *  @param featureManager* A pointer to a featureManagerArray
   */
  void initFeatureManagers(FeatureSetManager<nLevels,patchSize,nCam,nMax>& fsm){
    for(int i=0;i<nMax;i++){
      fsm.features_[i].mpCoordinates_ = &CfP(i);
      fsm.features_[i].mpDistance_ = &dep(i);
    }
  }

  //@{
  /** \brief Get/Set the position vector pointing from the World-Frame to the IMU-Frame, expressed in World-Coordinates (World->IMU, expressed in World).
   *
   *  @return a reference to the position vector WrWM (World->IMU, expressed in World).
   */
  inline V3D& WrWM(){
    return this->template get<_pos>();
  }
  inline const V3D& WrWM() const{
    return this->template get<_pos>();
  }
  //@}

  //@{
  /** \brief Get/Set the absolute velocity vector of the IMU-Frame MvM, expressed in IMU-Coordinates.
   *
   *  @return a reference to the absolute velocity vector of the IMU-Frame MvM, expressed in IMU-Coordinates.
   */
  inline V3D& MvM(){
    return this->template get<_vel>();
  }
  inline const V3D& MvM() const{
    return this->template get<_vel>();
  }
  //@}

  //@{
  /** \brief Get/Set the Additive bias on accelerometer acb.
   *
   *  @return a reference to the additive bias on accelerometer acb.
   */
  inline V3D& acb(){
    return this->template get<_acb>();
  }
  inline const V3D& acb() const{
    return this->template get<_acb>();
  }
  //@}

  //@{
  /** \brief Get/Set the additive bias on gyroscope gyb.
    *
    *  @return a reference to the additive bias on gyroscope gyb.
    */
  inline V3D& gyb(){
    return this->template get<_gyb>();
  }
  inline const V3D& gyb() const{
    return this->template get<_gyb>();
  }
  //@}

  //@{
  /** \brief Get/Set the quaternion qWM, expressing the  IMU-Frame in World-Coordinates (IMU Coordinates->World Coordinates).
   *
   *  @return a reference to the quaternion qWM (IMU Coordinates->World Coordinates).
   */
  inline QPD& qWM(){
    return this->template get<_att>();
  }
  inline const QPD& qWM() const{
    return this->template get<_att>();
  }
  //@}

  //@{
  /** \brief Get/Set the feature coordinates belonging to a specific feature i.
   *
   *  @param i - Feature Index
   *  @return a reference to the feature coordinates of feature i.
   */
  inline FeatureCoordinates& CfP(const int i = 0) {
    assert(i<nMax_);
    return this->template get<_fea>(i).coordinates_;
  }
  inline const FeatureCoordinates& CfP(const int i = 0) const{
    assert(i<nMax_);
    return this->template get<_fea>(i).coordinates_;
  }
  //@}

  //@{
  /** \brief Get/Set the quaternion qCM, expressing the IMU-Frame in Camera-Coordinates (IMU Coordinates->%Camera Coordinates).
   *
   *  @param camID - %Camera ID
   *  @return a reference to the quaternion qCM (IMU Coordinates->%Camera Coordinates).
   */
  inline QPD& qCM(const int camID = 0){
    assert(camID<nCam_);
    if(this->template get<_aux>().doVECalibration_){
          return this->template get<_vea>(camID);
        } else {
          return this->template get<_aux>().qCM_[camID];
        }
  }
  inline const QPD& qCM(const int camID = 0) const{
    assert(camID<nCam_);
    if(this->template get<_aux>().doVECalibration_){
      return this->template get<_vea>(camID);
    } else {
      return this->template get<_aux>().qCM_[camID];
    }
  }
  //@}

  //@{
  /** \brief Get/Set the position vector pointing from the IMU-Frame to the Camera-Frame, expressed in IMU-Coordinates (IMU->%Camera, expressed in IMU).
   *
   *  @param camID - %Camera ID
   *  @return a reference to the position vector MrMC (IMU->%Camera, expressed in IMU).
   */
  inline V3D& MrMC(const int camID = 0){
    assert(camID<nCam_);
    if(this->template get<_aux>().doVECalibration_){
      return this->template get<_vep>(camID);
    } else {
      return this->template get<_aux>().MrMC_[camID];
    }
  }
  inline const V3D& MrMC(const int camID = 0) const{
    assert(camID<nCam_);
    if(this->template get<_aux>().doVECalibration_){
      return this->template get<_vep>(camID);
    } else {
      return this->template get<_aux>().MrMC_[camID];
    }
  }
  //@}

  //@{
  /** \brief Get the position vector pointing from the World-Frame to the Camera-Frame, expressed in World-Coordinates (World->%Camera, expressed in World).
   *
   *  @param camID - %Camera ID
   *  @return the position vector WrWC (World->%Camera, expressed in World).
   */
  inline V3D WrWC(const int camID = 0) const{
    assert(camID<nCam_);
    return this->template get<_pos>()+this->template get<_att>().rotate(MrMC(camID));
  }

  //@}

  //@{
  /** \brief Get the quaternion qCW, expressing the World-Frame in Camera-Coordinates (World Coordinates->%Camera Coordinates).
   *
   *  @param camID - %Camera ID
   *  @return he quaternion qCW (World Coordinates->%Camera Coordinates).
   */
  inline QPD qCW(const int camID = 0) const{
    assert(camID<nCam_);
    return qCM(camID)*this->template get<_att>().inverted();
  }
  //@}

  //@{
  /** \brief Get/Set the distance parameter of a specific feature i.
   *
   *  \note The distance parameter can be either defined as regular distance, inverse distance, logarithmic distance or hyperbolic distance.
   *        The kind of the distance encoding depends on the defined DepthMap.
   *
   *  @param i - Feature Index
   *  @return a reference to distance parameter of the feature.
   */
  inline FeatureDistance& dep(const int i){
    assert(i<nMax_);
    return this->template get<_fea>(i).distance_;
  }
  inline const FeatureDistance& dep(const int i) const{
    assert(i<nMax_);
    return this->template get<_fea>(i).distance_;
  }
  //@}

  //@{
  /** \brief Get/Set the rotational part of the pose with index i.
   *
   *  @param i - Pose index
   *  @return  a reference to the rotational part of the pose with index i.
   */
  inline QPD& poseRot(const int i){
    assert(i<nPose_);
    return this->template get<_poa>(i);
  }
  inline const QPD& poseRot(const int i) const{
    assert(i<nPose_);
    return this->template get<_poa>(i);
  }
  //@}

  //@{
  /** \brief Get/Set the linear part of the pose with index i.
   *
   *  @param i - Pose index
   *  @return a reference to the linear part of the pose with index i.
   */
  inline V3D& poseLin(const int i = 0){
    assert(i<nPose_);
    return this->template get<_pop>(i);
  }
  inline const V3D& poseLin(const int i = 0) const{
    assert(i<nPose_);
    return this->template get<_pop>(i);
  }
  //@}

  //@{
  /** \brief Get the position vector pointing from the World-Frame to the Camera-Frame, expressed in World-Coordinates (World->%Camera, expressed in World).
   *
   *  @note - This is compute based on the external pose measurement
   *  @return the position vector WrWC (World->%Camera, expressed in World).
   */
  inline V3D WrWC_ext() const{
    return this->aux().poseMeasLin_;
  }

  //@}

  //@{
  /** \brief Get the quaternion qCW, expressing the World-Frame in Camera-Coordinates (World Coordinates->%Camera Coordinates).
   *
   *  @note - This is compute based on the external pose measurement
   *  @return he quaternion qCW (World Coordinates->%Camera Coordinates).
   */
  inline QPD qCW_ext() const{
    return this->aux().poseMeasRot_;
  }
  //@}

  //@{
  /** \brief Get the auxiliary state.
   *
   *  \see StateAuxiliary;
   *  @return a reference to the auxiliary state.
   */
  inline StateAuxiliary<nMax,nLevels,patchSize,nCam>& aux(){
    return this->template get<_aux>();
  }
  inline const StateAuxiliary<nMax,nLevels,patchSize,nCam>& aux() const{
    return this->template get<_aux>();
  }
  //@}

  /** \brief Update the extrinsics of the MultiCamera
   *
   *  @param mpMultiCamera - Pointer to the multicamera object
   */
  void updateMultiCameraExtrinsics(MultiCamera<nCam>* mpMultiCamera) const{
    for(int i=0;i<nCam;i++){
      mpMultiCamera->BrBC_[i] = MrMC(i);
      mpMultiCamera->qCB_[i] = qCM(i);
    }
  }
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** \brief Class, holding the prediction measurement of the filter.
 */
class PredictionMeas: public LWF::State<LWF::VectorElement<3>,LWF::VectorElement<3>>{
 public:
  static constexpr unsigned int _acc = 0;  /**<Index: Acceleration*/
  static constexpr unsigned int _gyr = _acc+1;   /**<Index: Angular Velocity*/
  /** \brief Constructor
   */
  PredictionMeas(){
    static_assert(_gyr+1==E_,"Error with indices");
    this->template getName<_acc>() = "acc";
    this->template getName<_gyr>() = "gyr";
  }
  /** \brief Destructor
   */
  virtual ~PredictionMeas(){};
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** \brief Class, holding the prediction noise for the state members.
 *
 *  \see State
 *  @tparam STATE - Filter State
 */
template<typename STATE>
class PredictionNoise: public LWF::State<LWF::TH_multiple_elements<LWF::VectorElement<3>,5>,
LWF::ArrayElement<LWF::VectorElement<3>,STATE::nCam_>,
LWF::ArrayElement<LWF::VectorElement<3>,STATE::nCam_>,
LWF::ArrayElement<LWF::VectorElement<3>,STATE::nMax_>,
LWF::ArrayElement<LWF::VectorElement<3>,STATE::nPose_>,
LWF::ArrayElement<LWF::VectorElement<3>,STATE::nPose_>>{
 public:
  using LWF::State<LWF::TH_multiple_elements<LWF::VectorElement<3>,5>,
      LWF::ArrayElement<LWF::VectorElement<3>,STATE::nCam_>,
      LWF::ArrayElement<LWF::VectorElement<3>,STATE::nCam_>,
      LWF::ArrayElement<LWF::VectorElement<3>,STATE::nMax_>,
      LWF::ArrayElement<LWF::VectorElement<3>,STATE::nPose_>,
      LWF::ArrayElement<LWF::VectorElement<3>,STATE::nPose_>>::E_;
  static constexpr unsigned int _pos = 0;       /**<Idx. Position Vector WrWM: Pointing from the World-Frame to the IMU-Frame, expressed in World-Coordinates.*/
  static constexpr unsigned int _vel = _pos+1;  /**<Idx. Velocity Vector MvM: Absolute velocity of the IMU-Frame, expressed in IMU-Coordinates.*/
  static constexpr unsigned int _acb = _vel+1;  /**<Idx. Additive bias on accelerometer.*/
  static constexpr unsigned int _gyb = _acb+1;  /**<Idx. Additive bias on gyroscope.*/
  static constexpr unsigned int _att = _gyb+1;  /**<Idx. Quaternion qWM: IMU coordinates to World coordinates.*/
  static constexpr unsigned int _vep = _att+1;  /**<Idx. Position Vector MrMC: Pointing from the IMU-Frame to the Camera-Frame, expressed in IMU-Coordinates.*/
  static constexpr unsigned int _vea = _vep+1;  /**<Idx. Quaternion qCM: IMU-Coordinates to Camera-Coordinates.*/
  static constexpr unsigned int _fea = _vea+1;  /**<Idx. Feature parametrizations (bearing + depth parameter), array.*/
  static constexpr unsigned int _pop = _fea+1;  /**<Idx. Additonial pose in state, linear part.*/
  static constexpr unsigned int _poa = _pop+1;  /**<Idx. Additonial pose in state, rotational part.*/

  /** \brief Constructor
   */
  PredictionNoise(){
    static_assert(_poa+1==E_,"Error with indices");
    this->template getName<_pos>() = "pos";
    this->template getName<_vel>() = "vel";
    this->template getName<_acb>() = "acb";
    this->template getName<_gyb>() = "gyb";
    this->template getName<_att>() = "att";
    this->template getName<_vep>() = "vep";
    this->template getName<_vea>() = "vea";
    this->template getName<_fea>() = "fea";
    this->template getName<_pop>() = "pop";
    this->template getName<_poa>() = "poa";
  }

  /** \brief Destructor
   */
  virtual ~PredictionNoise(){};
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** \brief Class defining the overall filter state (state, prediction measurement, noise).
 *
 *  @tparam nMax      - Maximal number of considered features in the filter state.
 *  @tparam nLevels   - Total number of pyramid levels considered.
 *  @tparam patchSize - Edge length of the patches (in pixel). Must be a multiple of 2!
 *  @tparam nCam      - Used total number of cameras.
 *  @tparam nPose     - Additional 6D pose in state.
 */
template<unsigned int nMax, int nLevels, int patchSize,int nCam,int nPose>
class FilterState: public LWF::FilterState<State<nMax,nLevels,patchSize,nCam,nPose>,PredictionMeas,PredictionNoise<State<nMax,nLevels,patchSize,nCam,nPose>>,0>{
 public:
  typedef LWF::FilterState<State<nMax,nLevels,patchSize,nCam,nPose>,PredictionMeas,PredictionNoise<State<nMax,nLevels,patchSize,nCam,nPose>>,0> Base;
  typedef typename Base::mtState mtState;  /**<Local Filter %State Type. \see LWF::FilterState*/
  using Base::state_;  /**<Filter State. \see LWF::FilterState*/
  using Base::cov_;  /**<Filter State Covariance Matrix. \see LWF::FilterState*/
  using Base::usePredictionMerge_;  /**<Whether multiple subsequent prediction steps should be merged into one.*/
  FeatureSetManager<nLevels,patchSize,nCam,nMax> fsm_;
  mutable rovio::TransformFeatureOutputCT<mtState> transformFeatureOutputCT_;
  mutable FeatureOutput featureOutput_;
  mutable MXD featureOutputCov_;
  cv::Mat img_[nCam];     /**<Mainly used for drawing.*/
  cv::Mat patchDrawing_;  /**<Mainly used for drawing.*/
  int drawPB_;  /**<Size of border around patch.*/
  int drawPS_;  /**<Size of patch with border for drawing.*/
  double imgTime_;        /**<Time of the last image, which was processed.*/
  int imageCounter_;      /**<Total number of images, used so far for updates. Same as total number of update steps.*/
  ImagePyramid<nLevels> prevPyr_[nCam]; /**<Previous image pyramid.*/
  bool plotPoseMeas_; /**<Should the pose measurement be plotted.*/
  mutable MultilevelPatch<nLevels,patchSize> mlpErrorLog_[nMax];  /**<Multilevel patch containing log of error.*/

  /** \brief Constructor
   */
  FilterState():fsm_(nullptr), transformFeatureOutputCT_(nullptr), featureOutputCov_((int)(FeatureOutput::D_),(int)(FeatureOutput::D_)){
    usePredictionMerge_ = true;
    imgTime_ = 0.0;
    imageCounter_ = 0;
    plotPoseMeas_ = true;
    state_.initFeatureManagers(fsm_);
    fsm_.allocateMissing();
    drawPB_ = 1;
    drawPS_ = mtState::patchSize_*pow(2,mtState::nLevels_-1)+2*drawPB_;
  }

  /** \brief Destructor
   */
  virtual ~FilterState(){};

  /** \brief Sets the multicamera pointer
   *
   * @param mpMultiCamera - multicamera pointer;
   */
  void setCamera(MultiCamera<nCam>* mpMultiCamera){
    fsm_.setCamera(mpMultiCamera);
    transformFeatureOutputCT_.mpMultiCamera_ = mpMultiCamera;
  }

  /** \brief Initializes the FilterState \ref Base::state_ with the IMU-Pose.
   *
   *  @param WrWM - Position Vector, pointing from the World-Frame to the IMU-Frame, expressed in World-Coordinates.
   *  @param qMW  - Quaternion, expressing World-Frame in IMU-Coordinates (World Coordinates->IMU Coordinates)
   */
  void initWithImuPose(V3D WrWM, QPD qMW){
    state_.WrWM() = WrWM;
    state_.qWM()  = qMW.inverted();
  }

  /** \brief Initializes the FilterState \ref Base::state_ with the Acceleration-Vector.
   *
   *  @param fMeasInit - Acceleration-Vector which should be used for initializing the attitude of the IMU.
   */
  void initWithAccelerometer(const V3D& fMeasInit){
    V3D unitZ(0,0,1);
    if(fMeasInit.norm()>1e-6){
      state_.qWM().setFromVectors(fMeasInit,unitZ);
    } else {
      state_.qWM().setIdentity();
    }
  }

  /** \brief Resets the covariance of a feature
   *
   *  @param i       - Feature index.
   *  @param initCov - Initialization 3x3 Covariance-Matrix.
   */
  void resetFeatureCovariance(unsigned int i,const Eigen::Matrix<double,3,3>& initCov){
    cov_.template block<mtState::D_,1>(0,mtState::template getId<mtState::_fea>(i)+2).setZero();
    cov_.template block<1,mtState::D_>(mtState::template getId<mtState::_fea>(i)+2,0).setZero();
    cov_.template block<mtState::D_,2>(0,mtState::template getId<mtState::_fea>(i)).setZero();
    cov_.template block<2,mtState::D_>(mtState::template getId<mtState::_fea>(i),0).setZero();
    cov_.template block<1,1>(mtState::template getId<mtState::_fea>(i)+2,mtState::template getId<mtState::_fea>(i)+2) = initCov.block<1,1>(0,0);
    cov_.template block<1,2>(mtState::template getId<mtState::_fea>(i)+2,mtState::template getId<mtState::_fea>(i)) = initCov.block<1,2>(0,1);
    cov_.template block<2,1>(mtState::template getId<mtState::_fea>(i),mtState::template getId<mtState::_fea>(i)+2) = initCov.block<2,1>(1,0);
    cov_.template block<2,2>(mtState::template getId<mtState::_fea>(i),mtState::template getId<mtState::_fea>(i)) = initCov.block<2,2>(1,1);
  }

  /** \brief Get the median distance parameter values of the state features for each camera.
   *
   *  \note The distance parameter type depends on the set \ref DepthType.
   *  @param initDistanceParameter     - Depth parameter value which is set, if no median distance parameter could be
   *                                  computed from the state features for a specific camera.
   *  @param medianDistanceParameters  - Array, containing the median distance parameter values for each camera.
   *  @param maxUncertaintyToDistanceRatio  - Maximal uncertainty where feature gets considered
   * */
  void getMedianDepthParameters(double initDistanceParameter, std::array<double,nCam>* medianDistanceParameters, const float maxUncertaintyToDistanceRatio) {
    // Fill array with initialization value.
    // The initialization value is set, if no median distance value can be computed for a given camera frame.
    medianDistanceParameters->fill(initDistanceParameter);
    // Collect the distance values of the features for each camera frame.
    std::vector<double> distanceParameterCollection[nCam];
    for (unsigned int i = 0; i < nMax; i++) {
      if (fsm_.isValid_[i]) {
        for(int camID = 0;camID<nCam;camID++){
          transformFeatureOutputCT_.setFeatureID(i);
          transformFeatureOutputCT_.setOutputCameraID(camID);
          transformFeatureOutputCT_.transformState(state_, featureOutput_);
          if(featureOutput_.c().isInFront()){
            transformFeatureOutputCT_.transformCovMat(state_, cov_, featureOutputCov_);
            const double uncertainty = std::fabs(sqrt(featureOutputCov_(2,2))*featureOutput_.d().getDistanceDerivative());
            const double depth = featureOutput_.d().getDistance();
            if(uncertainty/depth < maxUncertaintyToDistanceRatio){
              distanceParameterCollection[camID].push_back(featureOutput_.d().p_);
            }
          }
        }
      }
    }
    // Compute and store the median distance parameter.
    int size;
    for (unsigned int i = 0; i < nCam; i++) {
      size = distanceParameterCollection[i].size();
      if(size > 3) { // Require a minimum of three features
        std::nth_element(distanceParameterCollection[i].begin(), distanceParameterCollection[i].begin() + size / 2, distanceParameterCollection[i].end());
        (*medianDistanceParameters)[i] = distanceParameterCollection[i][size/2];
      }
    }
  }
};

}


#endif /* ROVIO_FILTERSTATES_HPP_ */
