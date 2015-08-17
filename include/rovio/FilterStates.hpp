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

#include "kindr/rotations/RotationEigen.hpp"
#include <Eigen/Dense>
#include "lightweight_filtering/FilterState.hpp"
#include <map>
#include <unordered_set>

#include "FeatureLocationOutputCF.hpp"
#include "rovio/commonVision.hpp"

namespace rot = kindr::rotations::eigen_impl;

namespace rovio {
/** \brief Class , allowing the computation of some depth parameterization values.
 */
class DepthMap{
 public:

  /** \brief Specifies the depth type of the DepthMap.
   */
  enum DepthType{
    REGULAR,    /**<Regular Depth p = d*/
    INVERSE,    /**<Inverse Depth p = 1/d*/
    LOG,        /**<Logarithmic Depth p = ln(d)*/
    HYPERBOLIC  /**<Hyperbolic Depth p = asinh(d)*/
  } type_;

  /** \brief Constructor. Create a DepthMap object with a desired \ref DepthType.
   *
   *  @param type - enum \ref DepthType.
   */
  DepthMap(const DepthType& type = REGULAR){
    setType(type);
  }

  /** \brief Set the \ref DepthType type_ using the enum \ref DepthType.
   *
   *  @param type - Enum \ref DepthType.
   */
  void setType(const DepthType& type){
    type_ = type;
  }

  /** \brief Get the set \ref DepthType.
   *
   *  @return the set \ref DepthType.
   */
  DepthType getType() {
    return type_;
  }

  /** \brief Set the \ref DepthType type_ using the integer value of the enum \ref DepthType.
   *
   *  @param type - Integer value of the enum \ref DepthType.
   */
  void setType(const int& type){
    switch(type){
      case 0:
        type_ = REGULAR;
        break;
      case 1:
        type_ = INVERSE;
        break;
      case 2:
        type_ = LOG;
        break;
      case 3:
        type_ = HYPERBOLIC;
        break;
      default:
        std::cout << "Invalid type for depth parameterization: " << type << std::endl;
        type_ = REGULAR;
        break;
    }
  }

  /** \brief Computes some depth parameterization values, based on the set \ref DepthType type_.
   *
   *  @param p     - Parameter value. Has to match the set \ref DepthType type_.
   *                 If type_ = REGULAR then p must correspond to the depth value d.
   *                 If type_ = INVERSE then p must correspond to the inverse depth value p=1/d.
   *                 If type_ = LOG then p must correspond to the inverse depth value p=ln(d).
   *                 If type_ = HYPERBOLIC then p must correspond to the hyperbolic depth value p=asinh(d).
   *  @param d     - Depth value.
   *  @param d_p   - d derived w.r.t. p
   *  @param p_d   - p derived w.r.t. d
   *  @param p_d_p - p_d derived w.r.t. p
   */
  void map(const double& p, double& d, double& d_p, double& p_d, double& p_d_p) const{
    switch(type_){
      case REGULAR:
        mapRegular(p,d,d_p,p_d,p_d_p);
        break;
      case INVERSE:
        mapInverse(p,d,d_p,p_d,p_d_p);
        break;
      case LOG:
        mapLog(p,d,d_p,p_d,p_d_p);
        break;
      case HYPERBOLIC:
        mapHyperbolic(p,d,d_p,p_d,p_d_p);
        break;
      default:
        mapRegular(p,d,d_p,p_d,p_d_p);
        break;
    }
  }

  /** \brief Converts a value of a specific \ref DepthType into a value of another \ref DepthType.
   *
   *  @param type_p     - Input \ref DepthType, belonging to p.
   *  @param p          - Value of the input \ref DepthType.
   *  @param type_dep   - Output \ref DepthType, belonging to dep.
   *  @param dep        - Value of the output \ref DepthType.
   */
  static void convertDepthType(const DepthType& type_p, const double& p, const DepthType& type_dep, double& dep) {
    dep = p;
    // Source type is REGULAR.
    if (type_p == REGULAR) {
      if (type_dep == INVERSE) {
        dep = 1.0 / p;
      }
      else if (type_dep == LOG) {
        dep = std::log(p);
      }
      else if (type_dep == HYPERBOLIC) {
        dep = std::asinh(p);
      }
    }
    // Source type is INVERSE.
    else if (type_p == INVERSE) {
      if (type_dep == REGULAR) {
        dep = 1.0 / p;
      }
      else if (type_dep == LOG) {
        dep = std::log(1.0/p);
      }
      else if (type_dep == HYPERBOLIC) {
        dep = std::asinh(1.0/p);
      }
    }
    // Source type is LOG.
    else if (type_p == LOG) {
      if (type_dep == REGULAR) {
        dep = std::exp(p);
      }
      else if (type_dep == INVERSE) {
        dep = 1.0 / std::exp(p);
      }
      else if (type_dep == HYPERBOLIC) {
        dep = std::asinh(std::exp(p));
      }
    }
    // Source type is HYPERBOLIC.
    else if (type_p == HYPERBOLIC) {
      if (type_dep == REGULAR) {
        dep = std::sinh(p);
      }
      else if (type_dep == INVERSE) {
        dep = 1.0 / std::sinh(p);
      }
      else if (type_dep == LOG) {
        dep = std::log(std::sinh(p));
      }
    };
  }

  /** \brief Computes some depth parameterization values, given a depth value p = d.
   *
   *  @param p     - Input Parameter value: In this case p corresponds to the depth d.
   *  @param d     - Depth value.
   *  @param d_p   - d derived w.r.t. p
   *  @param p_d   - p derived w.r.t. d
   *  @param p_d_p - p_d derived w.r.t. p
   */
  void mapRegular(const double& p, double& d, double& d_p, double& p_d, double& p_d_p) const{
    d = p;
    d_p = 1.0;
    p_d = 1.0;
    p_d_p = 0.0;
  }

  /** \brief Computes some depth parameterization values, given an inverse depth value p = 1/d.
   *
   *  @param p     - Input Parameter value: In this case p corresponds to the inverse depth p = 1/d.
   *  @param d     - Depth value.
   *  @param d_p   - d derived w.r.t. p
   *  @param p_d   - p derived w.r.t. d
   *  @param p_d_p - p_d derived w.r.t. p
   */
  void mapInverse(const double& p, double& d, double& d_p, double& p_d, double& p_d_p) const{
    double p_temp = p;
    if(fabs(p_temp) < 1e-6){
      if(p_temp >= 0){
        p_temp = 1e-6;
      } else {
        p_temp = -1e-6;
      }
    }
    d = 1/p_temp;
    d_p = -d*d;
    p_d = -p_temp*p_temp;
    p_d_p = -2*p_temp;
  }

  /** \brief Computes some depth parameterization values, given a logarithmic depth value p = ln(d).
   *
   *  @param p     - Input Parameter value: In this case p corresponds to the logarithmic depth p = ln(d).
   *  @param d     - Depth value.
   *  @param d_p   - d derived w.r.t. p
   *  @param p_d   - p derived w.r.t. d
   *  @param p_d_p - p_d derived w.r.t. p
   */
  void mapLog(const double& p, double& d, double& d_p, double& p_d, double& p_d_p) const{
    d = std::exp(p);
    d_p = std::exp(p);
    p_d = 1/d;
    p_d_p = -1/pow(d,2)*d_p;
  }

  /** \brief Computes some depth parameterization values, given a hyperbolic depth value p = asinh(d).
   *
   *  @param p     - Input Parameter value: In this case p corresponds to the hyperbolic depth p = asinh(d).
   *  @param d     - Depth value.
   *  @param d_p   - d derived w.r.t. p
   *  @param p_d   - p derived w.r.t. d
   *  @param p_d_p - p_d derived w.r.t. p
   */
  void mapHyperbolic(const double& p, double& d, double& d_p, double& p_d, double& p_d_p) const{
    d = std::sinh(p);
    d_p = std::cosh(p);
    p_d = 1/std::sqrt(std::pow(d,2)+1); // p = asinh(d)
    p_d_p = -d/std::pow(std::pow(d,2)+1,1.5)*d_p;
  }
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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
      bearingMeas_[i].setIdentity();
      bearingCorners_[i][0].setZero();
      bearingCorners_[i][1].setZero();
      camID_[i] = 0;
    }
    doVECalibration_ = true;
    depthTypeInt_ = 1;
    depthMap_.setType(depthTypeInt_);
    activeFeature_ = 0;
    activeCameraCounter_ = 0;
    timeSinceLastInertialMotion_ = 0.0;
    timeSinceLastImageMotion_ = 0.0;
    for(unsigned int i=0;i<nCam;i++){
      qCM_[i].setIdentity();
      MrMC_[i].setZero();
    }
  };

  /** \brief Destructor
   */
  ~StateAuxiliary(){};

  V3D MwWMest_;  /**<Estimated rotational rate.*/
  V3D MwWMmeas_;  /**<Measured rotational rate.*/
  M3D wMeasCov_;  /**<Covariance of the measured rotational rate.*/
  Eigen::Matrix2d A_red_[nMax];  /**<Reduced Jacobian of the pixel intensities w.r.t. to pixel coordinates, needed for the multilevel patch alignment. \see rovio::MultilevelPatchFeature::A_ \see rovio::getLinearAlignEquationsReduced()*/
  Eigen::Vector2d b_red_[nMax];  /**<Reduced intensity errors, needed for the multilevel patch alignment. \see rovio::MultilevelPatchFeature::A_ \see rovio::getLinearAlignEquationsReduced()*/
  LWF::NormalVectorElement bearingMeas_[nMax];  /**<Intermediate variable for storing the measured bearing vectors.*/
  int camID_[nMax];  /**<%Camera ID*/
  BearingCorners bearingCorners_[nMax];
  QPD qCM_[nCam];  /**<Quaternion Array: IMU coordinates to camera coordinates.*/
  V3D MrMC_[nCam];  /**<Position Vector Array: Vectors pointing from IMU to the camera frame, expressed in the IMU frame.*/
  bool doVECalibration_;  /**<Do Camera-IMU extrinsic parameter calibration?*/
  DepthMap depthMap_;
  int depthTypeInt_;  /**<Integer enum value of the chosen DepthMap::DepthType.*/
  int activeFeature_;  /**< Active Feature ID. ID of the currently updated feature. Needed in the image update procedure.*/
  int activeCameraCounter_;  /**<Counter for iterating through the cameras, used such that when updating a feature we always start with the camId where the feature is expressed in.*/
  double timeSinceLastInertialMotion_;  /**<Time since the IMU showed motion last.*/
  double timeSinceLastImageMotion_;  /**<Time since the Image showed motion last.*/
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** \brief Filter State.
 *
 *  @tparam nMax      - Maximal number of considered features in the filter state.
 *  @tparam nLevels   - Total number of pyramid levels considered.
 *  @tparam patchSize - Edge length of the patches (in pixel). Must be a multiple of 2!
 *  @tparam nCam      - Used total number of cameras.
 */
template<unsigned int nMax, int nLevels, int patchSize, int nCam>
class State: public LWF::State<
LWF::TH_multiple_elements<LWF::VectorElement<3>,4>,
LWF::QuaternionElement,
LWF::ArrayElement<LWF::VectorElement<3>,nCam>,
LWF::ArrayElement<LWF::QuaternionElement,nCam>,
LWF::ArrayElement<LWF::ScalarElement,nMax>,
LWF::ArrayElement<LWF::NormalVectorElement,nMax>,
StateAuxiliary<nMax,nLevels,patchSize,nCam>>{
 public:
  typedef LWF::State<
      LWF::TH_multiple_elements<LWF::VectorElement<3>,4>,
      LWF::QuaternionElement,
      LWF::ArrayElement<LWF::VectorElement<3>,nCam>,
      LWF::ArrayElement<LWF::QuaternionElement,nCam>,
      LWF::ArrayElement<LWF::ScalarElement,nMax>,
      LWF::ArrayElement<LWF::NormalVectorElement,nMax>,
      StateAuxiliary<nMax,nLevels,patchSize,nCam>> Base;  /**<State definition.*/
  using Base::D_;
  using Base::E_;
  static constexpr unsigned int nMax_ = nMax;
  static constexpr unsigned int nLevels_ = nLevels;
  static constexpr unsigned int patchSize_ = patchSize;
  static constexpr unsigned int nCam_ = nCam;   /**<Total number of cameras.*/
  static constexpr unsigned int _pos = 0;       /**<Idx. Position Vector WrWM: Pointing from the World-Frame to the IMU-Frame, expressed in World-Coordinates.*/
  static constexpr unsigned int _vel = _pos+1;  /**<Idx. Velocity Vector MvM: Absolute velocity of the of the IMU-Frame, expressed in IMU-Coordinates.*/
  static constexpr unsigned int _acb = _vel+1;  /**<Idx. Additive bias on accelerometer.*/
  static constexpr unsigned int _gyb = _acb+1;  /**<Idx. Additive bias on gyroscope.*/
  static constexpr unsigned int _att = _gyb+1;  /**<Idx. Quaternion qWM: IMU coordinates to World coordinates.*/
  static constexpr unsigned int _vep = _att+1;  /**<Idx. Position Vector MrMC: Pointing from the IMU-Frame to the Camera-Frame, expressed in IMU-Coordinates.*/
  static constexpr unsigned int _vea = _vep+1;  /**<Idx. Quaternion qCM: IMU-Coordinates to Camera-Coordinates.*/
  static constexpr unsigned int _dep = _vea+1;  /**<Idx. Depth Parameter*/
  static constexpr unsigned int _nor = _dep+1;  /**<Idx. Bearing Vectors expressed in the Camera frame*/
  static constexpr unsigned int _aux = _nor+1;  /**<Idx. Auxiliary state.*/

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
    this->template getName<_dep>() = "dep";
    this->template getName<_nor>() = "nor";
    this->template getName<_aux>() = "auxiliary";
  }

  /** \brief Destructor
   */
  ~State(){};

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
  /** \brief Get/Set the bearing vector (NormalVectorElement) belonging to a specific feature i.
   *
   *  @param i - Feature Index
   *  @return a reference to the bearing vector (NormalVectorElement) of feature i.
   */
  inline LWF::NormalVectorElement& CfP(const int i = 0) {
    return this->template get<_nor>(i);
  }
  inline const LWF::NormalVectorElement& CfP(const int i = 0) const{
    return this->template get<_nor>(i);
  }
  //@}

  //@{
  /** \brief Get/Set the quaternion qCM, expressing the IMU-Frame in Camera-Coordinates (IMU Coordinates->%Camera Coordinates).
   *
   *  @param camID - %Camera ID
   *  @return a reference to the quaternion qCM (IMU Coordinates->%Camera Coordinates).
   */
  inline QPD& qCM(const int camID = 0){
    if(this->template get<_aux>().doVECalibration_){
          return this->template get<_vea>(camID);
        } else {
          return this->template get<_aux>().qCM_[camID];
        }
  }
  inline const QPD& qCM(const int camID = 0) const{
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
    if(this->template get<_aux>().doVECalibration_){
      return this->template get<_vep>(camID);
    } else {
      return this->template get<_aux>().MrMC_[camID];
    }
  }
  inline const V3D& MrMC(const int camID = 0) const{
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
    return qCM(camID)*this->template get<_att>().inverted();
  }
  //@}

  //@{
  /** \brief Get/Set the depth parameter of a specific feature i.
   *
   *  \note The depth parameter can be either defined as regular depth, inverse depth, logarithmic depth or hyperbolic depth.
   *        The kind of the depth encoding depends on the defined DepthMap.
   *
   *  @param i - Feature Index
   *  @return a reference to depth parameter of the feature.
   */
  inline double& dep(const int i){
    return this->template get<_dep>(i);
  }
  inline const double& dep(const int i) const{
    return this->template get<_dep>(i);
  }
  //@}

  //@{
  /** \brief Get the depth value of a specific feature.
   *
   *  @param i - Feature Index
   *  @return the depth value d of the feature.
   */
  double get_depth(const int i) const{
    double d, d_p, p_d, p_d_p;
    this->template get<_aux>().depthMap_.map(this->template get<_dep>(i),d,d_p,p_d,p_d_p);
    return d;
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
  ~PredictionMeas(){};
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** \brief Class, holding the prediction noise for the state members.
 *
 *  \see State
 *  @tparam STATE - Filter State
 *  @todo complete
 */
template<typename STATE>
class PredictionNoise: public LWF::State<LWF::TH_multiple_elements<LWF::VectorElement<3>,5>,
LWF::ArrayElement<LWF::VectorElement<3>,STATE::nCam_>,
LWF::ArrayElement<LWF::VectorElement<3>,STATE::nCam_>,
LWF::ArrayElement<LWF::ScalarElement,STATE::nMax_>,
LWF::ArrayElement<LWF::VectorElement<2>,STATE::nMax_>>{
 public:
  using LWF::State<LWF::TH_multiple_elements<LWF::VectorElement<3>,5>,
      LWF::ArrayElement<LWF::VectorElement<3>,STATE::nCam_>,
      LWF::ArrayElement<LWF::VectorElement<3>,STATE::nCam_>,
      LWF::ArrayElement<LWF::ScalarElement,STATE::nMax_>,
      LWF::ArrayElement<LWF::VectorElement<2>,STATE::nMax_>>::E_;
  static constexpr unsigned int _pos = 0;       /**<Idx. Position Vector WrWM: Pointing from the World-Frame to the IMU-Frame, expressed in World-Coordinates.*/
  static constexpr unsigned int _vel = _pos+1;  /**<Idx. Velocity Vector MvM: Absolute velocity of the IMU-Frame, expressed in IMU-Coordinates.*/
  static constexpr unsigned int _acb = _vel+1;  /**<Idx. Additive bias on accelerometer.*/
  static constexpr unsigned int _gyb = _acb+1;  /**<Idx. Additive bias on gyroscope.*/
  static constexpr unsigned int _att = _gyb+1;  /**<Idx. Quaternion qWM: IMU coordinates to World coordinates.*/
  static constexpr unsigned int _vep = _att+1;  /**<Idx. Position Vector MrMC: Pointing from the IMU-Frame to the Camera-Frame, expressed in IMU-Coordinates.*/
  static constexpr unsigned int _vea = _vep+1;  /**<Idx. Quaternion qCM: IMU-Coordinates to Camera-Coordinates.*/
  static constexpr unsigned int _dep = _vea+1;  /**<Idx. Depth Parameters @todo complete*/
  static constexpr unsigned int _nor = _dep+1;  /**<Idx. Bearing Vectors expressed in Camera-Coordinates.*/

  /** \brief Constructor
   */
  PredictionNoise(){
    static_assert(_nor+1==E_,"Error with indices");
    this->template getName<_pos>() = "pos";
    this->template getName<_vel>() = "vel";
    this->template getName<_acb>() = "acb";
    this->template getName<_gyb>() = "gyb";
    this->template getName<_att>() = "att";
    this->template getName<_vep>() = "vep";
    this->template getName<_vea>() = "vea";
    this->template getName<_dep>() = "dep";
    this->template getName<_nor>() = "nor";
  }

  /** \brief Destructor
   */
  ~PredictionNoise(){};
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** \brief Class defining the overall filter state (state, prediction measurement, noise).
 *
 *  @tparam nMax      - Maximal number of considered features in the filter state.
 *  @tparam nLevels   - Total number of pyramid levels considered.
 *  @tparam patchSize - Edge length of the patches (in pixel). Must be a multiple of 2!
 *  @tparam nCam      - Used total number of cameras.
 */
template<unsigned int nMax, int nLevels, int patchSize,int nCam>
class FilterState: public LWF::FilterState<State<nMax,nLevels,patchSize,nCam>,PredictionMeas,PredictionNoise<State<nMax,nLevels,patchSize,nCam>>,0,true>{
 public:
  typedef LWF::FilterState<State<nMax,nLevels,patchSize,nCam>,PredictionMeas,PredictionNoise<State<nMax,nLevels,patchSize,nCam>>,0,true> Base;
  typedef typename Base::mtState mtState;  /**<Local Filter %State Type. \see LWF::FilterState*/
  using Base::state_;  /**<Filter State. \see LWF::FilterState*/
  using Base::cov_;  /**<Filter State Covariance Matrix. \see LWF::FilterState*/
  using Base::usePredictionMerge_;  /**<Whether multiple subsequent prediction steps should be merged into one.*/
  MultilevelPatchSet<nLevels,patchSize,nMax> mlps_;
  FeatureLocationOutputCF<mtState> featureLocationOutputCF_;
  FeatureLocationOutput featureLocationOutput_;
  typename FeatureLocationOutputCF<mtState>::mtOutputCovMat featureLocationOutputCov_;
  cv::Mat img_[nCam];     /**<Mainly used for drawing.*/
  cv::Mat patchDrawing_;  /**<Mainly used for drawing.*/
  double imgTime_;        /**<Time of the last image, which was processed.*/
  int imageCounter_;      /**<Total number of images, used so far for updates. Same as total number of update steps.*/
  ImagePyramid<nLevels> prevPyr_[nCam]; /**<Previous image pyramid.*/
  rot::RotationQuaternionPD groundtruth_qCJ_; /**<Groundtruth attitude measurement.*/
  rot::RotationQuaternionPD groundtruth_qJI_; /**<Transformtion to groundtruth inertial frame (rotation).*/
  rot::RotationQuaternionPD groundtruth_qCB_; /**<Transformtion to groundtruth body frame (rotation).*/
  Eigen::Vector3d groundtruth_JrJC_; /**<Groundtruth position measurement.*/
  Eigen::Vector3d groundtruth_IrIJ_; /**<Transformtion to groundtruth inertial frame (translation).*/
  Eigen::Vector3d groundtruth_BrBC_; /**<Transformtion to groundtruth body frame (translation).*/
  bool plotGroundtruth_; /**<Should the groundtruth be plotted.*/

  /** \brief Constructor
   */
  FilterState(){
    usePredictionMerge_ = true;
    imgTime_ = 0.0;
    imageCounter_ = 0;
    groundtruth_qCJ_.setIdentity();
    groundtruth_JrJC_.setZero();
    groundtruth_qJI_.setIdentity();
    groundtruth_IrIJ_.setZero();
    groundtruth_qCB_.setIdentity();
    groundtruth_BrBC_.setZero();
    plotGroundtruth_ = true;
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
      state_.qWM().setFromVectors(unitZ,fMeasInit);
    } else {
      state_.qWM().setIdentity();
    }
  }

  /** \brief Initializes a specific feature in the filter state.
   *
   *  Note that a bearing vector is described with only 2 parameters.
   *  @param i       - Feature index.
   *  @param n       - Bearing vector of the feature (unit length not necessary).
   *  @param d       - Depth value.
   *  @param initCov - Initialization 3x3 Covariance-Matrix.
   *
   *                   [ Cov(d,d)      Cov(d,nor_1)      Cov(d,nor_2)
   *                     Cov(nor_1,d)  Cov(nor_1,nor_1)  Cov(nor_1,nor_2)
   *                     Cov(nor_2,d)  Cov(nor_2,nor_1)  Cov(nor_2,nor_2) ]
   */
  void initializeFeatureState(unsigned int i, V3D n, double d,const Eigen::Matrix<double,3,3>& initCov){
    state_.dep(i) = d;
    state_.CfP(i).setFromVector(n);
    cov_.template block<mtState::D_,1>(0,mtState::template getId<mtState::_dep>(i)).setZero();
    cov_.template block<1,mtState::D_>(mtState::template getId<mtState::_dep>(i),0).setZero();
    cov_.template block<mtState::D_,2>(0,mtState::template getId<mtState::_nor>(i)).setZero();
    cov_.template block<2,mtState::D_>(mtState::template getId<mtState::_nor>(i),0).setZero();
    cov_.template block<1,1>(mtState::template getId<mtState::_dep>(i),mtState::template getId<mtState::_dep>(i)) = initCov.block<1,1>(0,0);
    cov_.template block<1,2>(mtState::template getId<mtState::_dep>(i),mtState::template getId<mtState::_nor>(i)) = initCov.block<1,2>(0,1);
    cov_.template block<2,1>(mtState::template getId<mtState::_nor>(i),mtState::template getId<mtState::_dep>(i)) = initCov.block<2,1>(1,0);
    cov_.template block<2,2>(mtState::template getId<mtState::_nor>(i),mtState::template getId<mtState::_nor>(i)) = initCov.block<2,2>(1,1);
  }

  /** \brief Removes a feature from the state.
   *
   *  @param i       - Feature index.
   */
  void removeFeature(unsigned int i){
    state_.dep(i) = 1.0;
    state_.CfP(i).setIdentity();
    cov_.template block<mtState::D_,1>(0,mtState::template getId<mtState::_dep>(i)).setZero();
    cov_.template block<1,mtState::D_>(mtState::template getId<mtState::_dep>(i),0).setZero();
    cov_.template block<mtState::D_,2>(0,mtState::template getId<mtState::_nor>(i)).setZero();
    cov_.template block<2,mtState::D_>(mtState::template getId<mtState::_nor>(i),0).setZero();
    cov_.template block<1,1>(mtState::template getId<mtState::_dep>(i),mtState::template getId<mtState::_dep>(i)).setIdentity();
    cov_.template block<2,2>(mtState::template getId<mtState::_nor>(i),mtState::template getId<mtState::_nor>(i)).setIdentity();
  }

  /** \brief Get the median depth parameter values of the state features for each camera.
   *
   *  \note The depth parameter type depends on the set \ref DepthType.
   *  @param initDepthParameter     - Depth parameter value which is set, if no median depth parameter could be
   *                                  computed from the state features for a specific camera.
   *  @param medianDepthParameters  - Array, containing the median depth parameter values for each camera.
   * */
  void getMedianDepthParameters(double initDepthParameter, std::array<double,nCam>* medianDepthParameters, const float maxUncertaintyToDepthRatio) {
    // Fill array with initialization value.
    // The initialization value is set, if no median depth value can be computed for a given camera frame.
    medianDepthParameters->fill(initDepthParameter);
    // Collect the depth values of the features for each camera frame.
    std::vector<double> depthValueCollection[nCam];
    unsigned int camID, activeCamCounter, activeCamID;
    double depth, sigmaDepth;
    for (unsigned int i = 0; i < nMax; i++) {
      if (mlps_.isValid_[i]) {
        activeCamCounter = 0;
        camID = mlps_.features_[i].camID_;
        featureLocationOutputCF_.setFeatureID(i);
        while (activeCamCounter != nCam) {
          activeCamID = (activeCamCounter + camID)%nCam;
          featureLocationOutputCF_.setOutputCameraID(activeCamID);
          featureLocationOutputCF_.transformCovMat(state_, cov_, featureLocationOutputCov_);
          sigmaDepth = std::sqrt(featureLocationOutputCov_(FeatureLocationOutput::_dep,FeatureLocationOutput::_dep));
          featureLocationOutputCF_.transformState(state_, featureLocationOutput_);
          depth = featureLocationOutput_.dep();
          if (featureLocationOutput_.CfP().getVec()(2) <= 0.0) {   // Abort if the bearing vector in the current frame has a negative z-component. Todo: Change this. Should check if vector intersects with image plane.
            activeCamCounter++;
            continue;
          }
          // Collect depth data if uncertainty-depth ratio small enough.
          if (sigmaDepth/depth <= maxUncertaintyToDepthRatio) {
            depthValueCollection[activeCamID].push_back(depth);
          }
          activeCamCounter++;
        }
      }
    }
    // Compute and store the median depth parameter.
    int size;
    for (unsigned int i = 0; i < nCam; i++) {
      size = depthValueCollection[i].size();
      if(size > 2) { // Require a minimum of three features
        std::nth_element(depthValueCollection[i].begin(), depthValueCollection[i].begin() + size / 2, depthValueCollection[i].end());
        DepthMap::convertDepthType(DepthMap::REGULAR, depthValueCollection[i][size/2],
                                   state_.aux().depthMap_.getType(), (*medianDepthParameters)[i]);
      }
    }
  }
};

}


#endif /* ROVIO_FILTERSTATES_HPP_ */
