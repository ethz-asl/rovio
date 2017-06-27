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

#ifndef ROVIO_ROVIONODE_HPP_
#define ROVIO_ROVIONODE_HPP_

#include <memory>
#include <mutex>
#include <queue>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

#include <rovio/SrvResetToPose.h>
#include "rovio/RovioFilter.hpp"
#include "rovio/CoordinateTransform/RovioOutput.hpp"
#include "rovio/CoordinateTransform/FeatureOutput.hpp"
#include "rovio/CoordinateTransform/FeatureOutputReadable.hpp"
#include "rovio/CoordinateTransform/YprOutput.hpp"
#include "rovio/CoordinateTransform/LandmarkOutput.hpp"

namespace rovio {

/** \brief Class, defining the Rovio Node
 *
 *  @tparam FILTER  - \ref rovio::RovioFilter
 */
template<typename FILTER>
class RovioNode{
 public:
  // Filter Stuff
  typedef FILTER mtFilter;
  std::shared_ptr<mtFilter> mpFilter_;
  typedef typename mtFilter::mtFilterState mtFilterState;
  typedef typename mtFilterState::mtState mtState;
  typedef typename mtFilter::mtPrediction::mtMeas mtPredictionMeas;
  mtPredictionMeas predictionMeas_;
  typedef typename std::tuple_element<0,typename mtFilter::mtUpdates>::type mtImgUpdate;
  typedef typename mtImgUpdate::mtMeas mtImgMeas;
  mtImgMeas imgUpdateMeas_;
  mtImgUpdate* mpImgUpdate_;
  typedef typename std::tuple_element<1,typename mtFilter::mtUpdates>::type mtPoseUpdate;
  typedef typename mtPoseUpdate::mtMeas mtPoseMeas;
  mtPoseMeas poseUpdateMeas_;
  mtPoseUpdate* mpPoseUpdate_;
  typedef typename std::tuple_element<2,typename mtFilter::mtUpdates>::type mtVelocityUpdate;
  typedef typename mtVelocityUpdate::mtMeas mtVelocityMeas;
  mtVelocityMeas velocityUpdateMeas_;

  struct FilterInitializationState {
    FilterInitializationState()
        : WrWM_(V3D::Zero()),
          state_(State::WaitForInitUsingAccel) {}

    enum class State {
      // Initialize the filter using accelerometer measurement on the next
      // opportunity.
      WaitForInitUsingAccel,
      // Initialize the filter using an external pose on the next opportunity.
      WaitForInitExternalPose,
      // The filter is initialized.
      Initialized
    } state_;

    // Buffer to hold the initial pose that should be set during initialization
    // with the state WaitForInitExternalPose.
    V3D WrWM_;
    QPD qMW_;

    explicit operator bool() const {
      return isInitialized();
    }

    bool isInitialized() const {
      return (state_ == State::Initialized);
    }
  };
  FilterInitializationState init_state_;

  bool forceOdometryPublishing_;
  bool forcePoseWithCovariancePublishing_;
  bool forceTransformPublishing_;
  bool forceExtrinsicsPublishing_;
  bool forceImuBiasPublishing_;
  bool forcePclPublishing_;
  bool forceMarkersPublishing_;
  bool forcePatchPublishing_;
  bool gotFirstMessages_;
  std::mutex m_filter_;

  // Nodes, Subscriber, Publishers
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber subImu_;
  ros::Subscriber subImg0_;
  ros::Subscriber subImg1_;
  ros::Subscriber subGroundtruth_;
  ros::Subscriber subGroundtruthOdometry_;
  ros::Subscriber subVelocity_;
  ros::ServiceServer srvResetFilter_;
  ros::ServiceServer srvResetToPoseFilter_;
  ros::Publisher pubOdometry_;
  ros::Publisher pubTransform_;
  ros::Publisher pubPoseWithCovStamped_;
  ros::Publisher pub_T_J_W_transform;
  tf::TransformBroadcaster tb_;
  ros::Publisher pubPcl_;            /**<Publisher: Ros point cloud, visualizing the landmarks.*/
  ros::Publisher pubPatch_;            /**<Publisher: Patch data.*/
  ros::Publisher pubMarkers_;          /**<Publisher: Ros line marker, indicating the depth uncertainty of a landmark.*/
  ros::Publisher pubExtrinsics_[mtState::nCam_];
  ros::Publisher pubImuBias_;

  // Ros Messages
  geometry_msgs::TransformStamped transformMsg_;
  geometry_msgs::TransformStamped T_J_W_Msg_;
  nav_msgs::Odometry odometryMsg_;
  geometry_msgs::PoseWithCovarianceStamped estimatedPoseWithCovarianceStampedMsg_;
  geometry_msgs::PoseWithCovarianceStamped extrinsicsMsg_[mtState::nCam_];
  sensor_msgs::PointCloud2 pclMsg_;
  sensor_msgs::PointCloud2 patchMsg_;
  visualization_msgs::Marker markerMsg_;
  sensor_msgs::Imu imuBiasMsg_;
  int msgSeq_;

  // Rovio outputs and coordinate transformations
  typedef StandardOutput mtOutput;
  mtOutput cameraOutput_;
  MXD cameraOutputCov_;
  mtOutput imuOutput_;
  MXD imuOutputCov_;
  CameraOutputCT<mtState> cameraOutputCT_;
  ImuOutputCT<mtState> imuOutputCT_;
  rovio::TransformFeatureOutputCT<mtState> transformFeatureOutputCT_;
  rovio::LandmarkOutputImuCT<mtState> landmarkOutputImuCT_;
  rovio::FeatureOutput featureOutput_;
  rovio::LandmarkOutput landmarkOutput_;
  MXD featureOutputCov_;
  MXD landmarkOutputCov_;
  rovio::FeatureOutputReadableCT featureOutputReadableCT_;
  rovio::FeatureOutputReadable featureOutputReadable_;
  MXD featureOutputReadableCov_;

  // ROS names for output tf frames.
  std::string map_frame_;
  std::string world_frame_;
  std::string camera_frame_;
  std::string imu_frame_;

  /** \brief Constructor
   */
  RovioNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private, std::shared_ptr<mtFilter> mpFilter)
      : nh_(nh), nh_private_(nh_private), mpFilter_(mpFilter), transformFeatureOutputCT_(&mpFilter->multiCamera_), landmarkOutputImuCT_(&mpFilter->multiCamera_),
        cameraOutputCov_((int)(mtOutput::D_),(int)(mtOutput::D_)), featureOutputCov_((int)(FeatureOutput::D_),(int)(FeatureOutput::D_)), landmarkOutputCov_(3,3),
        featureOutputReadableCov_((int)(FeatureOutputReadable::D_),(int)(FeatureOutputReadable::D_)){
    #ifndef NDEBUG
      ROS_WARN("====================== Debug Mode ======================");
    #endif
    mpImgUpdate_ = &std::get<0>(mpFilter_->mUpdates_);
    mpPoseUpdate_ = &std::get<1>(mpFilter_->mUpdates_);
    forceOdometryPublishing_ = false;
    forcePoseWithCovariancePublishing_ = false;
    forceTransformPublishing_ = false;
    forceExtrinsicsPublishing_ = false;
    forceImuBiasPublishing_ = false;
    forcePclPublishing_ = false;
    forceMarkersPublishing_ = false;
    forcePatchPublishing_ = false;
    gotFirstMessages_ = false;

    // Subscribe topics
    subImu_ = nh_.subscribe("imu0", 1000, &RovioNode::imuCallback,this);
    subImg0_ = nh_.subscribe("cam0/image_raw", 1000, &RovioNode::imgCallback0,this);
    subImg1_ = nh_.subscribe("cam1/image_raw", 1000, &RovioNode::imgCallback1,this);
    subGroundtruth_ = nh_.subscribe("pose", 1000, &RovioNode::groundtruthCallback,this);
    subGroundtruthOdometry_ = nh_.subscribe("odometry", 1000, &RovioNode::groundtruthOdometryCallback, this);
    subVelocity_ = nh_.subscribe("abss/twist", 1000, &RovioNode::velocityCallback,this);

    // Initialize ROS service servers.
    srvResetFilter_ = nh_.advertiseService("rovio/reset", &RovioNode::resetServiceCallback, this);
    srvResetToPoseFilter_ = nh_.advertiseService("rovio/reset_to_pose", &RovioNode::resetToPoseServiceCallback, this);

    // Advertise topics
    pubTransform_ = nh_.advertise<geometry_msgs::TransformStamped>("rovio/transform", 1);
    pubOdometry_ = nh_.advertise<nav_msgs::Odometry>("rovio/odometry", 1);
    pubPoseWithCovStamped_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("rovio/pose_with_covariance_stamped", 1);
    pubPcl_ = nh_.advertise<sensor_msgs::PointCloud2>("rovio/pcl", 1);
    pubPatch_ = nh_.advertise<sensor_msgs::PointCloud2>("rovio/patch", 1);
    pubMarkers_ = nh_.advertise<visualization_msgs::Marker>("rovio/markers", 1 );

    pub_T_J_W_transform = nh_.advertise<geometry_msgs::TransformStamped>("rovio/T_G_W", 1);
    for(int camID=0;camID<mtState::nCam_;camID++){
      pubExtrinsics_[camID] = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("rovio/extrinsics" + std::to_string(camID), 1 );
    }
    pubImuBias_ = nh_.advertise<sensor_msgs::Imu>("rovio/imu_biases", 1 );

    // Handle coordinate frame naming
    map_frame_ = "/map";
    world_frame_ = "/world";
    camera_frame_ = "/camera";
    imu_frame_ = "/imu";
    nh_private_.param("map_frame", map_frame_, map_frame_);
    nh_private_.param("world_frame", world_frame_, world_frame_);
    nh_private_.param("camera_frame", camera_frame_, camera_frame_);
    nh_private_.param("imu_frame", imu_frame_, imu_frame_);

    // Initialize messages
    transformMsg_.header.frame_id = world_frame_;
    transformMsg_.child_frame_id = imu_frame_;

    T_J_W_Msg_.child_frame_id = world_frame_;
    T_J_W_Msg_.header.frame_id = map_frame_;

    odometryMsg_.header.frame_id = world_frame_;
    odometryMsg_.child_frame_id = imu_frame_;
    msgSeq_ = 1;
    for(int camID=0;camID<mtState::nCam_;camID++){
      extrinsicsMsg_[camID].header.frame_id = imu_frame_;
    }
    imuBiasMsg_.header.frame_id = world_frame_;
    imuBiasMsg_.orientation.x = 0;
    imuBiasMsg_.orientation.y = 0;
    imuBiasMsg_.orientation.z = 0;
    imuBiasMsg_.orientation.w = 1;
    for(int i=0;i<9;i++){
      imuBiasMsg_.orientation_covariance[i] = 0.0;
    }

    // PointCloud message.
    pclMsg_.header.frame_id = imu_frame_;
    pclMsg_.height = 1;               // Unordered point cloud.
    pclMsg_.width  = mtState::nMax_;  // Number of features/points.
    const int nFieldsPcl = 18;
    std::string namePcl[nFieldsPcl] = {"id","camId","rgb","status","x","y","z","b_x","b_y","b_z","d","c_00","c_01","c_02","c_11","c_12","c_22","c_d"};
    int sizePcl[nFieldsPcl] = {4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4};
    int countPcl[nFieldsPcl] = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
    int datatypePcl[nFieldsPcl] = {sensor_msgs::PointField::INT32,sensor_msgs::PointField::INT32,sensor_msgs::PointField::UINT32,sensor_msgs::PointField::UINT32,
        sensor_msgs::PointField::FLOAT32,sensor_msgs::PointField::FLOAT32,sensor_msgs::PointField::FLOAT32,
        sensor_msgs::PointField::FLOAT32,sensor_msgs::PointField::FLOAT32,sensor_msgs::PointField::FLOAT32,sensor_msgs::PointField::FLOAT32,
        sensor_msgs::PointField::FLOAT32,sensor_msgs::PointField::FLOAT32,sensor_msgs::PointField::FLOAT32,sensor_msgs::PointField::FLOAT32,sensor_msgs::PointField::FLOAT32,sensor_msgs::PointField::FLOAT32,sensor_msgs::PointField::FLOAT32};
    pclMsg_.fields.resize(nFieldsPcl);
    int byteCounter = 0;
    for(int i=0;i<nFieldsPcl;i++){
      pclMsg_.fields[i].name     = namePcl[i];
      pclMsg_.fields[i].offset   = byteCounter;
      pclMsg_.fields[i].count    = countPcl[i];
      pclMsg_.fields[i].datatype = datatypePcl[i];
      byteCounter += sizePcl[i]*countPcl[i];
    }
    pclMsg_.point_step = byteCounter;
    pclMsg_.row_step = pclMsg_.point_step * pclMsg_.width;
    pclMsg_.data.resize(pclMsg_.row_step * pclMsg_.height);
    pclMsg_.is_dense = false;

    // PointCloud message.
    patchMsg_.header.frame_id = "";
    patchMsg_.height = 1;               // Unordered point cloud.
    patchMsg_.width  = mtState::nMax_;  // Number of features/points.
    const int nFieldsPatch = 5;
    std::string namePatch[nFieldsPatch] = {"id","patch","dx","dy","error"};
    int sizePatch[nFieldsPatch] = {4,4,4,4,4};
    int countPatch[nFieldsPatch] = {1,mtState::nLevels_*mtState::patchSize_*mtState::patchSize_,mtState::nLevels_*mtState::patchSize_*mtState::patchSize_,mtState::nLevels_*mtState::patchSize_*mtState::patchSize_,mtState::nLevels_*mtState::patchSize_*mtState::patchSize_};
    int datatypePatch[nFieldsPatch] = {sensor_msgs::PointField::INT32,sensor_msgs::PointField::FLOAT32,sensor_msgs::PointField::FLOAT32,sensor_msgs::PointField::FLOAT32,sensor_msgs::PointField::FLOAT32};
    patchMsg_.fields.resize(nFieldsPatch);
    byteCounter = 0;
    for(int i=0;i<nFieldsPatch;i++){
      patchMsg_.fields[i].name     = namePatch[i];
      patchMsg_.fields[i].offset   = byteCounter;
      patchMsg_.fields[i].count    = countPatch[i];
      patchMsg_.fields[i].datatype = datatypePatch[i];
      byteCounter += sizePatch[i]*countPatch[i];
    }
    patchMsg_.point_step = byteCounter;
    patchMsg_.row_step = patchMsg_.point_step * patchMsg_.width;
    patchMsg_.data.resize(patchMsg_.row_step * patchMsg_.height);
    patchMsg_.is_dense = false;

    // Marker message (vizualization of uncertainty)
    markerMsg_.header.frame_id = imu_frame_;
    markerMsg_.id = 0;
    markerMsg_.type = visualization_msgs::Marker::LINE_LIST;
    markerMsg_.action = visualization_msgs::Marker::ADD;
    markerMsg_.pose.position.x = 0;
    markerMsg_.pose.position.y = 0;
    markerMsg_.pose.position.z = 0;
    markerMsg_.pose.orientation.x = 0.0;
    markerMsg_.pose.orientation.y = 0.0;
    markerMsg_.pose.orientation.z = 0.0;
    markerMsg_.pose.orientation.w = 1.0;
    markerMsg_.scale.x = 0.04; // Line width.
    markerMsg_.color.a = 1.0;
    markerMsg_.color.r = 0.0;
    markerMsg_.color.g = 1.0;
    markerMsg_.color.b = 0.0;
  }

  /** \brief Destructor
   */
  virtual ~RovioNode(){}

  /** \brief Tests the functionality of the rovio node.
   *
   *  @todo debug with   doVECalibration = false and depthType = 0
   */
  void makeTest(){
    mtFilterState* mpTestFilterState = new mtFilterState();
    *mpTestFilterState = mpFilter_->init_;
    mpTestFilterState->setCamera(&mpFilter_->multiCamera_);
    mtState& testState = mpTestFilterState->state_;
    unsigned int s = 2;
    testState.setRandom(s);
    predictionMeas_.setRandom(s);
    imgUpdateMeas_.setRandom(s);

    LWF::NormalVectorElement tempNor;
    for(int i=0;i<mtState::nMax_;i++){
      testState.CfP(i).camID_ = 0;
      tempNor.setRandom(s);
      if(tempNor.getVec()(2) < 0){
        tempNor.boxPlus(Eigen::Vector2d(3.14,0),tempNor);
      }
      testState.CfP(i).set_nor(tempNor);
      testState.CfP(i).trackWarping_ = false;
      tempNor.setRandom(s);
      if(tempNor.getVec()(2) < 0){
        tempNor.boxPlus(Eigen::Vector2d(3.14,0),tempNor);
      }
      testState.aux().feaCoorMeas_[i].set_nor(tempNor,true);
      testState.aux().feaCoorMeas_[i].mpCamera_ = &mpFilter_->multiCamera_.cameras_[0];
      testState.aux().feaCoorMeas_[i].camID_ = 0;
    }
    testState.CfP(0).camID_ = mtState::nCam_-1;
    mpTestFilterState->fsm_.setAllCameraPointers();

    // Prediction
    std::cout << "Testing Prediction" << std::endl;
    mpFilter_->mPrediction_.testPredictionJacs(testState,predictionMeas_,1e-8,1e-6,0.1);

    // Update
    if(!mpImgUpdate_->useDirectMethod_){
      std::cout << "Testing Update (can sometimes exhibit large absolut errors due to the float precision)" << std::endl;
      for(int i=0;i<(std::min((int)mtState::nMax_,2));i++){
        testState.aux().activeFeature_ = i;
        testState.aux().activeCameraCounter_ = 0;
        mpImgUpdate_->testUpdateJacs(testState,imgUpdateMeas_,1e-4,1e-5);
        testState.aux().activeCameraCounter_ = mtState::nCam_-1;
        mpImgUpdate_->testUpdateJacs(testState,imgUpdateMeas_,1e-4,1e-5);
      }
    }

    // Testing CameraOutputCF and CameraOutputCF
    std::cout << "Testing cameraOutputCF" << std::endl;
    cameraOutputCT_.testTransformJac(testState,1e-8,1e-6);
    std::cout << "Testing imuOutputCF" << std::endl;
    imuOutputCT_.testTransformJac(testState,1e-8,1e-6);
    std::cout << "Testing attitudeToYprCF" << std::endl;
    rovio::AttitudeToYprCT attitudeToYprCF;
    attitudeToYprCF.testTransformJac(1e-8,1e-6);

    // Testing TransformFeatureOutputCT
    std::cout << "Testing transformFeatureOutputCT" << std::endl;
    transformFeatureOutputCT_.setFeatureID(0);
    if(mtState::nCam_>1){
      transformFeatureOutputCT_.setOutputCameraID(1);
      transformFeatureOutputCT_.testTransformJac(testState,1e-8,1e-5);
    }
    transformFeatureOutputCT_.setOutputCameraID(0);
    transformFeatureOutputCT_.testTransformJac(testState,1e-8,1e-5);

    // Testing LandmarkOutputImuCT
    std::cout << "Testing LandmarkOutputImuCT" << std::endl;
    landmarkOutputImuCT_.setFeatureID(0);
    landmarkOutputImuCT_.testTransformJac(testState,1e-8,1e-5);

    // Getting featureOutput for next tests
    transformFeatureOutputCT_.transformState(testState,featureOutput_);
    if(!featureOutput_.c().isInFront()){
      featureOutput_.c().set_nor(featureOutput_.c().get_nor().rotated(QPD(0.0,1.0,0.0,0.0)),false);
    }

    // Testing FeatureOutputReadableCT
    std::cout << "Testing FeatureOutputReadableCT" << std::endl;
    featureOutputReadableCT_.testTransformJac(featureOutput_,1e-8,1e-5);

    // Testing pixelOutputCT
    rovio::PixelOutputCT pixelOutputCT;
    std::cout << "Testing pixelOutputCT (can sometimes exhibit large absolut errors due to the float precision)" << std::endl;
    pixelOutputCT.testTransformJac(featureOutput_,1e-4,1.0); // Reduces accuracy due to float and strong camera distortion

    // Testing ZeroVelocityUpdate_
    std::cout << "Testing zero velocity update" << std::endl;
    mpImgUpdate_->zeroVelocityUpdate_.testJacs();

    // Testing PoseUpdate
    if(!mpPoseUpdate_->noFeedbackToRovio_){
      std::cout << "Testing pose update" << std::endl;
      mpPoseUpdate_->testUpdateJacs(1e-8,1e-5);
    }

    delete mpTestFilterState;
  }

  /** \brief Callback for IMU-Messages. Adds IMU measurements (as prediction measurements) to the filter.
   */
  void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg){
    std::lock_guard<std::mutex> lock(m_filter_);
    predictionMeas_.template get<mtPredictionMeas::_acc>() = Eigen::Vector3d(imu_msg->linear_acceleration.x,imu_msg->linear_acceleration.y,imu_msg->linear_acceleration.z);
    predictionMeas_.template get<mtPredictionMeas::_gyr>() = Eigen::Vector3d(imu_msg->angular_velocity.x,imu_msg->angular_velocity.y,imu_msg->angular_velocity.z);
    if(init_state_.isInitialized()){
      mpFilter_->addPredictionMeas(predictionMeas_,imu_msg->header.stamp.toSec());
      updateAndPublish();
    } else {
      switch(init_state_.state_) {
        case FilterInitializationState::State::WaitForInitExternalPose: {
          std::cout << "-- Filter: Initializing using external pose ..." << std::endl;
          mpFilter_->resetWithPose(init_state_.WrWM_, init_state_.qMW_, imu_msg->header.stamp.toSec());
          break;
        }
        case FilterInitializationState::State::WaitForInitUsingAccel: {
          std::cout << "-- Filter: Initializing using accel. measurement ..." << std::endl;
          mpFilter_->resetWithAccelerometer(predictionMeas_.template get<mtPredictionMeas::_acc>(),imu_msg->header.stamp.toSec());
          break;
        }
        default: {
          std::cout << "Unhandeld initialization type." << std::endl;
          abort();
          break;
        }
      }

      std::cout << std::setprecision(12);
      std::cout << "-- Filter: Initialized at t = " << imu_msg->header.stamp.toSec() << std::endl;
      init_state_.state_ = FilterInitializationState::State::Initialized;
    }
  }

  /** \brief Image callback for the camera with ID 0
   *
   * @param img - Image message.
   * @todo generalize
   */
  void imgCallback0(const sensor_msgs::ImageConstPtr & img){
    std::lock_guard<std::mutex> lock(m_filter_);
    imgCallback(img,0);
  }

  /** \brief Image callback for the camera with ID 1
   *
   * @param img - Image message.
   * @todo generalize
   */
  void imgCallback1(const sensor_msgs::ImageConstPtr & img) {
    std::lock_guard<std::mutex> lock(m_filter_);
    if(mtState::nCam_ > 1) imgCallback(img,1);
  }

  /** \brief Image callback. Adds images (as update measurements) to the filter.
   *
   *   @param img   - Image message.
   *   @param camID - Camera ID.
   */
  void imgCallback(const sensor_msgs::ImageConstPtr & img, const int camID = 0){
    // Get image from msg
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::TYPE_8UC1);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    cv::Mat cv_img;
    cv_ptr->image.copyTo(cv_img);
    if(init_state_.isInitialized() && !cv_img.empty()){
      double msgTime = img->header.stamp.toSec();
      if(msgTime != imgUpdateMeas_.template get<mtImgMeas::_aux>().imgTime_){
        for(int i=0;i<mtState::nCam_;i++){
          if(imgUpdateMeas_.template get<mtImgMeas::_aux>().isValidPyr_[i]){
            std::cout << "    \033[31mFailed Synchronization of Camera Frames, t = " << msgTime << "\033[0m" << std::endl;
          }
        }
        imgUpdateMeas_.template get<mtImgMeas::_aux>().reset(msgTime);
      }
      imgUpdateMeas_.template get<mtImgMeas::_aux>().pyr_[camID].computeFromImage(cv_img,true);
      imgUpdateMeas_.template get<mtImgMeas::_aux>().isValidPyr_[camID] = true;

      if(imgUpdateMeas_.template get<mtImgMeas::_aux>().areAllValid()){
        mpFilter_->template addUpdateMeas<0>(imgUpdateMeas_,msgTime);
        imgUpdateMeas_.template get<mtImgMeas::_aux>().reset(msgTime);
        updateAndPublish();
      }
    }
  }

  /** \brief Callback for external groundtruth as TransformStamped
   *
   *  @param transform - Groundtruth message.
   */
  void groundtruthCallback(const geometry_msgs::TransformStamped::ConstPtr& transform){
    std::lock_guard<std::mutex> lock(m_filter_);
    if(init_state_.isInitialized()){
      Eigen::Vector3d JrJV(transform->transform.translation.x,transform->transform.translation.y,transform->transform.translation.z);
      poseUpdateMeas_.pos() = JrJV;
      QPD qJV(transform->transform.rotation.w,transform->transform.rotation.x,transform->transform.rotation.y,transform->transform.rotation.z);
      poseUpdateMeas_.att() = qJV.inverted();
      mpFilter_->template addUpdateMeas<1>(poseUpdateMeas_,transform->header.stamp.toSec()+mpPoseUpdate_->timeOffset_);
      updateAndPublish();
    }
  }

  /** \brief Callback for external groundtruth as Odometry
   *
   * @param odometry - Groundtruth message.
   */
  void groundtruthOdometryCallback(const nav_msgs::Odometry::ConstPtr& odometry) {
    std::lock_guard<std::mutex> lock(m_filter_);
    if(init_state_.isInitialized()) {
      Eigen::Vector3d JrJV(odometry->pose.pose.position.x,odometry->pose.pose.position.y,odometry->pose.pose.position.z);
      poseUpdateMeas_.pos() = JrJV;

      QPD qJV(odometry->pose.pose.orientation.w,odometry->pose.pose.orientation.x,odometry->pose.pose.orientation.y,odometry->pose.pose.orientation.z);
      poseUpdateMeas_.att() = qJV.inverted();

      const Eigen::Matrix<double,6,6> measuredCov = Eigen::Map<const Eigen::Matrix<double,6,6,Eigen::RowMajor>>(odometry->pose.covariance.data());
      poseUpdateMeas_.measuredCov() = measuredCov;

      mpFilter_->template addUpdateMeas<1>(poseUpdateMeas_,odometry->header.stamp.toSec()+mpPoseUpdate_->timeOffset_);
      updateAndPublish();
    }
  }

  /** \brief Callback for external velocity measurements
   *
   *  @param transform - Groundtruth message.
   */
  void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& velocity){
    std::lock_guard<std::mutex> lock(m_filter_);
    if(init_state_.isInitialized()){
      Eigen::Vector3d AvM(velocity->twist.linear.x,velocity->twist.linear.y,velocity->twist.linear.z);
      velocityUpdateMeas_.vel() = AvM;
      mpFilter_->template addUpdateMeas<2>(velocityUpdateMeas_,velocity->header.stamp.toSec());
      updateAndPublish();
    }
  }

  /** \brief ROS service handler for resetting the filter.
   */
  bool resetServiceCallback(std_srvs::Empty::Request& /*request*/,
                            std_srvs::Empty::Response& /*response*/){
    requestReset();
    return true;
  }

  /** \brief ROS service handler for resetting the filter to a given pose.
   */
  bool resetToPoseServiceCallback(rovio::SrvResetToPose::Request& request,
                                  rovio::SrvResetToPose::Response& /*response*/){
    V3D WrWM(request.T_WM.position.x, request.T_WM.position.y,
             request.T_WM.position.z);
    QPD qWM(request.T_WM.orientation.w, request.T_WM.orientation.x,
            request.T_WM.orientation.y, request.T_WM.orientation.z);
    requestResetToPose(WrWM, qWM.inverted());
    return true;
  }

  /** \brief Reset the filter when the next IMU measurement is received.
   *         The orientaetion is initialized using an accel. measurement.
   */
  void requestReset() {
    std::lock_guard<std::mutex> lock(m_filter_);
    if (!init_state_.isInitialized()) {
      std::cout << "Reinitialization already triggered. Ignoring request...";
      return;
    }

    init_state_.state_ = FilterInitializationState::State::WaitForInitUsingAccel;
  }

  /** \brief Reset the filter when the next IMU measurement is received.
   *         The pose is initialized to the passed pose.
   *  @param WrWM - Position Vector, pointing from the World-Frame to the IMU-Frame, expressed in World-Coordinates.
   *  @param qMW  - Quaternion, expressing World-Frame in IMU-Coordinates (World Coordinates->IMU Coordinates)
   */
  void requestResetToPose(const V3D& WrWM, const QPD& qMW) {
    std::lock_guard<std::mutex> lock(m_filter_);
    if (!init_state_.isInitialized()) {
      std::cout << "Reinitialization already triggered. Ignoring request...";
      return;
    }

    init_state_.WrWM_ = WrWM;
    init_state_.qMW_ = qMW;
    init_state_.state_ = FilterInitializationState::State::WaitForInitExternalPose;
  }

  /** \brief Executes the update step of the filter and publishes the updated data.
   */
  void updateAndPublish(){
    if(init_state_.isInitialized()){
      // Execute the filter update.
      const double t1 = (double) cv::getTickCount();
      static double timing_T = 0;
      static int timing_C = 0;
      const double oldSafeTime = mpFilter_->safe_.t_;
      int c1 = std::get<0>(mpFilter_->updateTimelineTuple_).measMap_.size();
      double lastImageTime;
      if(std::get<0>(mpFilter_->updateTimelineTuple_).getLastTime(lastImageTime)){
        mpFilter_->updateSafe(&lastImageTime);
      }
      const double t2 = (double) cv::getTickCount();
      int c2 = std::get<0>(mpFilter_->updateTimelineTuple_).measMap_.size();
      timing_T += (t2-t1)/cv::getTickFrequency()*1000;
      timing_C += c1-c2;
      bool plotTiming = false;
      if(plotTiming){
        ROS_INFO_STREAM(" == Filter Update: " << (t2-t1)/cv::getTickFrequency()*1000 << " ms for processing " << c1-c2 << " images, average: " << timing_T/timing_C);
      }
      if(mpFilter_->safe_.t_ > oldSafeTime){ // Publish only if something changed
        for(int i=0;i<mtState::nCam_;i++){
          if(!mpFilter_->safe_.img_[i].empty() && mpImgUpdate_->doFrameVisualisation_){
            cv::imshow("Tracker" + std::to_string(i), mpFilter_->safe_.img_[i]);
            cv::waitKey(3);
          }
        }
        if(!mpFilter_->safe_.patchDrawing_.empty() && mpImgUpdate_->visualizePatches_){
          cv::imshow("Patches", mpFilter_->safe_.patchDrawing_);
          cv::waitKey(3);
        }

        // Obtain the save filter state.
        mtFilterState& filterState = mpFilter_->safe_;
	mtState& state = mpFilter_->safe_.state_;
        state.updateMultiCameraExtrinsics(&mpFilter_->multiCamera_);
        MXD& cov = mpFilter_->safe_.cov_;
        imuOutputCT_.transformState(state,imuOutput_);

        // Cout verbose for pose measurements
        if(mpImgUpdate_->verbose_){
          if(mpPoseUpdate_->inertialPoseIndex_ >=0){
            std::cout << "Transformation between inertial frames, IrIW, qWI: " << std::endl;
            std::cout << "  " << state.poseLin(mpPoseUpdate_->inertialPoseIndex_).transpose() << std::endl;
            std::cout << "  " << state.poseRot(mpPoseUpdate_->inertialPoseIndex_) << std::endl;
          }
          if(mpPoseUpdate_->bodyPoseIndex_ >=0){
            std::cout << "Transformation between body frames, MrMV, qVM: " << std::endl;
            std::cout << "  " << state.poseLin(mpPoseUpdate_->bodyPoseIndex_).transpose() << std::endl;
            std::cout << "  " << state.poseRot(mpPoseUpdate_->bodyPoseIndex_) << std::endl;
          }
        }

        // Send Map (Pose Sensor, I) to World (rovio-intern, W) transformation
        if(mpPoseUpdate_->inertialPoseIndex_ >=0){
          Eigen::Vector3d IrIW = state.poseLin(mpPoseUpdate_->inertialPoseIndex_);
          QPD qWI = state.poseRot(mpPoseUpdate_->inertialPoseIndex_);
          tf::StampedTransform tf_transform_WI;
          tf_transform_WI.frame_id_ = map_frame_;
          tf_transform_WI.child_frame_id_ = world_frame_;
          tf_transform_WI.stamp_ = ros::Time(mpFilter_->safe_.t_);
          tf_transform_WI.setOrigin(tf::Vector3(IrIW(0),IrIW(1),IrIW(2)));
          tf_transform_WI.setRotation(tf::Quaternion(qWI.x(),qWI.y(),qWI.z(),-qWI.w()));
          tb_.sendTransform(tf_transform_WI);
        }

        // Send IMU pose.
        tf::StampedTransform tf_transform_MW;
        tf_transform_MW.frame_id_ = world_frame_;
        tf_transform_MW.child_frame_id_ = imu_frame_;
        tf_transform_MW.stamp_ = ros::Time(mpFilter_->safe_.t_);
        tf_transform_MW.setOrigin(tf::Vector3(imuOutput_.WrWB()(0),imuOutput_.WrWB()(1),imuOutput_.WrWB()(2)));
        tf_transform_MW.setRotation(tf::Quaternion(imuOutput_.qBW().x(),imuOutput_.qBW().y(),imuOutput_.qBW().z(),-imuOutput_.qBW().w()));
        tb_.sendTransform(tf_transform_MW);

        // Send camera pose.
        for(int camID=0;camID<mtState::nCam_;camID++){
          tf::StampedTransform tf_transform_CM;
          tf_transform_CM.frame_id_ = imu_frame_;
          tf_transform_CM.child_frame_id_ = camera_frame_ + std::to_string(camID);
          tf_transform_CM.stamp_ = ros::Time(mpFilter_->safe_.t_);
          tf_transform_CM.setOrigin(tf::Vector3(state.MrMC(camID)(0),state.MrMC(camID)(1),state.MrMC(camID)(2)));
          tf_transform_CM.setRotation(tf::Quaternion(state.qCM(camID).x(),state.qCM(camID).y(),state.qCM(camID).z(),-state.qCM(camID).w()));
          tb_.sendTransform(tf_transform_CM);
        }

        // Publish Odometry
        if(pubOdometry_.getNumSubscribers() > 0 || forceOdometryPublishing_){
          // Compute covariance of output
          imuOutputCT_.transformCovMat(state,cov,imuOutputCov_);

          odometryMsg_.header.seq = msgSeq_;
          odometryMsg_.header.stamp = ros::Time(mpFilter_->safe_.t_);
          odometryMsg_.pose.pose.position.x = imuOutput_.WrWB()(0);
          odometryMsg_.pose.pose.position.y = imuOutput_.WrWB()(1);
          odometryMsg_.pose.pose.position.z = imuOutput_.WrWB()(2);
          odometryMsg_.pose.pose.orientation.w = -imuOutput_.qBW().w();
          odometryMsg_.pose.pose.orientation.x = imuOutput_.qBW().x();
          odometryMsg_.pose.pose.orientation.y = imuOutput_.qBW().y();
          odometryMsg_.pose.pose.orientation.z = imuOutput_.qBW().z();
          for(unsigned int i=0;i<6;i++){
            unsigned int ind1 = mtOutput::template getId<mtOutput::_pos>()+i;
            if(i>=3) ind1 = mtOutput::template getId<mtOutput::_att>()+i-3;
            for(unsigned int j=0;j<6;j++){
              unsigned int ind2 = mtOutput::template getId<mtOutput::_pos>()+j;
              if(j>=3) ind2 = mtOutput::template getId<mtOutput::_att>()+j-3;
              odometryMsg_.pose.covariance[j+6*i] = imuOutputCov_(ind1,ind2);
            }
          }
          odometryMsg_.twist.twist.linear.x = imuOutput_.BvB()(0);
          odometryMsg_.twist.twist.linear.y = imuOutput_.BvB()(1);
          odometryMsg_.twist.twist.linear.z = imuOutput_.BvB()(2);
          odometryMsg_.twist.twist.angular.x = imuOutput_.BwWB()(0);
          odometryMsg_.twist.twist.angular.y = imuOutput_.BwWB()(1);
          odometryMsg_.twist.twist.angular.z = imuOutput_.BwWB()(2);
          for(unsigned int i=0;i<6;i++){
            unsigned int ind1 = mtOutput::template getId<mtOutput::_vel>()+i;
            if(i>=3) ind1 = mtOutput::template getId<mtOutput::_ror>()+i-3;
            for(unsigned int j=0;j<6;j++){
              unsigned int ind2 = mtOutput::template getId<mtOutput::_vel>()+j;
              if(j>=3) ind2 = mtOutput::template getId<mtOutput::_ror>()+j-3;
              odometryMsg_.twist.covariance[j+6*i] = imuOutputCov_(ind1,ind2);
            }
          }
          pubOdometry_.publish(odometryMsg_);
        }

        if(pubPoseWithCovStamped_.getNumSubscribers() > 0 || forcePoseWithCovariancePublishing_){
          // Compute covariance of output
          imuOutputCT_.transformCovMat(state,cov,imuOutputCov_);

          estimatedPoseWithCovarianceStampedMsg_.header.seq = msgSeq_;
          estimatedPoseWithCovarianceStampedMsg_.header.stamp = ros::Time(mpFilter_->safe_.t_);
          estimatedPoseWithCovarianceStampedMsg_.pose.pose.position.x = imuOutput_.WrWB()(0);
          estimatedPoseWithCovarianceStampedMsg_.pose.pose.position.y = imuOutput_.WrWB()(1);
          estimatedPoseWithCovarianceStampedMsg_.pose.pose.position.z = imuOutput_.WrWB()(2);
          estimatedPoseWithCovarianceStampedMsg_.pose.pose.orientation.w = -imuOutput_.qBW().w();
          estimatedPoseWithCovarianceStampedMsg_.pose.pose.orientation.x = imuOutput_.qBW().x();
          estimatedPoseWithCovarianceStampedMsg_.pose.pose.orientation.y = imuOutput_.qBW().y();
          estimatedPoseWithCovarianceStampedMsg_.pose.pose.orientation.z = imuOutput_.qBW().z();

          for(unsigned int i=0;i<6;i++){
            unsigned int ind1 = mtOutput::template getId<mtOutput::_pos>()+i;
            if(i>=3) ind1 = mtOutput::template getId<mtOutput::_att>()+i-3;
            for(unsigned int j=0;j<6;j++){
              unsigned int ind2 = mtOutput::template getId<mtOutput::_pos>()+j;
              if(j>=3) ind2 = mtOutput::template getId<mtOutput::_att>()+j-3;
              estimatedPoseWithCovarianceStampedMsg_.pose.covariance[j+6*i] = imuOutputCov_(ind1,ind2);
            }
          }

          pubPoseWithCovStamped_.publish(estimatedPoseWithCovarianceStampedMsg_);

        }

        // Send IMU pose message.
        if(pubTransform_.getNumSubscribers() > 0 || forceTransformPublishing_){
          transformMsg_.header.seq = msgSeq_;
          transformMsg_.header.stamp = ros::Time(mpFilter_->safe_.t_);
          transformMsg_.transform.translation.x = imuOutput_.WrWB()(0);
          transformMsg_.transform.translation.y = imuOutput_.WrWB()(1);
          transformMsg_.transform.translation.z = imuOutput_.WrWB()(2);
          transformMsg_.transform.rotation.x = imuOutput_.qBW().x();
          transformMsg_.transform.rotation.y = imuOutput_.qBW().y();
          transformMsg_.transform.rotation.z = imuOutput_.qBW().z();
          transformMsg_.transform.rotation.w = -imuOutput_.qBW().w();
          pubTransform_.publish(transformMsg_);
        }

        if(pub_T_J_W_transform.getNumSubscribers() > 0 || forceTransformPublishing_){
          if (mpPoseUpdate_->inertialPoseIndex_ >= 0) {
            Eigen::Vector3d IrIW = state.poseLin(mpPoseUpdate_->inertialPoseIndex_);
            QPD qWI = state.poseRot(mpPoseUpdate_->inertialPoseIndex_);
            T_J_W_Msg_.header.seq = msgSeq_;
            T_J_W_Msg_.header.stamp = ros::Time(mpFilter_->safe_.t_);
            T_J_W_Msg_.transform.translation.x = IrIW(0);
            T_J_W_Msg_.transform.translation.y = IrIW(1);
            T_J_W_Msg_.transform.translation.z = IrIW(2);
            T_J_W_Msg_.transform.rotation.x = qWI.x();
            T_J_W_Msg_.transform.rotation.y = qWI.y();
            T_J_W_Msg_.transform.rotation.z = qWI.z();
            T_J_W_Msg_.transform.rotation.w = -qWI.w();
            pub_T_J_W_transform.publish(T_J_W_Msg_);
          }
        }

        // Publish Extrinsics
        for(int camID=0;camID<mtState::nCam_;camID++){
          if(pubExtrinsics_[camID].getNumSubscribers() > 0 || forceExtrinsicsPublishing_){
            extrinsicsMsg_[camID].header.seq = msgSeq_;
            extrinsicsMsg_[camID].header.stamp = ros::Time(mpFilter_->safe_.t_);
            extrinsicsMsg_[camID].pose.pose.position.x = state.MrMC(camID)(0);
            extrinsicsMsg_[camID].pose.pose.position.y = state.MrMC(camID)(1);
            extrinsicsMsg_[camID].pose.pose.position.z = state.MrMC(camID)(2);
            extrinsicsMsg_[camID].pose.pose.orientation.x = state.qCM(camID).x();
            extrinsicsMsg_[camID].pose.pose.orientation.y = state.qCM(camID).y();
            extrinsicsMsg_[camID].pose.pose.orientation.z = state.qCM(camID).z();
            extrinsicsMsg_[camID].pose.pose.orientation.w = -state.qCM(camID).w();
            for(unsigned int i=0;i<6;i++){
              unsigned int ind1 = mtState::template getId<mtState::_vep>(camID)+i;
              if(i>=3) ind1 = mtState::template getId<mtState::_vea>(camID)+i-3;
              for(unsigned int j=0;j<6;j++){
                unsigned int ind2 = mtState::template getId<mtState::_vep>(camID)+j;
                if(j>=3) ind2 = mtState::template getId<mtState::_vea>(camID)+j-3;
                extrinsicsMsg_[camID].pose.covariance[j+6*i] = cov(ind1,ind2);
              }
            }
            pubExtrinsics_[camID].publish(extrinsicsMsg_[camID]);
          }
        }

        // Publish IMU biases
        if(pubImuBias_.getNumSubscribers() > 0 || forceImuBiasPublishing_){
          imuBiasMsg_.header.seq = msgSeq_;
          imuBiasMsg_.header.stamp = ros::Time(mpFilter_->safe_.t_);
          imuBiasMsg_.angular_velocity.x = state.gyb()(0);
          imuBiasMsg_.angular_velocity.y = state.gyb()(1);
          imuBiasMsg_.angular_velocity.z = state.gyb()(2);
          imuBiasMsg_.linear_acceleration.x = state.acb()(0);
          imuBiasMsg_.linear_acceleration.y = state.acb()(1);
          imuBiasMsg_.linear_acceleration.z = state.acb()(2);
          for(int i=0;i<3;i++){
            for(int j=0;j<3;j++){
              imuBiasMsg_.angular_velocity_covariance[3*i+j] = cov(mtState::template getId<mtState::_gyb>()+i,mtState::template getId<mtState::_gyb>()+j);
            }
          }
          for(int i=0;i<3;i++){
            for(int j=0;j<3;j++){
              imuBiasMsg_.linear_acceleration_covariance[3*i+j] = cov(mtState::template getId<mtState::_acb>()+i,mtState::template getId<mtState::_acb>()+j);
            }
          }
          pubImuBias_.publish(imuBiasMsg_);
        }

        // PointCloud message.
        if(pubPcl_.getNumSubscribers() > 0 || pubMarkers_.getNumSubscribers() > 0 || forcePclPublishing_ || forceMarkersPublishing_){
          pclMsg_.header.seq = msgSeq_;
          pclMsg_.header.stamp = ros::Time(mpFilter_->safe_.t_);
          markerMsg_.header.seq = msgSeq_;
          markerMsg_.header.stamp = ros::Time(mpFilter_->safe_.t_);
          markerMsg_.points.clear();
          float badPoint = std::numeric_limits<float>::quiet_NaN();  // Invalid point.
          int offset = 0;

          FeatureDistance distance;
          double d,d_minus,d_plus;
          const double stretchFactor = 3;
          for (unsigned int i=0;i<mtState::nMax_; i++, offset += pclMsg_.point_step) {
            if(filterState.fsm_.isValid_[i]){
              // Get 3D feature coordinates.
              int camID = filterState.fsm_.features_[i].mpCoordinates_->camID_;
              distance = state.dep(i);
              d = distance.getDistance();
              const double sigma = sqrt(cov(mtState::template getId<mtState::_fea>(i)+2,mtState::template getId<mtState::_fea>(i)+2));
              distance.p_ -= stretchFactor*sigma;
              d_minus = distance.getDistance();
              if(d_minus > 1000) d_minus = 1000;
              if(d_minus < 0) d_minus = 0;
              distance.p_ += 2*stretchFactor*sigma;
              d_plus = distance.getDistance();
              if(d_plus > 1000) d_plus = 1000;
              if(d_plus < 0) d_plus = 0;
              Eigen::Vector3d bearingVector = filterState.state_.CfP(i).get_nor().getVec();
              const Eigen::Vector3d CrCPm = bearingVector*d_minus;
              const Eigen::Vector3d CrCPp = bearingVector*d_plus;
              const Eigen::Vector3f MrMPm = V3D(mpFilter_->multiCamera_.BrBC_[camID] + mpFilter_->multiCamera_.qCB_[camID].inverseRotate(CrCPm)).cast<float>();
              const Eigen::Vector3f MrMPp = V3D(mpFilter_->multiCamera_.BrBC_[camID] + mpFilter_->multiCamera_.qCB_[camID].inverseRotate(CrCPp)).cast<float>();

              // Get human readable output
              transformFeatureOutputCT_.setFeatureID(i);
              transformFeatureOutputCT_.setOutputCameraID(filterState.fsm_.features_[i].mpCoordinates_->camID_);
              transformFeatureOutputCT_.transformState(state,featureOutput_);
              transformFeatureOutputCT_.transformCovMat(state,cov,featureOutputCov_);
              featureOutputReadableCT_.transformState(featureOutput_,featureOutputReadable_);
              featureOutputReadableCT_.transformCovMat(featureOutput_,featureOutputCov_,featureOutputReadableCov_);

              // Get landmark output
              landmarkOutputImuCT_.setFeatureID(i);
              landmarkOutputImuCT_.transformState(state,landmarkOutput_);
              landmarkOutputImuCT_.transformCovMat(state,cov,landmarkOutputCov_);
              const Eigen::Vector3f MrMP = landmarkOutput_.get<LandmarkOutput::_lmk>().template cast<float>();

              // Write feature id, camera id, and rgb
              uint8_t gray = 255;
              uint32_t rgb = (gray << 16) | (gray << 8) | gray;
              uint32_t status = filterState.fsm_.features_[i].mpStatistics_->status_[0];
              memcpy(&pclMsg_.data[offset + pclMsg_.fields[0].offset], &filterState.fsm_.features_[i].idx_, sizeof(int));  // id
              memcpy(&pclMsg_.data[offset + pclMsg_.fields[1].offset], &camID, sizeof(int));  // cam id
              memcpy(&pclMsg_.data[offset + pclMsg_.fields[2].offset], &rgb, sizeof(uint32_t));  // rgb
              memcpy(&pclMsg_.data[offset + pclMsg_.fields[3].offset], &status, sizeof(int));  // status

              // Write coordinates to pcl message.
              memcpy(&pclMsg_.data[offset + pclMsg_.fields[4].offset], &MrMP[0], sizeof(float));  // x
              memcpy(&pclMsg_.data[offset + pclMsg_.fields[5].offset], &MrMP[1], sizeof(float));  // y
              memcpy(&pclMsg_.data[offset + pclMsg_.fields[6].offset], &MrMP[2], sizeof(float));  // z

              // Add feature bearing vector and distance
              const Eigen::Vector3f bearing = featureOutputReadable_.bea().template cast<float>();
              const float distance = static_cast<float>(featureOutputReadable_.dis());
              memcpy(&pclMsg_.data[offset + pclMsg_.fields[7].offset], &bearing[0], sizeof(float));  // x
              memcpy(&pclMsg_.data[offset + pclMsg_.fields[8].offset], &bearing[1], sizeof(float));  // y
              memcpy(&pclMsg_.data[offset + pclMsg_.fields[9].offset], &bearing[2], sizeof(float));  // z
              memcpy(&pclMsg_.data[offset + pclMsg_.fields[10].offset], &distance, sizeof(float)); // d

              // Add the corresponding covariance (upper triangular)
              Eigen::Matrix3f cov_MrMP = landmarkOutputCov_.cast<float>();
              int mCounter = 11;
              for(int row=0;row<3;row++){
                for(int col=row;col<3;col++){
                  memcpy(&pclMsg_.data[offset + pclMsg_.fields[mCounter].offset], &cov_MrMP(row,col), sizeof(float));
                  mCounter++;
                }
              }

              // Add distance uncertainty
              const float distance_cov = static_cast<float>(featureOutputReadableCov_(3,3));
              memcpy(&pclMsg_.data[offset + pclMsg_.fields[mCounter].offset], &distance_cov, sizeof(float));

              // Line markers (Uncertainty rays).
              geometry_msgs::Point point_near_msg;
              geometry_msgs::Point point_far_msg;
              point_near_msg.x = float(CrCPp[0]);
              point_near_msg.y = float(CrCPp[1]);
              point_near_msg.z = float(CrCPp[2]);
              point_far_msg.x = float(CrCPm[0]);
              point_far_msg.y = float(CrCPm[1]);
              point_far_msg.z = float(CrCPm[2]);
              markerMsg_.points.push_back(point_near_msg);
              markerMsg_.points.push_back(point_far_msg);
            }
            else {
              // If current feature is not valid copy NaN
              int id = -1;
              memcpy(&pclMsg_.data[offset + pclMsg_.fields[0].offset], &id, sizeof(int));  // id
              for(int j=1;j<pclMsg_.fields.size();j++){
                memcpy(&pclMsg_.data[offset + pclMsg_.fields[j].offset], &badPoint, sizeof(float));
              }
            }
          }
          pubPcl_.publish(pclMsg_);
          pubMarkers_.publish(markerMsg_);
        }
        if(pubPatch_.getNumSubscribers() > 0 || forcePatchPublishing_){
          patchMsg_.header.seq = msgSeq_;
          patchMsg_.header.stamp = ros::Time(mpFilter_->safe_.t_);
          int offset = 0;
          for (unsigned int i=0;i<mtState::nMax_; i++, offset += patchMsg_.point_step) {
            if(filterState.fsm_.isValid_[i]){
              memcpy(&patchMsg_.data[offset + patchMsg_.fields[0].offset], &filterState.fsm_.features_[i].idx_, sizeof(int));  // id
              // Add patch data
              for(int l=0;l<mtState::nLevels_;l++){
                for(int y=0;y<mtState::patchSize_;y++){
                  for(int x=0;x<mtState::patchSize_;x++){
                    memcpy(&patchMsg_.data[offset + patchMsg_.fields[1].offset + (l*mtState::patchSize_*mtState::patchSize_ + y*mtState::patchSize_ + x)*4], &filterState.fsm_.features_[i].mpMultilevelPatch_->patches_[l].patch_[y*mtState::patchSize_ + x], sizeof(float)); // Patch
                    memcpy(&patchMsg_.data[offset + patchMsg_.fields[2].offset + (l*mtState::patchSize_*mtState::patchSize_ + y*mtState::patchSize_ + x)*4], &filterState.fsm_.features_[i].mpMultilevelPatch_->patches_[l].dx_[y*mtState::patchSize_ + x], sizeof(float)); // dx
                    memcpy(&patchMsg_.data[offset + patchMsg_.fields[3].offset + (l*mtState::patchSize_*mtState::patchSize_ + y*mtState::patchSize_ + x)*4], &filterState.fsm_.features_[i].mpMultilevelPatch_->patches_[l].dy_[y*mtState::patchSize_ + x], sizeof(float)); // dy
                    memcpy(&patchMsg_.data[offset + patchMsg_.fields[4].offset + (l*mtState::patchSize_*mtState::patchSize_ + y*mtState::patchSize_ + x)*4], &filterState.mlpErrorLog_[i].patches_[l].patch_[y*mtState::patchSize_ + x], sizeof(float)); // error
                  }
                }
              }
            }
            else {
              // If current feature is not valid copy NaN
              int id = -1;
              memcpy(&patchMsg_.data[offset + patchMsg_.fields[0].offset], &id, sizeof(int));  // id
            }
          }

          pubPatch_.publish(patchMsg_);
        }
        gotFirstMessages_ = true;
      }
    }
  }
};

}


#endif /* ROVIO_ROVIONODE_HPP_ */
