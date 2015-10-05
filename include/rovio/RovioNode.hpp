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

#include <queue>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <cv_bridge/cv_bridge.h>
#include <rovio/RovioOutput.h>
#include "rovio/CameraOutputCF.hpp"
#include "rovio/YprOutputCF.hpp"
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include "FeatureBearingOutputCF.hpp"
#include "FeatureLocationOutputCF.hpp"

namespace rovio {

/** \brief Class, defining the Rovio Node
 *
 *  @tparam FILTER  - \ref rovio::RovioFilter
 */
template<typename FILTER>
class RovioNode{
 public:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber subImu_;
  ros::Subscriber subImg0_;
  ros::Subscriber subImg1_;
  ros::Subscriber subGroundtruth_;
  ros::Publisher pubPose_;
  ros::Publisher pubRovioOutput_;
  ros::Publisher pubOdometry_;
  ros::Publisher pubTransform_;
  tf::TransformBroadcaster tb_;
  ros::Publisher pubPcl_;            /**<Publisher: Ros point cloud, visualizing the landmarks.*/
  ros::Publisher pubURays_;          /**<Publisher: Ros line marker, indicating the depth uncertainty of a landmark.*/

  typedef FILTER mtFilter;
  mtFilter* mpFilter_;
  typedef typename mtFilter::mtFilterState mtFilterState;
  typedef typename mtFilterState::mtState mtState;
  typedef typename decltype(mpFilter_->mPrediction_)::mtMeas mtPredictionMeas;
  mtPredictionMeas predictionMeas_;
  typedef typename std::tuple_element<0,decltype(mpFilter_->mUpdates_)>::type::mtMeas mtImgMeas;
  mtImgMeas imgUpdateMeas_;
  bool isInitialized_;
  geometry_msgs::PoseStamped poseMsg_;
  geometry_msgs::TransformStamped transformMsg_;
  nav_msgs::Odometry odometryMsg_;
  RovioOutput rovioOutputMsg_;
  int poseMsgSeq_;
  typedef StandardOutput mtOutput;
  mtOutput output_;
  CameraOutputCF<mtState> cameraOutputCF_;
  typename CameraOutputCF<mtState>::mtOutputCovMat outputCov_;

  typedef AttitudeOutput mtAttitudeOutput;
  mtAttitudeOutput attitudeOutput_;
  typedef YprOutput mtYprOutput;
  mtYprOutput yprOutput_;
  AttitudeToYprCF attitudeToYprCF_;
  AttitudeToYprCF::mtInputCovMat attitudeOutputCov_;
  AttitudeToYprCF::mtOutputCovMat yprOutputCov_;

  static const int groundtruthN_ = 3;
  std::queue<rot::RotationQuaternionPD> groundtruth_qCJ_;
  std::queue<Eigen::Vector3d> groundtruth_JrJC_;

  // ROS names for output tf frames.
  std::string world_frame_;
  std::string camera_frame_;
  std::string imu_frame_;

  /** \brief Constructor
   */
  RovioNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private, FILTER* mpFilter)
      : nh_(nh), nh_private_(nh_private), mpFilter_(mpFilter) {
    #ifndef NDEBUG
      ROS_WARN("====================== Debug Mode ======================");
    #endif
    subImu_ = nh_.subscribe("imu0", 1000, &RovioNode::imuCallback,this);
    subImg0_ = nh_.subscribe("cam0/image_raw", 1000, &RovioNode::imgCallback0,this);
    subImg1_ = nh_.subscribe("cam1/image_raw", 1000, &RovioNode::imgCallback1,this);
    subGroundtruth_ = nh_.subscribe("vrpn_client/pose", 1000, &RovioNode::groundtruthCallback,this);
    pubPose_ = nh_.advertise<geometry_msgs::PoseStamped>("rovio/pose", 1);
    pubTransform_ = nh_.advertise<geometry_msgs::TransformStamped>("rovio/transform", 1);
    pubRovioOutput_ = nh_.advertise<RovioOutput>("rovio/output", 1);
    pubOdometry_ = nh_.advertise<nav_msgs::Odometry>("rovio/odometry", 1);
    pubPcl_ = nh_.advertise<sensor_msgs::PointCloud2>("rovio/pcl", 1);
    pubURays_ = nh_.advertise<visualization_msgs::Marker>("rovio/urays", 1 );

    world_frame_ = "/world";
    camera_frame_ = "/camera";
    imu_frame_ = "/imu";
    nh_private_.param("world_frame", world_frame_, world_frame_);
    nh_private_.param("camera_frame", camera_frame_, camera_frame_);
    nh_private_.param("imu_frame", imu_frame_, imu_frame_);

    poseMsg_.header.frame_id = world_frame_;
    rovioOutputMsg_.header.frame_id = world_frame_;
    rovioOutputMsg_.points.header.frame_id = camera_frame_;
    odometryMsg_.header.frame_id = world_frame_;
    odometryMsg_.child_frame_id = camera_frame_;
    poseMsgSeq_ = 1;
    isInitialized_ = false;
  }

  /** \brief Destructor
   */
  ~RovioNode(){}

  /** \brief Tests the functionality of the rovio node.
   */
  void makeTest(){
    mtState testState = mpFilter_->init_.state_;
    unsigned int s = 2;
    testState.setRandom(s);            // TODO: debug with   doVECalibration = false and depthType = 0
    predictionMeas_.setRandom(s);
    imgUpdateMeas_.setRandom(s);

    testState.aux().camID_[0] = mtState::nCam_-1;
    for(int i=0;i<mtState::nMax_;i++){
      testState.aux().bearingMeas_[i].setRandom(s);
    }

    // Prediction
    mpFilter_->mPrediction_.testJacs(testState,predictionMeas_,1e-8,1e-6,0.1);

    // Update
    if(!std::get<0>(mpFilter_->mUpdates_).useDirectMethod_){
      for(int i=0;i<(std::min((int)mtState::nMax_,2));i++){
        testState.aux().activeFeature_ = i;
        testState.aux().activeCameraCounter_ = 0;
        const int camID = testState.aux().camID_[i];
        int activeCamID = (testState.aux().activeCameraCounter_ + camID)%mtState::nCam_;
        std::get<0>(mpFilter_->mUpdates_).featureBearingOutputCF_.setFeatureID(i);
        std::get<0>(mpFilter_->mUpdates_).featureBearingOutputCF_.setOutputCameraID(activeCamID);
        std::get<0>(mpFilter_->mUpdates_).testJacs(testState,imgUpdateMeas_,1e-8,1e-5,0.1);
        testState.aux().activeCameraCounter_ = mtState::nCam_-1;
        activeCamID = (testState.aux().activeCameraCounter_ + camID)%mtState::nCam_;
        std::get<0>(mpFilter_->mUpdates_).featureBearingOutputCF_.setOutputCameraID(activeCamID);
        std::get<0>(mpFilter_->mUpdates_).testJacs(testState,imgUpdateMeas_,1e-8,1e-5,0.1);
      }
    }

    // CF
    cameraOutputCF_.testJacInput(testState,testState,1e-8,1e-6,0.1);
    attitudeToYprCF_.testJacInput(1e-8,1e-6,s,0.1);
    rovio::FeatureBearingOutputCF<mtState> featureBearingOutputCFTest;
    featureBearingOutputCFTest.setFeatureID(0);
    if(mtState::nCam_>1){
      featureBearingOutputCFTest.setOutputCameraID(1);
      featureBearingOutputCFTest.testJacInput(1e-8,1e-5,s,0.1);
    }
    featureBearingOutputCFTest.setOutputCameraID(0);
    featureBearingOutputCFTest.testJacInput(1e-8,1e-5,s,0.1);
    rovio::FeatureLocationOutputCF<mtState> featureLocationOutputCFTest;
    featureLocationOutputCFTest.setFeatureID(0);
    if(mtState::nCam_>1){
      featureLocationOutputCFTest.setOutputCameraID(1);
      featureLocationOutputCFTest.testJacInput(1e-8,1e-5,s,0.1);
    }
    featureLocationOutputCFTest.setOutputCameraID(0);
    featureLocationOutputCFTest.testJacInput(1e-8,1e-5,s,0.1);

    // Zero Velocity Updates
    std::get<0>(mpFilter_->mUpdates_).zeroVelocityUpdate_.testJacs();
  }

  /** \brief Callback for IMU-Messages. Adds IMU measurements (as prediction measurements) to the filter.
   */
  void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg){
    predictionMeas_.template get<mtPredictionMeas::_acc>() = Eigen::Vector3d(imu_msg->linear_acceleration.x,imu_msg->linear_acceleration.y,imu_msg->linear_acceleration.z);
    predictionMeas_.template get<mtPredictionMeas::_gyr>() = Eigen::Vector3d(imu_msg->angular_velocity.x,imu_msg->angular_velocity.y,imu_msg->angular_velocity.z);
    if(isInitialized_){
      mpFilter_->addPredictionMeas(predictionMeas_,imu_msg->header.stamp.toSec());
    } else {
      mpFilter_->resetWithAccelerometer(predictionMeas_.template get<mtPredictionMeas::_acc>(),imu_msg->header.stamp.toSec());
      std::cout << std::setprecision(12);
      std::cout << "-- Filter: Initialized at t = " << imu_msg->header.stamp.toSec() << std::endl;
      isInitialized_ = true;
    }
  }

  /** \brief Image callback for the camera with ID 0
   *
   *  @param img - Image message.
   */
  void imgCallback0(const sensor_msgs::ImageConstPtr & img){
    imgCallback(img,0);
  }

  /** \brief Image callback for the camera with ID 1
   *
   * @param img - Image message.
   */
  void imgCallback1(const sensor_msgs::ImageConstPtr & img){
    if(mtState::nCam_ > 1) imgCallback(img,1); //TODO generalize
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
    if(isInitialized_ && !cv_img.empty()){
      double msgTime = img->header.stamp.toSec();
      if(msgTime != imgUpdateMeas_.template get<mtImgMeas::_aux>().imgTime_){
        imgUpdateMeas_.template get<mtImgMeas::_aux>().reset(msgTime);
      }
      imgUpdateMeas_.template get<mtImgMeas::_aux>().pyr_[camID].computeFromImage(cv_img,true);
      imgUpdateMeas_.template get<mtImgMeas::_aux>().isValidPyr_[camID] = true;

      if(imgUpdateMeas_.template get<mtImgMeas::_aux>().areAllValid()){
        mpFilter_->template addUpdateMeas<0>(imgUpdateMeas_,msgTime);
        double lastImuTime;
        if(mpFilter_->predictionTimeline_.getLastTime(lastImuTime)){
          auto rit = std::get<0>(mpFilter_->updateTimelineTuple_).measMap_.rbegin();
          while(rit != std::get<0>(mpFilter_->updateTimelineTuple_).measMap_.rend() && rit->first > lastImuTime){
            ++rit;
          }
          if(rit != std::get<0>(mpFilter_->updateTimelineTuple_).measMap_.rend()){
            updateAndPublish(rit->first);
          }
        }
      }
    }
  }

  /** \brief Groundtruth callback for external groundtruth
   *
   *  @param transform - Groundtruth message.
   */
  void groundtruthCallback(const geometry_msgs::TransformStamped::ConstPtr& transform){
    groundtruth_qCJ_.push(QPD(transform->transform.rotation.w,transform->transform.rotation.x,transform->transform.rotation.y,transform->transform.rotation.z));
    groundtruth_JrJC_.push(Eigen::Vector3d(transform->transform.translation.x,transform->transform.translation.y,transform->transform.translation.z));

    mpFilter_->safe_.groundtruth_qCJ_ = groundtruth_qCJ_.front();
    mpFilter_->safe_.groundtruth_JrJC_ = groundtruth_JrJC_.front();

    while(groundtruth_qCJ_.size() > groundtruthN_){
      groundtruth_qCJ_.pop();
      groundtruth_JrJC_.pop();
    }
  }

  /** \brief Executes the update step of the filter and publishes the updated data. @todo check this
   *
   *   @param updateTime   - Update Time.
   */
  void updateAndPublish(const double& updateTime){
    if(isInitialized_){

      // Execute the filter update.
      const double t1 = (double) cv::getTickCount();
      int c1 = std::get<0>(mpFilter_->updateTimelineTuple_).measMap_.size();
      static double timing_T = 0;
      static int timing_C = 0;
      const double oldSafeTime = mpFilter_->safe_.t_;
      mpFilter_->updateSafe(&updateTime); // TODO: continue only if actually updates
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
          if(!mpFilter_->safe_.img_[i].empty() && std::get<0>(mpFilter_->mUpdates_).doFrameVisualisation_){
            cv::imshow("Tracker" + std::to_string(i), mpFilter_->safe_.img_[i]);
            cv::waitKey(5);
          }
        }

        // Obtain the save filter state.
        mtFilterState& filterState = mpFilter_->safe_;
        mtState& state = mpFilter_->safe_.state_;
        typename mtFilterState::mtFilterCovMat& cov = mpFilter_->safe_.cov_;
        cameraOutputCF_.transformState(state,output_);
        cameraOutputCF_.transformCovMat(state,cov,outputCov_);

        // Get the position and orientation.
        Eigen::Vector3d WrWC = output_.WrWC();
        rot::RotationQuaternionPD qCW = output_.qCW();

        // Send camera pose message.
        poseMsg_.header.seq = poseMsgSeq_;
        poseMsg_.header.stamp = ros::Time(mpFilter_->safe_.t_);
        poseMsg_.pose.position.x = WrWC(0);
        poseMsg_.pose.position.y = WrWC(1);
        poseMsg_.pose.position.z = WrWC(2);
        poseMsg_.pose.orientation.w = qCW.w();
        poseMsg_.pose.orientation.x = qCW.x();
        poseMsg_.pose.orientation.y = qCW.y();
        poseMsg_.pose.orientation.z = qCW.z();
        pubPose_.publish(poseMsg_);

        // Send camera pose.
        tf::StampedTransform tf_transform_v;
        tf_transform_v.frame_id_ = world_frame_;
        tf_transform_v.child_frame_id_ = camera_frame_;
        tf_transform_v.stamp_ = ros::Time(mpFilter_->safe_.t_);
        tf_transform_v.setOrigin(tf::Vector3(WrWC(0),WrWC(1),WrWC(2)));
        tf_transform_v.setRotation(tf::Quaternion(qCW.x(),qCW.y(),qCW.z(),qCW.w()));
        tb_.sendTransform(tf_transform_v);

        // Send IMU pose.
        tf::StampedTransform tf_transform;
        tf_transform.frame_id_ = world_frame_;
        tf_transform.child_frame_id_ = imu_frame_;
        tf_transform.stamp_ = ros::Time(mpFilter_->safe_.t_);
        Eigen::Vector3d WrWM = state.WrWM();
        rot::RotationQuaternionPD qMW = state.qWM().inverted();
        tf_transform.setOrigin(tf::Vector3(WrWM(0),WrWM(1),WrWM(2)));
        tf_transform.setRotation(tf::Quaternion(qMW.x(),qMW.y(),qMW.z(),qMW.w()));
        tb_.sendTransform(tf_transform);

        // Send IMU pose message.
        transformMsg_.header = poseMsg_.header;
        transformMsg_.transform.translation.x = WrWM(0);
        transformMsg_.transform.translation.y = WrWM(1);
        transformMsg_.transform.translation.z = WrWM(2);
        transformMsg_.transform.rotation.x = qMW.x();
        transformMsg_.transform.rotation.y = qMW.y();
        transformMsg_.transform.rotation.z = qMW.z();
        transformMsg_.transform.rotation.w = qMW.w();
        pubTransform_.publish(transformMsg_);

        // Odometry
        odometryMsg_.header.seq = poseMsgSeq_;
        odometryMsg_.header.stamp = ros::Time(mpFilter_->safe_.t_);
        odometryMsg_.pose.pose.position.x = output_.WrWC()(0);
        odometryMsg_.pose.pose.position.y = output_.WrWC()(1);
        odometryMsg_.pose.pose.position.z = output_.WrWC()(2);
        odometryMsg_.pose.pose.orientation.w = output_.qCW().w();
        odometryMsg_.pose.pose.orientation.x = output_.qCW().x();
        odometryMsg_.pose.pose.orientation.y = output_.qCW().y();
        odometryMsg_.pose.pose.orientation.z = output_.qCW().z();
        for(unsigned int i=0;i<6;i++){
          unsigned int ind1 = mtOutput::template getId<mtOutput::_pos>()+i;
          if(i>=3) ind1 = mtOutput::template getId<mtOutput::_att>()+i-3;
          for(unsigned int j=0;j<6;j++){
            unsigned int ind2 = mtOutput::template getId<mtOutput::_pos>()+j;
            if(j>=3) ind2 = mtOutput::template getId<mtOutput::_att>()+j-3;
            odometryMsg_.pose.covariance[j+6*i] = outputCov_(ind1,ind2);
          }
        }
        odometryMsg_.twist.twist.linear.x = output_.CvC()(0);
        odometryMsg_.twist.twist.linear.y = output_.CvC()(1);
        odometryMsg_.twist.twist.linear.z = output_.CvC()(2);
        odometryMsg_.twist.twist.angular.x = output_.CwWC()(0);
        odometryMsg_.twist.twist.angular.y = output_.CwWC()(1);
        odometryMsg_.twist.twist.angular.z = output_.CwWC()(2);
        for(unsigned int i=0;i<6;i++){
          unsigned int ind1 = mtOutput::template getId<mtOutput::_vel>()+i;
          if(i>=3) ind1 = mtOutput::template getId<mtOutput::_ror>()+i-3;
          for(unsigned int j=0;j<6;j++){
            unsigned int ind2 = mtOutput::template getId<mtOutput::_vel>()+j;
            if(j>=3) ind2 = mtOutput::template getId<mtOutput::_ror>()+j-3;
            odometryMsg_.twist.covariance[j+6*i] = outputCov_(ind1,ind2);
          }
        }

        attitudeOutput_.template get<mtAttitudeOutput::_att>() = output_.qCW();
        attitudeOutputCov_ = outputCov_.template block<3,3>(mtOutput::template getId<mtOutput::_att>(),mtOutput::template getId<mtOutput::_att>());
        attitudeToYprCF_.transformState(attitudeOutput_,yprOutput_);
        attitudeToYprCF_.transformCovMat(attitudeOutput_,attitudeOutputCov_,yprOutputCov_);

        rovioOutputMsg_.header.seq = poseMsgSeq_;
        rovioOutputMsg_.header.stamp = ros::Time(mpFilter_->safe_.t_);
        rovioOutputMsg_.odometry = odometryMsg_;
        rovioOutputMsg_.ypr_odometry.x = yprOutput_.template get<mtYprOutput::_ypr>()(0);
        rovioOutputMsg_.ypr_odometry.y = yprOutput_.template get<mtYprOutput::_ypr>()(1);
        rovioOutputMsg_.ypr_odometry.z = yprOutput_.template get<mtYprOutput::_ypr>()(2);
        rovioOutputMsg_.ypr_odometry_sigma.x = yprOutputCov_(0,0);
        rovioOutputMsg_.ypr_odometry_sigma.y = yprOutputCov_(1,1);
        rovioOutputMsg_.ypr_odometry_sigma.z = yprOutputCov_(2,2);

        // IMU biases
        rovioOutputMsg_.acc_bias.x = state.acb()(0);
        rovioOutputMsg_.acc_bias.y = state.acb()(1);
        rovioOutputMsg_.acc_bias.z = state.acb()(2);
        rovioOutputMsg_.acc_bias_sigma.x = cov(mtState::template getId<mtState::_acb>()+0,mtState::template getId<mtState::_acb>()+0);
        rovioOutputMsg_.acc_bias_sigma.y = cov(mtState::template getId<mtState::_acb>()+1,mtState::template getId<mtState::_acb>()+1);
        rovioOutputMsg_.acc_bias_sigma.z = cov(mtState::template getId<mtState::_acb>()+2,mtState::template getId<mtState::_acb>()+2);
        rovioOutputMsg_.gyr_bias.x = state.gyb()(0);
        rovioOutputMsg_.gyr_bias.y = state.gyb()(1);
        rovioOutputMsg_.gyr_bias.z = state.gyb()(2);
        rovioOutputMsg_.gyr_bias_sigma.x = cov(mtState::template getId<mtState::_gyb>()+0,mtState::template getId<mtState::_gyb>()+0);
        rovioOutputMsg_.gyr_bias_sigma.y = cov(mtState::template getId<mtState::_gyb>()+1,mtState::template getId<mtState::_gyb>()+1);
        rovioOutputMsg_.gyr_bias_sigma.z = cov(mtState::template getId<mtState::_gyb>()+2,mtState::template getId<mtState::_gyb>()+2);

        // Extrinsics
        rovioOutputMsg_.extrinsics.pose.position.x = state.MrMC(0)(0);
        rovioOutputMsg_.extrinsics.pose.position.y = state.MrMC(0)(1);
        rovioOutputMsg_.extrinsics.pose.position.z = state.MrMC(0)(2);
        rovioOutputMsg_.extrinsics.pose.orientation.w = state.qCM(0).w();
        rovioOutputMsg_.extrinsics.pose.orientation.x = state.qCM(0).x();
        rovioOutputMsg_.extrinsics.pose.orientation.y = state.qCM(0).y();
        rovioOutputMsg_.extrinsics.pose.orientation.z = state.qCM(0).z();
        for(unsigned int i=0;i<6;i++){
          unsigned int ind1 = mtState::template getId<mtState::_vep>(0)+i;
          if(i>=3) ind1 = mtState::template getId<mtState::_vea>(0)+i-3;
          for(unsigned int j=0;j<6;j++){
            unsigned int ind2 = mtState::template getId<mtState::_vep>(0)+j;
            if(j>=3) ind2 = mtState::template getId<mtState::_vea>(0)+j-3;
            rovioOutputMsg_.extrinsics.covariance[j+6*i] = cov(ind1,ind2);
          }
        }
        attitudeOutput_.template get<mtAttitudeOutput::_att>() = state.qCM(0);
        attitudeOutputCov_ = cov.template block<3,3>(mtState::template getId<mtState::_vea>(0),mtState::template getId<mtState::_vea>(0));
        attitudeToYprCF_.transformState(attitudeOutput_,yprOutput_);
        attitudeToYprCF_.transformCovMat(attitudeOutput_,attitudeOutputCov_,yprOutputCov_);
        rovioOutputMsg_.ypr_extrinsics.x = yprOutput_.template get<mtYprOutput::_ypr>()(0);
        rovioOutputMsg_.ypr_extrinsics.y = yprOutput_.template get<mtYprOutput::_ypr>()(1);
        rovioOutputMsg_.ypr_extrinsics.z = yprOutput_.template get<mtYprOutput::_ypr>()(2);
        rovioOutputMsg_.ypr_extrinsics_sigma.x = yprOutputCov_(0,0);
        rovioOutputMsg_.ypr_extrinsics_sigma.y = yprOutputCov_(1,1);
        rovioOutputMsg_.ypr_extrinsics_sigma.z = yprOutputCov_(2,2);

        //Point cloud
        rovioOutputMsg_.points.header.seq = poseMsgSeq_;
        rovioOutputMsg_.points.header.stamp = ros::Time(mpFilter_->safe_.t_);
        rovioOutputMsg_.points.height = 1;

        pubRovioOutput_.publish(rovioOutputMsg_);
        pubOdometry_.publish(odometryMsg_);
        poseMsgSeq_++;

        // RVIZ Visualization
        ////////////////////////////////////////////////////////////

        // PointCloud2 message.
        sensor_msgs::PointCloud2 pcl_msg;
        pcl_msg.header.seq = poseMsgSeq_;
        pcl_msg.header.stamp = ros::Time(mpFilter_->safe_.t_);
        pcl_msg.header.frame_id = camera_frame_;
        pcl_msg.height = 1;               // Unordered point cloud.
        pcl_msg.width  = mtState::nMax_;  // Number of features/points.
        pcl_msg.fields.resize(4);

        pcl_msg.fields[0].name     = "x";
        pcl_msg.fields[0].offset   = 0;
        pcl_msg.fields[0].count    = 1;
        pcl_msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;

        pcl_msg.fields[1].name     = "y";
        pcl_msg.fields[1].offset   = 4;
        pcl_msg.fields[1].count    = 1;
        pcl_msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;

        pcl_msg.fields[2].name     = "z";
        pcl_msg.fields[2].offset   = 8;
        pcl_msg.fields[2].count    = 1;
        pcl_msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;

        pcl_msg.fields[3].name     = "rgb";
        pcl_msg.fields[3].offset   = 12;
        pcl_msg.fields[3].count    = 1;
        pcl_msg.fields[3].datatype = sensor_msgs::PointField::UINT32;

        pcl_msg.point_step = 16;
        pcl_msg.row_step = pcl_msg.point_step * pcl_msg.width;
        pcl_msg.data.resize(pcl_msg.row_step * pcl_msg.height);
        pcl_msg.is_dense = false;

        float badPoint = std::numeric_limits<float>::quiet_NaN();  // Invalid point.
        int offset = 0;
        double d, d_far, d_near, d_p, p_d, p_d_p;

        // Marker message (Uncertainty rays).
        visualization_msgs::Marker marker_msg;
        marker_msg.header.frame_id = camera_frame_;
        marker_msg.header.stamp = ros::Time(mpFilter_->safe_.t_);
        marker_msg.id = 0;
        marker_msg.type = visualization_msgs::Marker::LINE_LIST;
        marker_msg.action = visualization_msgs::Marker::ADD;
        marker_msg.pose.position.x = 0;
        marker_msg.pose.position.y = 0;
        marker_msg.pose.position.z = 0;
        marker_msg.pose.orientation.x = 0.0;
        marker_msg.pose.orientation.y = 0.0;
        marker_msg.pose.orientation.z = 0.0;
        marker_msg.pose.orientation.w = 1.0;
        marker_msg.scale.x = 0.04; // Line width.
        marker_msg.color.a = 1.0;
        marker_msg.color.r = 0.0;
        marker_msg.color.g = 1.0;
        marker_msg.color.b = 0.0;

        for (unsigned int i=0;i<mtState::nMax_; i++, offset += pcl_msg.point_step) {
          if(filterState.mlps_.isValid_[i]){

            // Get 3D feature coordinates.
            state.aux().depthMap_.map(filterState.state_.dep(i),d,d_p,p_d,p_d_p);
            const double sigma = cov(mtState::template getId<mtState::_dep>(i),mtState::template getId<mtState::_dep>(i));
            state.aux().depthMap_.map(filterState.state_.dep(i)-20*sigma,d_far,d_p,p_d,p_d_p);
            if(d_far > 1000 || d_far <= 0.0) d_far = 1000;
            state.aux().depthMap_.map(filterState.state_.dep(i)+20*sigma,d_near,d_p,p_d,p_d_p);
            const LWF::NormalVectorElement middle = filterState.state_.CfP(i);
            const Eigen::Vector3f pos = middle.getVec().cast<float>()*d;
            const Eigen::Vector3f pos_far = middle.getVec().cast<float>()*d_far;
            const Eigen::Vector3f pos_near = middle.getVec().cast<float>()*d_near;

            // Add feature coordinates to pcl message.
            memcpy(&pcl_msg.data[offset + 0],
                &pos[0], sizeof(float));  // x
            memcpy(&pcl_msg.data[offset + 4],
                &pos[1], sizeof(float));  // y
            memcpy(&pcl_msg.data[offset + 8],
                &pos[2], sizeof(float));  // z

            // Add color (gray values).
            uint8_t gray = 255;
            uint32_t rgb = (gray << 16) | (gray << 8) | gray;
            memcpy(&pcl_msg.data[offset + 12],
                &rgb, sizeof(uint32_t));

            // Line markers (Uncertainty rays).
            geometry_msgs::Point point_near_msg;
            geometry_msgs::Point point_far_msg;
            point_near_msg.x = float(pos_near[0]);
            point_near_msg.y = float(pos_near[1]);
            point_near_msg.z = float(pos_near[2]);
            point_far_msg.x = float(pos_far[0]);
            point_far_msg.y = float(pos_far[1]);
            point_far_msg.z = float(pos_far[2]);
            marker_msg.points.push_back(point_near_msg);
            marker_msg.points.push_back(point_far_msg);
          }
          else {
            // If current feature is not valid copy NaN
            memcpy(&pcl_msg.data[offset + 0], &badPoint,     // x
                sizeof(float));
            memcpy(&pcl_msg.data[offset + 4], &badPoint,     // y
                sizeof(float));
            memcpy(&pcl_msg.data[offset + 8], &badPoint,     // z
                sizeof(float));
            memcpy(&pcl_msg.data[offset + 12], &badPoint,
                sizeof(float));
          }
        }
        // Publish point cloud.
        pubPcl_.publish(pcl_msg);
        // Publish uncertainty rays.
        pubURays_.publish(marker_msg);
      }
    }
  }
};

}


#endif /* ROVIO_ROVIONODE_HPP_ */
