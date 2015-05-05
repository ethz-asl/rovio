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

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Odometry.h>
#include <cv_bridge/cv_bridge.h>
#include <rovio/RovioOutput.h>
#include "rovio/CameraOutputCF.hpp"
#include "rovio/YprOutputCF.hpp"
#include <tf/transform_broadcaster.h>

namespace rovio {

template<typename FILTER>
class RovioNode{
 public:
  ros::NodeHandle nh_;
  ros::Subscriber subImu_;
  ros::Subscriber subImg0_;
  ros::Subscriber subImg1_;
  ros::Publisher pubPose_;
  ros::Publisher pubRovioOutput_;
  ros::Publisher pubOdometry_;
  ros::Publisher pubTransform_;
  tf::TransformBroadcaster tb_;
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

  RovioNode(ros::NodeHandle& nh,FILTER* mpFilter): nh_(nh), mpFilter_(mpFilter){
    #ifndef NDEBUG
      ROS_WARN("====================== Debug Mode ======================");
    #endif
    subImu_ = nh_.subscribe("/imu0", 1000, &RovioNode::imuCallback,this);
    subImg0_ = nh_.subscribe("/cam0/image_raw", 1000, &RovioNode::imgCallback0,this);
    subImg1_ = nh_.subscribe("/cam1/image_raw", 1000, &RovioNode::imgCallback1,this);
    pubPose_ = nh_.advertise<geometry_msgs::PoseStamped>("/rovio/pose", 1);
    pubTransform_ = nh_.advertise<geometry_msgs::TransformStamped>("/rovio/transform", 1);
    pubRovioOutput_ = nh_.advertise<RovioOutput>("/rovio/output", 1);
    pubOdometry_ = nh_.advertise<nav_msgs::Odometry>("/rovio/odometry", 1);

//    // p21012_equidist_190215
//    Eigen::Matrix3d R_VM;
//    R_VM << 0.9999327192366118, -0.0044421301653885, 0.010715618492127002,
//            0.0043735142886046205, 0.9999698383876616, 0.0064183087897756765,
//            -0.01074380625488192, -0.006371012050474087, 0.9999219873733199;
//    Eigen::Vector3d VrVM;
//    VrVM << -0.03386081589435305, 0.005807520979411451, 0.0005138746353526602;
//    // p21012_radtan_190215 (false)
//    Eigen::Matrix3d R_VM;
//    R_VM << 0.9999327192366118, -0.0044421301653885, 0.010715618492127002,
//            0.0043735142886046205, 0.9999698383876616, 0.0064183087897756765,
//            -0.01074380625488192, -0.006371012050474087, 0.9999219873733199;
//    Eigen::Vector3d VrVM;
//    VrVM << -0.03386081589435305, 0.005807520979411451, 0.0005138746353526602;
//    mpFilter_->mPrediction_.setExtrinsics(R_VM,VrVM);

    poseMsg_.header.frame_id = "/world";
    rovioOutputMsg_.header.frame_id = "/world";
    rovioOutputMsg_.points.header.frame_id = "/camera";
    odometryMsg_.header.frame_id = "/world";
    odometryMsg_.child_frame_id = "/camera";
    poseMsgSeq_ = 1;
    isInitialized_ = false;
  }
  ~RovioNode(){}
  void makeTest(){
    mtState testState = mpFilter_->init_.state_;
    unsigned int s = 2;
    testState.setRandom(s); // TODO: debug with   doVECalibration = false and depthType = 0
    testState.template get<mtState::_aux>().useInUpdate_[0] = false;
    predictionMeas_.setRandom(s);
    imgUpdateMeas_.setRandom(s);

    mpFilter_->mPrediction_.testJacs(testState,predictionMeas_,1e-8,1e-6,0.1);
    std::get<0>(mpFilter_->mUpdates_).testJacs(testState,imgUpdateMeas_,1e-9,1e-6,0.1);
    cameraOutputCF_.testJacInput(testState,testState,1e-8,1e-6,0.1);
    attitudeToYprCF_.testJacInput(1e-8,1e-6,s,0.1);
  }
  void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg){
    predictionMeas_.template get<mtPredictionMeas::_acc>() = Eigen::Vector3d(imu_msg->linear_acceleration.x,imu_msg->linear_acceleration.y,imu_msg->linear_acceleration.z);
    predictionMeas_.template get<mtPredictionMeas::_gyr>() = Eigen::Vector3d(imu_msg->angular_velocity.x,imu_msg->angular_velocity.y,imu_msg->angular_velocity.z);
    if(isInitialized_){
      mpFilter_->addPredictionMeas(predictionMeas_,imu_msg->header.stamp.toSec());
    } else {
      mpFilter_->resetWithAccelerometer(predictionMeas_.template get<mtPredictionMeas::_acc>(),imu_msg->header.stamp.toSec());
      std::cout << "-- Filter: Initialized!" << std::endl;
      isInitialized_ = true;
    }
  }
  void imgCallback0(const sensor_msgs::ImageConstPtr & img){
    imgCallback(img,0);
  }
  void imgCallback1(const sensor_msgs::ImageConstPtr & img){
    imgCallback(img,1);
  }
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
  void updateAndPublish(const double& updateTime){
    if(isInitialized_){
      const double t1 = (double) cv::getTickCount();
      int c1 = std::get<0>(mpFilter_->updateTimelineTuple_).measMap_.size();
      static double timing_T = 0;
      static int timing_C = 0;
      mpFilter_->updateSafe(&updateTime);
      const double t2 = (double) cv::getTickCount();
      int c2 = std::get<0>(mpFilter_->updateTimelineTuple_).measMap_.size();
      timing_T += (t2-t1)/cv::getTickFrequency()*1000;
      timing_C += c1-c2;
      bool plotTiming = false;
      if(plotTiming){
        ROS_INFO_STREAM(" == Filter Update: " << (t2-t1)/cv::getTickFrequency()*1000 << " ms for processing " << c1-c2 << " images, average: " << timing_T/timing_C);
      }
      for(int i=0;i<mtState::nCam_;i++){
        if(!mpFilter_->safe_.img_[i].empty() && std::get<0>(mpFilter_->mUpdates_).doFrameVisualisation_){
          cv::imshow("Tracker" + std::to_string(i), mpFilter_->safe_.img_[i]);
          cv::waitKey(1);
        }
      }
      mtFilterState& filterState = mpFilter_->safe_;
      mtState& state = mpFilter_->safe_.state_;
      typename mtFilterState::mtFilterCovMat& cov = mpFilter_->safe_.cov_;
      cameraOutputCF_.transformState(state,output_);
      cameraOutputCF_.transformCovMat(state,cov,outputCov_);

      poseMsg_.header.seq = poseMsgSeq_;
      poseMsg_.header.stamp = ros::Time(mpFilter_->safe_.t_);
      poseMsg_.pose.position.x = output_.template get<mtOutput::_pos>()(0);
      poseMsg_.pose.position.y = output_.template get<mtOutput::_pos>()(1);
      poseMsg_.pose.position.z = output_.template get<mtOutput::_pos>()(2);
      poseMsg_.pose.orientation.w = output_.template get<mtOutput::_att>().w();
      poseMsg_.pose.orientation.x = output_.template get<mtOutput::_att>().x();
      poseMsg_.pose.orientation.y = output_.template get<mtOutput::_att>().y();
      poseMsg_.pose.orientation.z = output_.template get<mtOutput::_att>().z();
      pubPose_.publish(poseMsg_);

      tf::StampedTransform tf_transform;
      tf_transform.frame_id_ = "world";
      tf_transform.child_frame_id_ = "imu";
      tf_transform.stamp_ = ros::Time(mpFilter_->safe_.t_);
      Eigen::Vector3d IrIM = state.template get<mtState::_pos>();
      rot::RotationQuaternionPD qMI = state.template get<mtState::_att>().inverted();
      tf_transform.setOrigin(tf::Vector3(IrIM(0),IrIM(1),IrIM(2)));
      tf_transform.setRotation(tf::Quaternion(qMI.x(),qMI.y(),qMI.z(),qMI.w()));
      tb_.sendTransform(tf_transform);

      transformMsg_.header = poseMsg_.header;
      transformMsg_.transform.translation.x = IrIM(0);
      transformMsg_.transform.translation.y = IrIM(1);
      transformMsg_.transform.translation.z = IrIM(2);
      transformMsg_.transform.rotation.x = qMI.x();
      transformMsg_.transform.rotation.y = qMI.y();
      transformMsg_.transform.rotation.z = qMI.z();
      transformMsg_.transform.rotation.w = qMI.w();
      pubTransform_.publish(transformMsg_);

      rovioOutputMsg_.header.seq = poseMsgSeq_;
      rovioOutputMsg_.header.stamp = ros::Time(mpFilter_->safe_.t_);
      odometryMsg_.header.seq = poseMsgSeq_;
      odometryMsg_.header.stamp = ros::Time(mpFilter_->safe_.t_);

      // Odometry
      odometryMsg_.pose.pose.position.x = output_.template get<mtOutput::_pos>()(0);
      odometryMsg_.pose.pose.position.y = output_.template get<mtOutput::_pos>()(1);
      odometryMsg_.pose.pose.position.z = output_.template get<mtOutput::_pos>()(2);
      odometryMsg_.pose.pose.orientation.w = output_.template get<mtOutput::_att>().w();
      odometryMsg_.pose.pose.orientation.x = output_.template get<mtOutput::_att>().x();
      odometryMsg_.pose.pose.orientation.y = output_.template get<mtOutput::_att>().y();
      odometryMsg_.pose.pose.orientation.z = output_.template get<mtOutput::_att>().z();
      for(unsigned int i=0;i<6;i++){
        unsigned int ind1 = mtOutput::template getId<mtOutput::_pos>()+i;
        if(i>=3) ind1 = mtOutput::template getId<mtOutput::_att>()+i-3;
        for(unsigned int j=0;j<6;j++){
          unsigned int ind2 = mtOutput::template getId<mtOutput::_pos>()+j;
          if(j>=3) ind2 = mtOutput::template getId<mtOutput::_att>()+j-3;
          odometryMsg_.pose.covariance[j+6*i] = outputCov_(ind1,ind2);
        }
      }
      odometryMsg_.twist.twist.linear.x = output_.template get<mtOutput::_vel>()(0);
      odometryMsg_.twist.twist.linear.y = output_.template get<mtOutput::_vel>()(1);
      odometryMsg_.twist.twist.linear.z = output_.template get<mtOutput::_vel>()(2);
      odometryMsg_.twist.twist.angular.x = output_.template get<mtOutput::_ror>()(0);
      odometryMsg_.twist.twist.angular.y = output_.template get<mtOutput::_ror>()(1);
      odometryMsg_.twist.twist.angular.z = output_.template get<mtOutput::_ror>()(2);
      for(unsigned int i=0;i<6;i++){
        unsigned int ind1 = mtOutput::template getId<mtOutput::_vel>()+i;
        if(i>=3) ind1 = mtOutput::template getId<mtOutput::_ror>()+i-3;
        for(unsigned int j=0;j<6;j++){
          unsigned int ind2 = mtOutput::template getId<mtOutput::_vel>()+j;
          if(j>=3) ind2 = mtOutput::template getId<mtOutput::_ror>()+j-3;
          odometryMsg_.twist.covariance[j+6*i] = outputCov_(ind1,ind2);
        }
      }
      rovioOutputMsg_.odometry = odometryMsg_;
      attitudeOutput_.template get<mtAttitudeOutput::_att>() = output_.template get<mtOutput::_att>();
      attitudeOutputCov_ = outputCov_.template block<3,3>(mtOutput::template getId<mtOutput::_att>(),mtOutput::template getId<mtOutput::_att>());
      attitudeToYprCF_.transformState(attitudeOutput_,yprOutput_);
      attitudeToYprCF_.transformCovMat(attitudeOutput_,attitudeOutputCov_,yprOutputCov_);
      rovioOutputMsg_.ypr_odometry.x = yprOutput_.template get<mtYprOutput::_ypr>()(0);
      rovioOutputMsg_.ypr_odometry.y = yprOutput_.template get<mtYprOutput::_ypr>()(1);
      rovioOutputMsg_.ypr_odometry.z = yprOutput_.template get<mtYprOutput::_ypr>()(2);
      rovioOutputMsg_.ypr_odometry_sigma.x = yprOutputCov_(0,0);
      rovioOutputMsg_.ypr_odometry_sigma.y = yprOutputCov_(1,1);
      rovioOutputMsg_.ypr_odometry_sigma.z = yprOutputCov_(2,2);

      // IMU biases
      rovioOutputMsg_.acc_bias.x = state.template get<mtState::_acb>()(0);
      rovioOutputMsg_.acc_bias.y = state.template get<mtState::_acb>()(1);
      rovioOutputMsg_.acc_bias.z = state.template get<mtState::_acb>()(2);
      rovioOutputMsg_.acc_bias_sigma.x = cov(mtState::template getId<mtState::_acb>()+0,mtState::template getId<mtState::_acb>()+0);
      rovioOutputMsg_.acc_bias_sigma.y = cov(mtState::template getId<mtState::_acb>()+1,mtState::template getId<mtState::_acb>()+1);
      rovioOutputMsg_.acc_bias_sigma.z = cov(mtState::template getId<mtState::_acb>()+2,mtState::template getId<mtState::_acb>()+2);
      rovioOutputMsg_.gyr_bias.x = state.template get<mtState::_gyb>()(0);
      rovioOutputMsg_.gyr_bias.y = state.template get<mtState::_gyb>()(1);
      rovioOutputMsg_.gyr_bias.z = state.template get<mtState::_gyb>()(2);
      rovioOutputMsg_.gyr_bias_sigma.x = cov(mtState::template getId<mtState::_gyb>()+0,mtState::template getId<mtState::_gyb>()+0);
      rovioOutputMsg_.gyr_bias_sigma.y = cov(mtState::template getId<mtState::_gyb>()+1,mtState::template getId<mtState::_gyb>()+1);
      rovioOutputMsg_.gyr_bias_sigma.z = cov(mtState::template getId<mtState::_gyb>()+2,mtState::template getId<mtState::_gyb>()+2);

      // Extrinsics
      rovioOutputMsg_.extrinsics.pose.position.x = state.template get<mtState::_vep>(0)(0);
      rovioOutputMsg_.extrinsics.pose.position.y = state.template get<mtState::_vep>(0)(1);
      rovioOutputMsg_.extrinsics.pose.position.z = state.template get<mtState::_vep>(0)(2);
      rovioOutputMsg_.extrinsics.pose.orientation.w = state.template get<mtState::_vea>(0).w();
      rovioOutputMsg_.extrinsics.pose.orientation.x = state.template get<mtState::_vea>(0).x();
      rovioOutputMsg_.extrinsics.pose.orientation.y = state.template get<mtState::_vea>(0).y();
      rovioOutputMsg_.extrinsics.pose.orientation.z = state.template get<mtState::_vea>(0).z();
      for(unsigned int i=0;i<6;i++){
        unsigned int ind1 = mtState::template getId<mtState::_vep>(0)+i;
        if(i>=3) ind1 = mtState::template getId<mtState::_vea>(0)+i-3;
        for(unsigned int j=0;j<6;j++){
          unsigned int ind2 = mtState::template getId<mtState::_vep>(0)+j;
          if(j>=3) ind2 = mtState::template getId<mtState::_vea>(0)+j-3;
          rovioOutputMsg_.extrinsics.covariance[j+6*i] = cov(ind1,ind2);
        }
      }
      attitudeOutput_.template get<mtAttitudeOutput::_att>() = state.template get<mtState::_vea>(0);
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
    }
  }
};

}


#endif /* ROVIO_ROVIONODE_HPP_ */
