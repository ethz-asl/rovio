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

#include <ros/package.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <memory>
#include <iostream>
#include <locale>
#include <string>
#include <Eigen/StdVector>
#include "rovio/RovioFilter.hpp"
#include "rovio/RovioNode.hpp"
#include <boost/foreach.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>
#define foreach BOOST_FOREACH

#ifdef ROVIO_NMAXFEATURE
static constexpr int nMax_ = ROVIO_NMAXFEATURE;
#else
static constexpr int nMax_ = 25; // Maximal number of considered features in the filter state.
#endif

#ifdef ROVIO_NLEVELS
static constexpr int nLevels_ = ROVIO_NLEVELS;
#else
static constexpr int nLevels_ = 4; // // Total number of pyramid levels considered.
#endif

#ifdef ROVIO_PATCHSIZE
static constexpr int patchSize_ = ROVIO_PATCHSIZE;
#else
static constexpr int patchSize_ = 8; // Edge length of the patches (in pixel). Must be a multiple of 2!
#endif

#ifdef ROVIO_NCAM
static constexpr int nCam_ = ROVIO_NCAM;
#else
static constexpr int nCam_ = 1; // Used total number of cameras.
#endif

#ifdef ROVIO_NPOSE
static constexpr int nPose_ = ROVIO_NPOSE;
#else
static constexpr int nPose_ = 0; // Additional pose states.
#endif

typedef rovio::RovioFilter<rovio::FilterState<nMax_,nLevels_,patchSize_,nCam_,nPose_>> mtFilter;

int main(int argc, char** argv){
  ros::init(argc, argv, "rovio");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  std::string rootdir = ros::package::getPath("rovio"); // Leaks memory
  std::string filter_config = rootdir + "/cfg/rovio.info";

  nh_private.param("filter_config", filter_config, filter_config);

  // Filter
  std::shared_ptr<mtFilter> mpFilter(new mtFilter);
  mpFilter->readFromInfo(filter_config);

  // Force the camera calibration paths to the ones from ROS parameters.
  for (unsigned int camID = 0; camID < nCam_; ++camID) {
    std::string camera_config;
    if (nh_private.getParam("camera" + std::to_string(camID)
                            + "_config", camera_config)) {
      mpFilter->cameraCalibrationFile_[camID] = camera_config;
    }
  }
  mpFilter->refreshProperties();

  // Node
  rovio::RovioNode<mtFilter> rovioNode(nh, nh_private, mpFilter);
  rovioNode.makeTest();
  double resetTrigger = 0.0;
  nh_private.param("record_odometry", rovioNode.forceOdometryPublishing_, rovioNode.forceOdometryPublishing_);
  nh_private.param("record_pose_with_covariance_stamped", rovioNode.forcePoseWithCovariancePublishing_, rovioNode.forcePoseWithCovariancePublishing_);
  nh_private.param("record_transform", rovioNode.forceTransformPublishing_, rovioNode.forceTransformPublishing_);
  nh_private.param("record_extrinsics", rovioNode.forceExtrinsicsPublishing_, rovioNode.forceExtrinsicsPublishing_);
  nh_private.param("record_imu_bias", rovioNode.forceImuBiasPublishing_, rovioNode.forceImuBiasPublishing_);
  nh_private.param("record_pcl", rovioNode.forcePclPublishing_, rovioNode.forcePclPublishing_);
  nh_private.param("record_markers", rovioNode.forceMarkersPublishing_, rovioNode.forceMarkersPublishing_);
  nh_private.param("record_patch", rovioNode.forcePatchPublishing_, rovioNode.forcePatchPublishing_);
  nh_private.param("reset_trigger", resetTrigger, resetTrigger);

  std::cout << "Recording";
  if(rovioNode.forceOdometryPublishing_) std::cout << ", odometry";
  if(rovioNode.forceTransformPublishing_) std::cout << ", transform";
  if(rovioNode.forceExtrinsicsPublishing_) std::cout << ", extrinsics";
  if(rovioNode.forceImuBiasPublishing_) std::cout << ", imu biases";
  if(rovioNode.forcePclPublishing_) std::cout << ", point cloud";
  if(rovioNode.forceMarkersPublishing_) std::cout << ", markers";
  if(rovioNode.forcePatchPublishing_) std::cout << ", patch data";
  std::cout << std::endl;

  rosbag::Bag bagIn;
  std::string rosbag_filename = "dataset.bag";
  nh_private.param("rosbag_filename", rosbag_filename, rosbag_filename);
  bagIn.open(rosbag_filename, rosbag::bagmode::Read);

  rosbag::Bag bagOut;
  std::size_t found = rosbag_filename.find_last_of("/");
  std::string file_path = rosbag_filename.substr(0,found);
  std::string file_name = rosbag_filename.substr(found+1);
  if(file_path==rosbag_filename){
    file_path = ".";
    file_name = rosbag_filename;
  }

  std::stringstream stream;
  boost::posix_time::time_facet* facet = new boost::posix_time::time_facet();
  facet->format("%Y-%m-%d-%H-%M-%S");
  stream.imbue(std::locale(std::locale::classic(), facet));
  stream << ros::Time::now().toBoost() << "_" << nMax_ << "_" << nLevels_ << "_" << patchSize_ << "_" << nCam_  << "_" << nPose_;
  std::string filename_out = file_path + "/rovio/" + stream.str();
  nh_private.param("filename_out", filename_out, filename_out);
  std::string rosbag_filename_out = filename_out + ".bag";
  std::string info_filename_out = filename_out + ".info";
  std::cout << "Storing output to: " << rosbag_filename_out << std::endl;
  bagOut.open(rosbag_filename_out, rosbag::bagmode::Write);

  // Copy info
  std::ifstream  src(filter_config, std::ios::binary);
  std::ofstream  dst(info_filename_out,   std::ios::binary);
  dst << src.rdbuf();

  std::vector<std::string> topics;
  std::string imu_topic_name = "/imu0";
  nh_private.param("imu_topic_name", imu_topic_name, imu_topic_name);
  std::string cam0_topic_name = "/cam0/image_raw";
  nh_private.param("cam0_topic_name", cam0_topic_name, cam0_topic_name);
  std::string cam1_topic_name = "/cam1/image_raw";
  nh_private.param("cam1_topic_name", cam1_topic_name, cam1_topic_name);
  std::string odometry_topic_name = rovioNode.pubOdometry_.getTopic();
  std::string transform_topic_name = rovioNode.pubTransform_.getTopic();
  std::string extrinsics_topic_name[mtFilter::mtState::nCam_];
  for(int camID=0;camID<mtFilter::mtState::nCam_;camID++){
    extrinsics_topic_name[camID] = rovioNode.pubExtrinsics_[camID].getTopic();
  }
  std::string imu_bias_topic_name = rovioNode.pubImuBias_.getTopic();
  std::string pcl_topic_name = rovioNode.pubPcl_.getTopic();
  std::string u_rays_topic_name = rovioNode.pubMarkers_.getTopic();
  std::string patch_topic_name = rovioNode.pubPatch_.getTopic();

  topics.push_back(std::string(imu_topic_name));
  topics.push_back(std::string(cam0_topic_name));
  topics.push_back(std::string(cam1_topic_name));
  rosbag::View view(bagIn, rosbag::TopicQuery(topics));


  bool isTriggerInitialized = false;
  double lastTriggerTime = 0.0;
  for(rosbag::View::iterator it = view.begin();it != view.end() && ros::ok();it++){
    if(it->getTopic() == imu_topic_name){
      sensor_msgs::Imu::ConstPtr imuMsg = it->instantiate<sensor_msgs::Imu>();
      if (imuMsg != NULL) rovioNode.imuCallback(imuMsg);
    }
    if(it->getTopic() == cam0_topic_name){
      sensor_msgs::ImageConstPtr imgMsg = it->instantiate<sensor_msgs::Image>();
      if (imgMsg != NULL) rovioNode.imgCallback0(imgMsg);
    }
    if(it->getTopic() == cam1_topic_name){
      sensor_msgs::ImageConstPtr imgMsg = it->instantiate<sensor_msgs::Image>();
      if (imgMsg != NULL) rovioNode.imgCallback1(imgMsg);
    }
    ros::spinOnce();

    if(rovioNode.gotFirstMessages_){
      static double lastSafeTime = rovioNode.mpFilter_->safe_.t_;
      if(rovioNode.mpFilter_->safe_.t_ > lastSafeTime){
        if(rovioNode.forceOdometryPublishing_) bagOut.write(odometry_topic_name,ros::Time::now(),rovioNode.odometryMsg_);
        if(rovioNode.forceTransformPublishing_) bagOut.write(transform_topic_name,ros::Time::now(),rovioNode.transformMsg_);
        for(int camID=0;camID<mtFilter::mtState::nCam_;camID++){
          if(rovioNode.forceExtrinsicsPublishing_) bagOut.write(extrinsics_topic_name[camID],ros::Time::now(),rovioNode.extrinsicsMsg_[camID]);
        }
        if(rovioNode.forceImuBiasPublishing_) bagOut.write(imu_bias_topic_name,ros::Time::now(),rovioNode.imuBiasMsg_);
        if(rovioNode.forcePclPublishing_) bagOut.write(pcl_topic_name,ros::Time::now(),rovioNode.pclMsg_);
        if(rovioNode.forceMarkersPublishing_) bagOut.write(u_rays_topic_name,ros::Time::now(),rovioNode.markerMsg_);
        if(rovioNode.forcePatchPublishing_) bagOut.write(patch_topic_name,ros::Time::now(),rovioNode.patchMsg_);
        lastSafeTime = rovioNode.mpFilter_->safe_.t_;
      }
      if(!isTriggerInitialized){
        lastTriggerTime = lastSafeTime;
        isTriggerInitialized = true;
      }
      if(resetTrigger>0.0 && lastSafeTime - lastTriggerTime > resetTrigger){
        rovioNode.requestReset();
        rovioNode.mpFilter_->init_.state_.WrWM() = rovioNode.mpFilter_->safe_.state_.WrWM();
        rovioNode.mpFilter_->init_.state_.qWM() = rovioNode.mpFilter_->safe_.state_.qWM();
        lastTriggerTime = lastSafeTime;
      }
    }
  }

  bagOut.close();
  bagIn.close();


  return 0;
}
