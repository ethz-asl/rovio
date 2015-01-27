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

#ifndef ROVIO_NODE_HPP_
#define ROVIO_NODE_HPP_

#include "common_vision.hpp"

namespace rovio{

struct lk_landmark {
  std::vector<cv::Point2f> landmarks_;
  std::vector<unsigned int> idx_;
};

struct TrackedKeypointWithID {
  TrackedKeypointWithID(
      unsigned int _id,
      const Eigen::Matrix<double, 2, 1>& _location_previous_frame,
      const Eigen::Matrix<double, 2, 1>& _location_current_frame) :
    id(_id), location_previous_frame(_location_previous_frame),
    location_current_frame(_location_current_frame) { }
  unsigned int id;
  Eigen::Matrix<double, 2, 1> location_previous_frame;
  Eigen::Matrix<double, 2, 1> location_current_frame;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class TestNode{
 public:
  ros::NodeHandle nh_;
  ros::Subscriber subImu_;
  ros::Subscriber subImg_;
  ros::Publisher pubPose_;
  unsigned int landmark_idx_;
  static constexpr int n_levels_ = 4;
  static constexpr int lk_max_level_ = 3;
  static constexpr double lk_min_eigen_threshold_ = 0.001;
  static constexpr double good_features_to_track_quality_level_ = 0.01;
  static constexpr double good_features_to_track_min_distance_ = 10.0;
  cv::Mat curr_image_, prev_image_, draw_image_;
  cv::Mat curr_pyr_[n_levels_];
  cv::Mat prev_pyr_[n_levels_];
  bool draw_matches_, need_init_, use_fast_corners_;
  double uniformity_radius_;
  int fast_threshold_;
  unsigned int min_feature_count_, max_feature_count_;
  cv::TermCriteria termcrit_;
  cv::Size sub_pix_win_size_, win_size_;
  std::shared_ptr<lk_landmark> landmarks_prev_image_, landmarks_curr_image_;
  std::shared_ptr<cv::FeatureDetector> feature_detector_fast_;
  std::vector<TrackedKeypointWithID> feature_and_index_vec_;
  std::shared_ptr<RosCamera> camera_;
  TestNode(ros::NodeHandle& nh): nh_(nh), termcrit_(cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 20,0.03)),
      sub_pix_win_size_(cv::Size(10, 10)),win_size_(cv::Size(31, 31)){
    camera_.reset(new RosCamera("fpga21_cam0.yaml"));
    subImu_ = nh_.subscribe("imuMeas", 1000, &TestNode::imuCallback,this);
    subImg_ = nh_.subscribe("/cam0/image_raw", 1000, &TestNode::imgCallback,this);
    pubPose_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("filterPose", 1000);
    landmark_idx_ = 0;
    draw_matches_ = true;
    use_fast_corners_ = true;
    min_feature_count_ = 50;
    max_feature_count_ = 20; // Maximal number of feature which is added at a time (not total)
    fast_threshold_ = 10;
    uniformity_radius_ = 50;
    need_init_ = false;
    landmarks_prev_image_.reset(new lk_landmark);
    landmarks_curr_image_.reset(new lk_landmark);
    feature_detector_fast_.reset(new cv::FastFeatureDetector(fast_threshold_, true));
    cv::namedWindow("test1");
    cv::namedWindow("test2");
    cv::namedWindow("test3");
  };
  ~TestNode(){
    cv::destroyWindow("test1");
    cv::destroyWindow("test2");
    cv::destroyWindow("test3");
  }
  void updateSafe(){
  }
  void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg){
  }

  void imgCallback2(const sensor_msgs::ImageConstPtr & img){
//    static double last_timeStamp = 0.0;
//    double current_timeStamp = img->header.stamp.toSec();
//    cv_bridge::CvImagePtr cv_ptr;
//    try {
//      cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::TYPE_8UC1);
//      cv_ptr->image.copyTo(draw_image_);
//      curr_image_ = cv_ptr->image;
//    } catch (cv_bridge::Exception& e) {
//      ROS_ERROR("cv_bridge exception: %s", e.what());
//      return;
//    }
//    std::vector<cv::Point2f> points_temp;
//    DetectFastCorners(curr_image_, points_temp, landmarks_prev_image_->landmarks_);
//    // Assign IDs to points and add them to vectors
//    for (size_t i = 0; i < points_temp.size(); ++i) {
//      cv::circle(draw_image_,cv::Point(points_temp[i].x,points_temp[i].y),3, cv::Scalar(0, 0, 0), -1, 8);
////      landmarks_prev_image_->idx_.push_back(landmark_idx_++);
//    }
////    landmarks_prev_image_->landmarks_.insert(landmarks_prev_image_->landmarks_.end(), points_temp.begin(),points_temp.end());
//
//
//
////    for (int i = 0; i < landmarks_prev_image_->landmarks_.size(); ++i) {
////      cv::circle(draw_image_,cv::Point(landmarks_prev_image_->landmarks_[i].x,landmarks_prev_image_->landmarks_[i].y),3, cv::Scalar(0, 0, 0), -1, 8);
////    }
//
////    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
////      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
//
//    cv::imshow("test1", draw_image_);
//    cv::waitKey(30);
  }
  void imgCallback(const sensor_msgs::ImageConstPtr & img){
    imgCallback2(img);
    // Get image from msg
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::TYPE_8UC1);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    curr_image_ = cv_ptr->image;

    // Timing
    static double last_timeStamp = 0.0;
    double current_timeStamp = img->header.stamp.toSec();

    // Pyramid
    cv_ptr->image.copyTo(curr_pyr_[0]);
    for(int i=1; i<n_levels_; ++i){
//      curr_pyr_[i] = cv::Mat(curr_pyr_[i-1].rows/2, curr_pyr_[i-1].cols/2, CV_8U);
      cv::pyrDown(curr_pyr_[i-1],curr_pyr_[i],cv::Size(curr_pyr_[i-1].cols/2, curr_pyr_[i-1].rows/2));
    }

    //If we just started up
    if (prev_image_.empty()){
      curr_image_.copyTo(prev_image_);
      for(unsigned int i=0;i<n_levels_;i++){
        curr_pyr_[i].copyTo(prev_pyr_[i]);
      }
    }

    // Drawing
    if (draw_matches_)
      cv_ptr->image.copyTo(draw_image_);

    // Check if initialization of new feature is required
    if (landmarks_prev_image_->landmarks_.size() < min_feature_count_)
      need_init_ = true;

    // Initialize new features (on prev image)
    if (need_init_) {
      int detect_level = 1;
      ROS_INFO_STREAM("adding keypoints");

      // Get points from detector
      std::vector<cv::Point2f> points_temp;
      std::vector<cv::Point2f> current_points_temp;
      for (size_t i = 0; i < landmarks_prev_image_->landmarks_.size(); ++i) {
        current_points_temp.emplace_back(landmarks_prev_image_->landmarks_[i].x*pow(0.5,detect_level),landmarks_prev_image_->landmarks_[i].y*pow(0.5,detect_level));
      }
      DetectFastCorners(prev_pyr_[detect_level], points_temp, current_points_temp,max_feature_count_);
      // TODO: look at SVO detector

      // Assign IDs to points and add them to vectors
      for (size_t i = 0; i < points_temp.size(); ++i) {
        landmarks_prev_image_->idx_.push_back(landmark_idx_++);
        landmarks_prev_image_->landmarks_.emplace_back(points_temp[i].x*pow(2.0,detect_level), points_temp[i].y*pow(2.0,detect_level));
      }

      need_init_ = false;
    }

    ROS_INFO_STREAM(" landmark size: " << landmarks_prev_image_->landmarks_.size() << " " << landmarks_prev_image_->idx_.size());

    // Find matches in current image
    if (!landmarks_prev_image_->landmarks_.empty()) {
      std::vector<uchar> status;
      std::vector<float> err;

      size_t i, k;
      Patch<10> patch10;
      Patch<8> patch8;
      Vector2d curr_sta_pix;
      Vector2d curr_end_pix;
      Vector2d prev_pix;
      landmarks_curr_image_->landmarks_.clear();
      landmarks_curr_image_->idx_.clear();
      int start_level = 3;
      int end_level = 1;
      bool success = true;
      // Feature matches and indices, used for plotting
      feature_and_index_vec_.clear();
      feature_and_index_vec_.reserve(landmarks_prev_image_->landmarks_.size());
      for (i = 0; i < landmarks_prev_image_->landmarks_.size(); ++i) {
        prev_pix = Eigen::Vector2d(landmarks_prev_image_->landmarks_[i].x,landmarks_prev_image_->landmarks_[i].y);
        curr_sta_pix = prev_pix;
        success = true;
        for(int level = start_level;level>=end_level;--level){
          curr_end_pix = curr_sta_pix*pow(0.5,level);
          getPatchInterpolated(prev_pyr_[level],prev_pix*pow(0.5,level),5,patch10.data_);
          createPatchFromPatchWithBorder(patch10.data_,patch8.data_);
          if(level<=3){
            if(!align2D(curr_pyr_[level],patch10.data_,patch8.data_,10,curr_end_pix)){
              success = false; // TODO: rmeove for higher levels
              break;
            } else {
              curr_end_pix = curr_end_pix*pow(2.0,level);
              cv::line(draw_image_,cv::Point(curr_end_pix.x(),curr_end_pix.y()),cv::Point(curr_sta_pix.x(),curr_sta_pix.y()),cv::Scalar(250-60*(start_level-level), 250-60*(start_level-level), 250-60*(start_level-level)), 2);
            }
          }
          curr_sta_pix = curr_end_pix;
        }
        if(success){
          landmarks_curr_image_->landmarks_.emplace_back(curr_end_pix(0),curr_end_pix(1));
          landmarks_curr_image_->idx_.push_back(landmarks_prev_image_->idx_[i]);
          status.push_back(1);
          feature_and_index_vec_.emplace_back(landmarks_prev_image_->idx_[i],prev_pix,curr_sta_pix);// Drawing
          if (draw_matches_) {
            //TODO(omaris) Add flag to plot keypoint id or simply point
            if ((feature_and_index_vec_.back().id % 1) == 0) {
              cv::putText(
                  draw_image_,
                  std::to_string(feature_and_index_vec_.back().id),
                  cv::Point(
                      feature_and_index_vec_.back().location_current_frame.x(),
                      feature_and_index_vec_.back().location_current_frame.y()),
                  cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255));

//              cv::line(
//                  draw_image_,
//                  cv::Point(
//                      feature_and_index_vec_.back().location_current_frame.x(),
//                      feature_and_index_vec_.back().location_current_frame.y()),
//                  cv::Point(
//                      feature_and_index_vec_.back().location_previous_frame.x(),
//                      feature_and_index_vec_.back().location_previous_frame.y()),
//                  cv::Scalar(255, 255, 0), 2);
              cv::circle(
                  draw_image_,
                  cv::Point(
                      feature_and_index_vec_.back().location_current_frame.x(),
                      feature_and_index_vec_.back().location_current_frame.y()),
                  3, cv::Scalar(0, 0, 0), -1, 8);
            }
          }
        } else {
//          landmarks_curr_image_->landmarks_.emplace_back(0,0);
//          landmarks_curr_image_->idx_.push_back(0);
//          status.push_back(0);
        }
      }

//      // Sparse Optical Flow
//      cv::calcOpticalFlowPyrLK(prev_image_, curr_image_, landmarks_prev_image_->landmarks_, landmarks_curr_image_->landmarks_,
//                               status, err, win_size_, lk_max_level_, termcrit_, 0, lk_min_eigen_threshold_);
//      // Copy indices of previous to current
//      landmarks_curr_image_->idx_ = landmarks_prev_image_->idx_;

//
//      feature_tracker::optical_flow_measurementPtr of_measurementmsg(
//          new feature_tracker::optical_flow_measurement);
//      FILE * pFile;
//      pFile = fopen("vi_data.txt", "a");
//      fprintf(pFile, "New Frame:\t%.3f\n", img->header.stamp.toSec());
//      for (i = k = 0; i < landmarks_curr_image_->landmarks_.size(); ++i) {
//        if (!status[i])
//          continue;
//
////        feature_tracker::flow_vector oneFlow;
////
//        // Just for plotting
//        feature_and_index_vec_.emplace_back(
//            landmarks_prev_image_->idx_[i],
//            Eigen::Matrix<double, 2, 1>(landmarks_prev_image_->landmarks_[i].x,
//                                        landmarks_prev_image_->landmarks_[i].y),
//            Eigen::Matrix<double, 2, 1>(landmarks_curr_image_->landmarks_[i].x,
//                                        landmarks_curr_image_->landmarks_[i].y));
//
//        // Removes invalid landmarks
//        landmarks_curr_image_->landmarks_[k] =
//            landmarks_curr_image_->landmarks_[i];
//        landmarks_curr_image_->idx_[k] = landmarks_curr_image_->idx_[i];
//        ++k;
//
//        Eigen::Vector3d dirvec_f2 = camera_->camToWorld(
//            feature_and_index_vec_.back().location_current_frame.x(),
//            feature_and_index_vec_.back().location_current_frame.y());
//        Eigen::Vector3d dirvec_f1 = camera_->camToWorld(
//            feature_and_index_vec_.back().location_previous_frame.x(),
//            feature_and_index_vec_.back().location_previous_frame.y());
//
////        fprintf(pFile, "%.6f\t%.6f\t%.6f\t\t%.6f\t%.6f\t%.6f\t\n", dirvec_f1[0],
////                dirvec_f1[1], dirvec_f1[2], dirvec_f2[0], dirvec_f2[1],
////                dirvec_f2[2]);
////
////        oneFlow.p[0] = (dirvec_f2[0] + dirvec_f1[0])*0.5;
////        oneFlow.p[1] = (dirvec_f2[1] + dirvec_f1[1])*0.5;
////        oneFlow.p[2] = (dirvec_f2[2] + dirvec_f1[2])*0.5;
////
////        oneFlow.u[0] = (dirvec_f2[0] - dirvec_f1[0])/(current_timeStamp-last_timeStamp);
////        oneFlow.u[1] = (dirvec_f2[1] - dirvec_f1[1])/(current_timeStamp-last_timeStamp);
////        oneFlow.u[2] = (dirvec_f2[2] - dirvec_f1[2])/(current_timeStamp-last_timeStamp);
////
////        oneFlow.R = 0.5;  // ZMSSD distance.
////        of_measurementmsg->flow.push_back(oneFlow);
////
//
//        // Drawing
//        if (draw_matches_) {
//          //TODO(omaris) Add flag to plot keypoint id or simply point
//          if ((feature_and_index_vec_.back().id % 3) == 0) {
//            cv::putText(
//                draw_image_,
//                std::to_string(feature_and_index_vec_.back().id),
//                cv::Point(
//                    feature_and_index_vec_.back().location_current_frame.x(),
//                    feature_and_index_vec_.back().location_current_frame.y()),
//                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255));
//
//            cv::line(
//                draw_image_,
//                cv::Point(
//                    feature_and_index_vec_.back().location_current_frame.x(),
//                    feature_and_index_vec_.back().location_current_frame.y()),
//                cv::Point(
//                    feature_and_index_vec_.back().location_previous_frame.x(),
//                    feature_and_index_vec_.back().location_previous_frame.y()),
//                cv::Scalar(255, 255, 0), 2);
//            cv::circle(
//                draw_image_,
//                cv::Point(
//                    feature_and_index_vec_.back().location_current_frame.x(),
//                    feature_and_index_vec_.back().location_current_frame.y()),
//                3, cv::Scalar(0, 0, 0), -1, 8);
//          }
//        }
//      }
//      fclose(pFile);
//      static int seq = 0;
//      of_measurementmsg->header.stamp = img->header.stamp;
//      of_measurementmsg->header.frame_id = "/world";
//      of_measurementmsg->header.seq = seq++;
//      pub_flow_.publish(of_measurementmsg);
//
//      // Remove the END
//      landmarks_curr_image_->landmarks_.resize(k);
//      landmarks_curr_image_->idx_.resize(k);
    } else {
      need_init_ = true;
    }
    if (draw_matches_) {
      cv::imshow("LK Tracker Window", draw_image_);
      cv::imshow("test1", curr_pyr_[1]);
      cv::imshow("test2", curr_pyr_[2]);
      cv::imshow("test3", curr_pyr_[3]);
      cv::waitKey(30);
    }
    std::swap(landmarks_prev_image_, landmarks_curr_image_);
    cv::swap(prev_image_, curr_image_);
    for(unsigned int i=0;i<n_levels_;i++){
      cv::swap(prev_pyr_[i], curr_pyr_[i]);
    }
    last_timeStamp = current_timeStamp;
  }
};
}


#endif /* ROVIO_NODE_HPP_ */
