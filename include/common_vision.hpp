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

#ifndef COMMON_VISION_HPP_
#define COMMON_VISION_HPP_

#include <Eigen/Dense>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <unordered_set>

using namespace Eigen;

namespace rovio{

template<int n_levels>
class ImagePyramid{
 public:
  ImagePyramid(){};
  ~ImagePyramid(){};
  cv::Mat imgs_[n_levels];
  void computeFromImage(const cv::Mat& img){
    img.copyTo(imgs_[0]);
    for(int i=1; i<n_levels; ++i){
      cv::pyrDown(imgs_[i-1],imgs_[i],cv::Size(imgs_[i-1].cols/2, imgs_[i-1].rows/2));
    }
  }
};

template<int size>
class PatchNew {
 public:
  static constexpr int size_ = size;
  uint8_t patch_[size_*size_] __attribute__ ((aligned (16)));
  uint8_t patchWithBorder_[(size_+2)*(size_+2)] __attribute__ ((aligned (16)));
  float dx_[size_*size_] __attribute__ ((aligned (16)));
  float dy_[size_*size_] __attribute__ ((aligned (16)));
  Matrix3f H_;
  float s_;
  bool validGradientParameters;
  bool hasValidPatch_; // Patch gets invalidates if extractPatchFromImage is not succesfull
  PatchNew(){
    static_assert(size_%2==0,"Patch size must be a multiple of 2");
    validGradientParameters = false;
    s_ = 0.0;
    hasValidPatch_ = false;
  }
  ~PatchNew(){}
  void computeGradientParameters(){
    if(validGradientParameters)
      return;
    H_.setZero();
    const int refStep = size_+2;
    float* it_dx = dx_;
    float* it_dy = dy_;
    uint8_t* it;
    Vector3f J;
    for(int y=0; y<size_; ++y){
      it = patchWithBorder_ + (y+1)*refStep + 1;
      for(int x=0; x<size_; ++x, ++it, ++it_dx, ++it_dy)
      {
        J[0] = 0.5 * (it[1] - it[-1]);
        J[1] = 0.5 * (it[refStep] - it[-refStep]);
        J[2] = 1;
        *it_dx = J[0];
        *it_dy = J[1];
        H_ += J*J.transpose();
      }
    }
    const float dXX = H_(0,0)/(size_*size_);
    const float dYY = H_(1,1)/(size_*size_);
    const float dXY = H_(0,1)/(size_*size_);
    // Find and return smaller eigenvalue:
    s_ = 0.5 * (dXX + dYY - sqrtf((dXX + dYY) * (dXX + dYY) - 4 * (dXX * dYY - dXY * dXY)));
    validGradientParameters = true;
  }
  bool extractPatchFromImage(const cv::Mat& img,const cv::Point2f c){
    const int halfpatch_size = size_/2+1;
    const int refStep = img.step.p[0];
    const float u = c.x;
    const float v = c.y;
    const int u_r = floor(u);
    const int v_r = floor(v);
    if(u_r < halfpatch_size || v_r < halfpatch_size || u_r >= img.cols-halfpatch_size || v_r >= img.rows-halfpatch_size){
      hasValidPatch_ = false;
      return false;
    }

    // compute interpolation weights
    const float subpix_x = u-u_r;
    const float subpix_y = v-v_r;
    const float wTL = (1.0-subpix_x)*(1.0-subpix_y);
    const float wTR = subpix_x * (1.0-subpix_y);
    const float wBL = (1.0-subpix_x)*subpix_y;
    const float wBR = subpix_x * subpix_y;

    // Extract Patch
    uint8_t* patch_ptr = patchWithBorder_;
    uint8_t* img_ptr;
    for(int y=0; y<(size_+2); ++y){
      img_ptr = (uint8_t*) img.data + (v_r+y-halfpatch_size)*refStep + u_r-halfpatch_size;
      for(int x=0; x<(size_+2); ++x, ++img_ptr, ++patch_ptr)
      {
        *patch_ptr = wTL*img_ptr[0] + wTR*img_ptr[1] + wBL*img_ptr[refStep] + wBR*img_ptr[refStep+1];
      }
    }

    extractPatchFromPatchWithBorder();
    validGradientParameters = false;
    hasValidPatch_ = true;
    return true;
  }
  void extractPatchFromPatchWithBorder(){
    uint8_t* it_patch = patch_;
    uint8_t* it_patchWithBorder;
    for(int y=1; y<size_+1; ++y, it_patch += size_){
      it_patchWithBorder = patchWithBorder_ + y*(size_+2) + 1;
      for(int x=0; x<size_; ++x)
        it_patch[x] = it_patchWithBorder[x];
    }
  }
  float getScore(){
    if(!validGradientParameters) computeGradientParameters();
    return s_;
  }
  Eigen::Matrix3f getHessian(){
    if(!validGradientParameters) computeGradientParameters();
    return H_;
  }
};

template<int n_levels,int patch_size>
class MultilevelPatchFeature{
 public:
  static const int nLevels_ = n_levels;
  PatchNew<patch_size> patches_[nLevels_];
  cv::Point2f c_;
  int idx_;
  Matrix3f H_;
  float s_;
  bool trackingSuccess_;
  int numConsecutiveFailures_;
  int numSuccess_;
  int numFailures_;
  bool hasValidPatches_;
  bool inFrame_;
  bool inFrameWithBorder_;
  bool isGoodFeature(){  // TODO param
    double r = static_cast<double>(numSuccess_)/static_cast<double>(numSuccess_+numFailures_);
    double s = r*std::max(static_cast<double>(numSuccess_)/100.0,1.0);
    int threshold = 3+s*10;
    return numConsecutiveFailures_ < threshold;
  }
  void addSuccess(){
    trackingSuccess_ = true;
    numSuccess_++;
    numConsecutiveFailures_ = 0;
  }
  void addFailure(){
    trackingSuccess_ = false;
    numFailures_++;
    numConsecutiveFailures_++;
  }
  MultilevelPatchFeature(int idx,const cv::Point2f& c){
    idx_ = idx;
    c_ = c;
    s_ = 0;
    hasValidPatches_ = false;
    trackingSuccess_ = false;
    numConsecutiveFailures_ = 0;
    numSuccess_ = 0;
    numFailures_ = 0;
    inFrame_ = false;
    inFrameWithBorder_ = false;
  }
  ~MultilevelPatchFeature(){}
  bool extractPatchesFromImage(const ImagePyramid<n_levels>& pyr){
    bool success = true;
    for(unsigned int i=0;i<nLevels_;i++){
      success = success & patches_[i].extractPatchFromImage(pyr.imgs_[i],c_*pow(0.5,i));
    }
    hasValidPatches_ = success;
    return success;
  }
  void computeMultilevelShiTomasiScore(){
    if(hasValidPatches_){
      for(unsigned int i=0;i<nLevels_;i++){
        H_ += pow(0.25,i)*patches_[i].getHessian();
      }
      const float dXX = H_(0,0)/(n_levels*patch_size*patch_size);
      const float dYY = H_(1,1)/(n_levels*patch_size*patch_size);
      const float dXY = H_(0,1)/(n_levels*patch_size*patch_size);
      // Find and return smaller eigenvalue:
      s_ = 0.5 * (dXX + dYY - sqrtf((dXX + dYY) * (dXX + dYY) - 4 * (dXX * dYY - dXY * dXY)));
    } else {
      s_ = -1;
    }
  }
  void getSpecificLevelScore(const int level){
    if(patches_[level].hasValidPatch_){
      s_ = patches_[level].getScore();
    } else {
      s_ = -1;
    }
  }
  bool align2DSingleLevel(const ImagePyramid<n_levels>& pyr,cv::Point2f& c, const int level){
    if(!patches_[level].hasValidPatch_) return false;
    const int halfpatch_size = patch_size/2;
    bool converged=false;
    patches_[level].computeGradientParameters();
    Matrix3f Hinv = patches_[level].H_.inverse();
    float mean_diff = 0;
    const int max_iter = 10;

    // Compute pixel location in new image:
    float u = c.x*pow(0.5,level);
    float v = c.y*pow(0.5,level);

    // termination condition
    const float min_update_squared = 0.03*0.03;
    const int refStep = pyr.imgs_[level].step.p[0];
    Vector3f update; update.setZero();
    for(int iter = 0; iter<max_iter; ++iter){
      int u_r = floor(u);
      int v_r = floor(v);
      if(u_r < halfpatch_size || v_r < halfpatch_size || u_r >= pyr.imgs_[level].cols-halfpatch_size || v_r >= pyr.imgs_[level].rows-halfpatch_size){
        break;
      }
      if(isnan(u) || isnan(v)){ // TODO check
        return false;
      }

      // compute interpolation weights
      const float subpix_x = u-u_r;
      const float subpix_y = v-v_r;
      const float wTL = (1.0-subpix_x)*(1.0-subpix_y);
      const float wTR = subpix_x * (1.0-subpix_y);
      const float wBL = (1.0-subpix_x)*subpix_y;
      const float wBR = subpix_x * subpix_y;

      // loop through search_patch, interpolate
      uint8_t* it_patch = patches_[level].patch_;
      float* it_dx = patches_[level].dx_;
      float* it_dy = patches_[level].dy_;
      uint8_t* it_img;
      Vector3f Jres; Jres.setZero();
      for(int y=0; y<patch_size; ++y){
        it_img = (uint8_t*) pyr.imgs_[level].data + (v_r+y-halfpatch_size)*refStep + u_r-halfpatch_size;
        for(int x=0; x<patch_size; ++x, ++it_img, ++it_patch, ++it_dx, ++it_dy){
          const float intensity = wTL*it_img[0] + wTR*it_img[1] + wBL*it_img[refStep] + wBR*it_img[refStep+1];
          const float res = intensity - *it_patch + mean_diff;
          Jres[0] -= res*(*it_dx);
          Jres[1] -= res*(*it_dy);
          Jres[2] -= res;
        }
      }

      update = Hinv * Jres;
      u += update[0];
      v += update[1];
      mean_diff += update[2];

      if(update[0]*update[0]+update[1]*update[1] < min_update_squared){
        converged=true;
        break;
      }
    }

    c.x = u*pow(2.0,level);
    c.y = v*pow(2.0,level);
    return converged;
  }
};

template<int n_levels,int patch_size>
class FeatureManager{
 public:
  std::vector<MultilevelPatchFeature<n_levels,patch_size>> features_;
  std::vector<MultilevelPatchFeature<n_levels,patch_size>> candidates_;
  int maxIdx_;
  void selectCandidates(std::vector<cv::Point2f>& detected_keypoints){ // TODO: add corner motion dependency
    float d2;
    constexpr float t2 = patch_size*patch_size; // TODO: param
    bool add;
    for (auto it = detected_keypoints.begin(); it != detected_keypoints.end(); ++it) {
      add = true;
      for (auto it_features = features_.begin(); it_features != features_.end(); ++it_features){
//        if(it_features->trackingSuccess_){ // TODO: rather inFrame
          d2 = pow(it->x-it_features->c_.x,2) + pow(it->y-it_features->c_.y,2);
          if(d2 < t2){
            add = false;
          }
//        }
      }
      if(add){
        candidates_.emplace_back(0,*it);
      }
    }
  }
  void extractCandidatePatchesFromImage(const ImagePyramid<n_levels>& pyr){
    for(auto it = candidates_.begin(); it != candidates_.end(); ++it){
      it->extractPatchesFromImage(pyr);
    }
  }
  void extractFeaturePatchesFromImage(const ImagePyramid<n_levels>& pyr){
    // TODO: should feature only be extracted if extractable on all levels?
    for(auto it = features_.begin(); it != features_.end(); ++it){
      it->extractPatchesFromImage(pyr);
    }
  }
  void computeCandidatesScore(int mode){
    if(mode < 0){
      for(auto it = candidates_.begin(); it != candidates_.end(); ++it){
        it->computeMultilevelShiTomasiScore();
      }
    } else {
      for(auto it = candidates_.begin(); it != candidates_.end(); ++it){
        it->getSpecificLevelScore(mode);
      }
    }
  }
  void addBestCandidates(const int maxN,cv::Mat& drawImg){
    float maxScore = -1.0;
    for(auto it = candidates_.begin(); it != candidates_.end(); ++it){;
      if(it->s_ > maxScore) maxScore = it->s_;
    }

    // Make buckets and fill based on score
    const unsigned int nBuckets = 100; // TODO param
    const float exponent = 0.5; // TODO param
    std::unordered_set<MultilevelPatchFeature<n_levels,patch_size>*> buckets[nBuckets];
    unsigned int newBucketID;
    for (auto it_cand = candidates_.begin(); it_cand != candidates_.end(); ++it_cand) {
      if(it_cand->s_ > 0.0){
        newBucketID = std::ceil(nBuckets*(pow(it_cand->s_/maxScore,exponent)))-1;
        if(newBucketID>nBuckets-1) newBucketID = nBuckets-1;
        buckets[newBucketID].insert(&(*it_cand));
      }
    }

    // Move buckets based on current features
    double d2;
    double t2 = pow(20,2); // TODO param
    double zeroDistancePenalty = nBuckets*1.0; // TODO param
    bool doDelete;
    for (auto it_feat = features_.begin(); it_feat != features_.end(); ++it_feat) {
      for (unsigned int bucketID = 1;bucketID < nBuckets;bucketID++) {
        for (auto it_cand = buckets[bucketID].begin();it_cand != buckets[bucketID].end();) {
          doDelete = false;
          d2 = std::pow(it_feat->c_.x - (*it_cand)->c_.x,2) + std::pow(it_feat->c_.y - (*it_cand)->c_.y,2);
          if(d2<t2){
            newBucketID = std::max((int)(bucketID - (t2-d2)/t2*zeroDistancePenalty),0);
            if(bucketID != newBucketID){
              buckets[newBucketID].insert(*it_cand);
              doDelete = true;
            }
          }
          if(doDelete){
            buckets[bucketID].erase(it_cand++);
          } else {
            ++it_cand;
          }
        }
      }
    }

    // Incrementally add features and update candidate buckets
    features_.reserve(candidates_.size());
    MultilevelPatchFeature<n_levels,patch_size>* mpNewFeature;
    int addedCount = 0;
    for (int bucketID = nBuckets-1;bucketID >= 0;bucketID--) {
      while(!buckets[bucketID].empty() && addedCount < maxN) {
        mpNewFeature = *(buckets[bucketID].begin());
        mpNewFeature->idx_ = maxIdx_++;
        buckets[bucketID].erase(mpNewFeature);
        features_.push_back(*mpNewFeature); // TODO: add further criteria like threshold on score
//        cv::circle(drawImg,mpNewFeature->c_, 5, cv::Scalar(0, 0, 0), -1, 8);
        addedCount++;
        for (unsigned int bucketID2 = 1;bucketID2 <= bucketID;bucketID2++) {
          for (auto it_cand = buckets[bucketID2].begin();it_cand != buckets[bucketID2].end();) {
            doDelete = false;
            d2 = std::pow(mpNewFeature->c_.x - (*it_cand)->c_.x,2) + std::pow(mpNewFeature->c_.y - (*it_cand)->c_.y,2);
            if(d2<t2){
              newBucketID = std::max((int)(bucketID2 - (t2-d2)/t2*zeroDistancePenalty),0);
              if(bucketID2 != newBucketID){
                buckets[newBucketID].insert(*it_cand);
                doDelete = true;
              }
            }
            if(doDelete){
              buckets[bucketID2].erase(it_cand++);
            } else {
              ++it_cand;
            }
          }
        }
      }
    }
  }
  void alignFeaturesSeq(const ImagePyramid<n_levels>& pyr,cv::Mat& drawImg,const int start_level,const int end_level){
    cv::Point2f c_new;
    bool success;
    for(auto it = features_.begin(); it != features_.end(); ++it){
      c_new = it->c_;
      success = true;
      for(int level = start_level;level>=end_level;--level){
        if(!it->align2DSingleLevel(pyr,c_new,level)){
          it->addFailure();
          success = false;
          break;
        }
      }
      if(success){
        it->addSuccess();
        cv::line(drawImg,c_new,it->c_,cv::Scalar(255,255,255), 2);
        cv::putText(drawImg,std::to_string(it->idx_),c_new,cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255));
        cv::circle(drawImg,c_new, 3, cv::Scalar(0, 0, 0), -1, 8);
        it->c_ = c_new;
      }
    }
  }
  void alignFeaturesSingle(const ImagePyramid<n_levels>& pyr,cv::Mat& drawImg,const int start_level,const int end_level){
    cv::Point2f c_new;
    bool success;
    for(auto it = features_.begin(); it != features_.end(); ++it){
      c_new = it->c_;
      success = true;
      for(int level = start_level;level>=end_level;--level){
        if(!it->align2DSingleLevel(pyr,c_new,level) && level==end_level){
          it->addFailure();
          success = false;
          break;
        }
      }
      if(success){
        it->addSuccess();
        cv::line(drawImg,c_new,it->c_,cv::Scalar(255,255,255), 2);
        cv::putText(drawImg,std::to_string(it->idx_),c_new,cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255));
        cv::circle(drawImg,c_new, 3, cv::Scalar(0, 0, 0), -1, 8);
        it->c_ = c_new;
      }
    }
  }
  void drawCandidates(cv::Mat& drawImg){
    for(auto it = candidates_.begin(); it != candidates_.end(); ++it){
      cv::circle(drawImg,it->c_, 3, cv::Scalar(0, 0, 0), -1, 8);
    }
  }
  void drawFeatures(cv::Mat& drawImg){
    for(auto it = features_.begin(); it != features_.end(); ++it){
      cv::circle(drawImg,it->c_, 3, cv::Scalar(0, 0, 0), -1, 8);
    }
  }
  void removeInvisible(){
    for(auto it = features_.begin(); it != features_.end();){
      if(!it->isGoodFeature()){
        it = features_.erase(it);
      } else {
        ++it;
      }
    }
  }
};

template <int n_levels>
void DetectFastCorners(const ImagePyramid<n_levels>& pyr, std::vector<cv::Point2f>& detected_keypoints,int level) {
  std::vector<cv::KeyPoint> keypoints;
  cv::FastFeatureDetector feature_detector_fast(10, true); // TODO param
  feature_detector_fast.detect(pyr.imgs_[level], keypoints);
  detected_keypoints.reserve(keypoints.size());
  for (auto it = keypoints.cbegin(), end = keypoints.cend(); it != end; ++it) {
    detected_keypoints.emplace_back(it->pt.x*pow(2.0,level), it->pt.y*pow(2.0,level));
  }
}

}


#endif /* COMMON_VISION_HPP_ */
