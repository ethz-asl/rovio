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
#include <Eigen/Eigenvalues>

using namespace Eigen;

namespace rovio{

class TrackingStatistics{
 public:
  TrackingStatistics(){
    inFrame_ = false;
    status_ = NOTFOUND;
  };
  ~TrackingStatistics(){};
  bool inFrame_;
  enum Status{
    TRACKED,
    OUTLIER,
    FOUND,
    INIT,
    NOTFOUND
  } status_;
};

class DrawPoint{
 public:
  cv::Point2f c_;
  cv::Size sigma_;
  double sigmaAngle_;
  bool hasSigma_;
  Eigen::EigenSolver<Eigen::Matrix2d> es_;
  DrawPoint(){
    c_ = cv::Point2f(0,0);
    sigma_ = cv::Size(2,2);
    hasSigma_ = false;
    sigmaAngle_ = 0.0;
  }
  void draw(cv::Mat& drawImg,const cv::Scalar& color){
    cv::Size size(2,2);
    cv::ellipse(drawImg,c_,size,0,0,360,color,-1,8,0);
    if(hasSigma_){
      cv::ellipse(drawImg,c_,sigma_,sigmaAngle_,0,360,color,1,8,0);
    }
  }
  void drawLine(cv::Mat& drawImg,const DrawPoint& p,const cv::Scalar& color){
    cv::line(drawImg,c_,p.c_,color, 2);
  }
  void drawText(cv::Mat& drawImg,const std::string& s,const cv::Scalar& color){
    cv::putText(drawImg,s,c_,cv::FONT_HERSHEY_SIMPLEX, 0.4, color);
  }
  void setSigmaFromCov(const Eigen::Matrix2d& cov){
    es_.compute(cov);
    sigmaAngle_ = std::atan2(es_.eigenvectors()(1,0).real(),es_.eigenvectors()(0,0).real())*180/M_PI;
    sigma_.width =  std::max(static_cast<int>(es_.eigenvalues()(0).real()+0.5),1);
    sigma_.height  = std::max(static_cast<int>(es_.eigenvalues()(1).real()+0.5),1);
    hasSigma_ = true;
  }
};

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
  ImagePyramid<n_levels>& operator=(const ImagePyramid<n_levels> &rhs) {
    for(unsigned int i=0;i<n_levels;i++){
      rhs.imgs_[i].copyTo(imgs_[i]);
    }
    return *this;
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
  bool hasValidPatches_;
  std::map<double,TrackingStatistics> trackingStatistics_;
  std::map<TrackingStatistics::Status,int> cumulativeStatistics_; // Should be 0 initialized
  int totCount_;

  DrawPoint log_previous_;
  DrawPoint log_prediction_;
  DrawPoint log_meas_;
  DrawPoint log_current_;

  void increaseStatistics(const double& t){
    if(!trackingStatistics_.empty() && t<trackingStatistics_.rbegin()->first) std::cout << "Warning: adding statistics NOT at end" << std::endl;
    if(!trackingStatistics_.empty()) cumulativeStatistics_[lastStatistics().status_]++;
    totCount_++;
    trackingStatistics_[t] = TrackingStatistics();
  }
  TrackingStatistics& lastStatistics(){
    assert(trackingStatistics_.rbegin() != trackingStatistics_.rend());
    return trackingStatistics_.rbegin()->second;
  }
  const TrackingStatistics& lastStatistics() const{
    assert(trackingStatistics_.rbegin() != trackingStatistics_.rend());
    return trackingStatistics_.rbegin()->second;
  }
  int countStatistics(const TrackingStatistics::Status s){
    return cumulativeStatistics_[s] + (int)(lastStatistics().status_ == s);
  }
  int countStatistics(const TrackingStatistics::Status s, const int n){
    int c = 0;
    auto it = trackingStatistics_.rbegin();
    for(int i=0;i<n && it != trackingStatistics_.rend();++i){
      if(it->second.status_ == s) c++;
      ++it;
    }
    return c;
  }
  bool isGoodFeature(){  // TODO param
    const int numTracked = countStatistics(TrackingStatistics::TRACKED);
    const double trackingRatio = static_cast<double>(numTracked)/static_cast<double>(totCount_-1);
    const double globalQuality = trackingRatio*std::min(static_cast<double>(cumulativeStatistics_[TrackingStatistics::TRACKED])/100.0,1.0); // param
    const int localRange = 10; // TODO: param
    const int localFailures = countStatistics(TrackingStatistics::FOUND,localRange)+countStatistics(TrackingStatistics::NOTFOUND,localRange)+countStatistics(TrackingStatistics::OUTLIER,localRange);
    const double localQuality = static_cast<double>(countStatistics(TrackingStatistics::TRACKED,localRange))/static_cast<double>(std::min(localRange,totCount_-1));
//    return localFailures < 2+globalQuality*5; // TODO: modes
    const double upper = 0.9; // TODO: param
    const double lower = 0.2; // TODO: param
    return localQuality > upper-(upper-lower)*globalQuality;

    /*
     * Local Quality: how is the tracking within the local range, OUTLIER/NOTFOUND is worse than not inImage
     * Global Quality:
     */
  }
  MultilevelPatchFeature(){
    idx_ = -1;
    c_ = cv::Point2f(0,0);
    reset();
  }
  MultilevelPatchFeature(int idx,const cv::Point2f& c){
    idx_ = idx;
    c_ = c;
    reset();
  }
  ~MultilevelPatchFeature(){}
  void reset(){
    s_ = 0;
    hasValidPatches_ = false;
    totCount_ = 0;
    cumulativeStatistics_.clear();
    trackingStatistics_.clear();
  }
  void init(const double& t){
    increaseStatistics(t);
    lastStatistics().status_ = TrackingStatistics::INIT;
  }
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
  bool align2D(const ImagePyramid<n_levels>& pyr,cv::Point2f& c, const int level1, const int level2){
    for(int level = level1; level <= level2; level++){
      if(!patches_[level].hasValidPatch_) return false;
    }
    const int halfpatch_size = patch_size/2;
    bool converged=false;
    Matrix3f H; H.setZero();
    for(int level = level1; level <= level2; level++){
      patches_[level].computeGradientParameters();
      H(0,0) += pow(0.25,level)*patches_[level].H_(0,0);
      H(0,1) += pow(0.25,level)*patches_[level].H_(0,1);
      H(1,0) += pow(0.25,level)*patches_[level].H_(1,0);
      H(1,1) += pow(0.25,level)*patches_[level].H_(1,1);
      H(2,0) += pow(0.5,level)*patches_[level].H_(2,0);
      H(2,1) += pow(0.5,level)*patches_[level].H_(2,1);
      H(0,2) += pow(0.5,level)*patches_[level].H_(0,2);
      H(1,2) += pow(0.5,level)*patches_[level].H_(1,2);
      H(2,2) += patches_[level].H_(2,2);
    }
    Matrix3f Hinv = H.inverse();
    float mean_diff = 0;
    const int max_iter = 10;

    // Compute pixel location in new image:
    float u = c.x;
    float v = c.y;

    // termination condition
    const float min_update_squared = 0.03*0.03;
    Vector3f update; update.setZero();
    for(int iter = 0; iter<max_iter; ++iter){
      if(isnan(u) || isnan(v)){ // TODO check
        return false;
      }
      Vector3f Jres; Jres.setZero();
      for(int level = level1; level <= level2; level++){
        const int refStep = pyr.imgs_[level].step.p[0];
        const float u_level = u*pow(0.5,level);
        const float v_level = v*pow(0.5,level);
        const int u_r = floor(u_level);
        const int v_r = floor(v_level);
        if(u_r < halfpatch_size || v_r < halfpatch_size || u_r >= pyr.imgs_[level].cols-halfpatch_size || v_r >= pyr.imgs_[level].rows-halfpatch_size){
          return false;
        }

        // compute interpolation weights
        const float subpix_x = u_level-u_r;
        const float subpix_y = v_level-v_r;
        const float wTL = (1.0-subpix_x)*(1.0-subpix_y);
        const float wTR = subpix_x * (1.0-subpix_y);
        const float wBL = (1.0-subpix_x)*subpix_y;
        const float wBR = subpix_x * subpix_y;

        // loop through search_patch, interpolate
        uint8_t* it_patch = patches_[level].patch_;
        float* it_dx = patches_[level].dx_;
        float* it_dy = patches_[level].dy_;
        uint8_t* it_img;
        for(int y=0; y<patch_size; ++y){
          it_img = (uint8_t*) pyr.imgs_[level].data + (v_r+y-halfpatch_size)*refStep + u_r-halfpatch_size;
          for(int x=0; x<patch_size; ++x, ++it_img, ++it_patch, ++it_dx, ++it_dy){
            const float intensity = wTL*it_img[0] + wTR*it_img[1] + wBL*it_img[refStep] + wBR*it_img[refStep+1];
            const float res = intensity - *it_patch + mean_diff;
            Jres[0] -= pow(0.5,level)*res*(*it_dx);
            Jres[1] -= pow(0.5,level)*res*(*it_dy);
            Jres[2] -= res;
          }
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

    c.x = u;
    c.y = v;
    return converged;
  }
  bool align2DSingleLevel(const ImagePyramid<n_levels>& pyr,cv::Point2f& c, const int level){
    return align2D(pyr,c,level,level);
  }
};

template<int n_levels,int patch_size,int nMax>
class FeatureManager{
 public:
  MultilevelPatchFeature<n_levels,patch_size> features_[nMax];
  std::unordered_set<unsigned int> validSet_;
  std::unordered_set<unsigned int> invalidSet_;
  std::vector<MultilevelPatchFeature<n_levels,patch_size>> candidates_;
  int maxIdx_;
  FeatureManager(){
    maxIdx_ = 0;
    for(unsigned int i=0;i<nMax;i++){
      invalidSet_.insert(i);
    }
  }
  int addFeature(const MultilevelPatchFeature<n_levels,patch_size>& feature){
    if(invalidSet_.empty()){
      std::cout << "Feature Manager: maximal number of feature reached" << std::endl;
      return -1;
    } else {
      unsigned int ind = *(invalidSet_.begin());
      invalidSet_.erase(ind);
      validSet_.insert(ind);
      features_[ind] = feature;
      features_[ind].idx_ = maxIdx_++;
      return ind;
    }
  }
  void removeFeature(unsigned int ind){
      validSet_.erase(ind);
      invalidSet_.insert(ind);
  }
  ~FeatureManager(){}
  void selectCandidates(std::vector<cv::Point2f>& detected_keypoints){ // TODO: add corner motion dependency
    float d2;
    constexpr float t2 = patch_size*patch_size; // TODO: param
    bool add;
    MultilevelPatchFeature<n_levels,patch_size>* mpFeature;
    for (auto it = detected_keypoints.begin(); it != detected_keypoints.end(); ++it) {
      add = true;
      for(auto it_f = validSet_.begin(); it_f != validSet_.end();++it_f){
        mpFeature = &features_[*it_f];
        d2 = pow(it->x-mpFeature->c_.x,2) + pow(it->y-mpFeature->c_.y,2); // TODO: check inFrame
        if(d2 < t2){
          add = false;
        }

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
    MultilevelPatchFeature<n_levels,patch_size>* mpFeature;
    for(auto it_f = validSet_.begin(); it_f != validSet_.end();++it_f){
      mpFeature = &features_[*it_f];
      if(mpFeature->lastStatistics().status_ == TrackingStatistics::TRACKED){ // TODO: adapt?
        mpFeature->extractPatchesFromImage(pyr);
      }
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
  std::unordered_set<unsigned int> addBestCandidates(const int maxN,cv::Mat& drawImg, const int nDetectionBuckets, const double scoreDetectionExponent, const double penaltyDistance, const double zeroDistancePenalty){
    std::unordered_set<unsigned int> newSet;
    float maxScore = -1.0;
    for(auto it = candidates_.begin(); it != candidates_.end(); ++it){;
      if(it->s_ > maxScore) maxScore = it->s_;
    }

    // Make buckets and fill based on score
    std::vector<std::unordered_set<MultilevelPatchFeature<n_levels,patch_size>*>> buckets(nDetectionBuckets,std::unordered_set<MultilevelPatchFeature<n_levels,patch_size>*>());
    unsigned int newBucketID;
    for (auto it_cand = candidates_.begin(); it_cand != candidates_.end(); ++it_cand) {
      if(it_cand->s_ > 0.0){
        newBucketID = std::ceil(nDetectionBuckets*(pow(it_cand->s_/maxScore,static_cast<float>(scoreDetectionExponent))))-1;
        if(newBucketID>nDetectionBuckets-1) newBucketID = nDetectionBuckets-1;
        buckets[newBucketID].insert(&(*it_cand));
      }
    }

    // Move buckets based on current features
    double d2;
    double t2 = pow(penaltyDistance,2);
    bool doDelete;
    MultilevelPatchFeature<n_levels,patch_size>* mpFeature;
    for(auto it_f = validSet_.begin(); it_f != validSet_.end();++it_f){
      mpFeature = &features_[*it_f];
      for (unsigned int bucketID = 1;bucketID < nDetectionBuckets;bucketID++) {
        for (auto it_cand = buckets[bucketID].begin();it_cand != buckets[bucketID].end();) {
          doDelete = false;
          d2 = std::pow(mpFeature->c_.x - (*it_cand)->c_.x,2) + std::pow(mpFeature->c_.y - (*it_cand)->c_.y,2);
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
    MultilevelPatchFeature<n_levels,patch_size>* mpNewFeature;
    int addedCount = 0;
    for (int bucketID = nDetectionBuckets-1;bucketID >= 0;bucketID--) {
      while(!buckets[bucketID].empty() && addedCount < maxN) {
        mpNewFeature = *(buckets[bucketID].begin());
        buckets[bucketID].erase(mpNewFeature);
        int ind = addFeature(*mpNewFeature);
        if(ind >= 0){
          newSet.insert(ind);
        }
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
    return newSet;
  }
  void alignFeaturesSeq(const ImagePyramid<n_levels>& pyr,cv::Mat& drawImg,const int start_level,const int end_level){ // TODO: clean
    cv::Point2f c_new;
    MultilevelPatchFeature<n_levels,patch_size>* mpFeature;
    for(auto it_f = validSet_.begin(); it_f != validSet_.end();++it_f){
      mpFeature = &features_[*it_f];
      c_new = mpFeature->c_;
      mpFeature->lastStatistics().status_ = TrackingStatistics::FOUND;
      for(int level = start_level;level>=end_level;--level){
        if(!mpFeature->align2D(pyr,c_new,level,start_level)){
          mpFeature->lastStatistics().status_ = TrackingStatistics::NOTFOUND;
          break;
        }
      }
      if(mpFeature->lastStatistics().status_ == TrackingStatistics::FOUND){
        mpFeature->c_ = c_new;
      }
    }
  }
  void alignFeaturesCom(const ImagePyramid<n_levels>& pyr,cv::Mat& drawImg,const int level1,const int level2){
    cv::Point2f c_new;
    MultilevelPatchFeature<n_levels,patch_size>* mpFeature;
    for(auto it_f = validSet_.begin(); it_f != validSet_.end();++it_f){
      mpFeature = &features_[*it_f];
      c_new = mpFeature->c_;
      if(!mpFeature->align2D(pyr,c_new,level1,level2)){
        mpFeature->lastStatistics().status_ = TrackingStatistics::NOTFOUND;
      } else {
        mpFeature->lastStatistics().status_ = TrackingStatistics::FOUND;
        cv::line(drawImg,c_new,mpFeature->c_,cv::Scalar(255,255,255), 2);
        cv::putText(drawImg,std::to_string(mpFeature->idx_),c_new,cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255));
        cv::circle(drawImg,c_new, 3, cv::Scalar(0, 0, 0), -1, 8);
        mpFeature->c_ = c_new;
      }
    }
  }
  void alignFeaturesSingle(const ImagePyramid<n_levels>& pyr,cv::Mat& drawImg,const int start_level,const int end_level){
    cv::Point2f c_new;
    MultilevelPatchFeature<n_levels,patch_size>* mpFeature;
    for(auto it_f = validSet_.begin(); it_f != validSet_.end();++it_f){
      mpFeature = &features_[*it_f];
      c_new = mpFeature->c_;
      mpFeature->lastStatistics().status_ = TrackingStatistics::FOUND;
      for(int level = start_level;level>=end_level;--level){
        if(!mpFeature->align2DSingleLevel(pyr,c_new,level) && level==end_level){
          mpFeature->lastStatistics().status_ = TrackingStatistics::NOTFOUND;
          break;
        }
      }
      if(mpFeature->lastStatistics().status_ == TrackingStatistics::FOUND){
        mpFeature->c_ = c_new;
      }
    }
  }
  void drawCandidates(cv::Mat& drawImg){
    for(auto it = candidates_.begin(); it != candidates_.end(); ++it){
      cv::circle(drawImg,it->c_, 3, cv::Scalar(0, 0, 0), -1, 8);
    }
  }
  void drawFeatures(cv::Mat& drawImg){
    MultilevelPatchFeature<n_levels,patch_size>* mpFeature;
    for(auto it_f = validSet_.begin(); it_f != validSet_.end();++it_f){
      mpFeature = &features_[*it_f];
      cv::circle(drawImg,mpFeature->c_, 3, cv::Scalar(0, 0, 0), -1, 8);
    }
  }
  void removeInvisible(){
    MultilevelPatchFeature<n_levels,patch_size>* mpFeature;
    for(auto it_f = validSet_.begin(); it_f != validSet_.end();){
      int ind = *it_f;
      it_f++;
      if(!features_[ind].isGoodFeature()){ // TODO: handle inFrame
        removeFeature(ind);
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
