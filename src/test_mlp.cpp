#include "rovio/Camera.hpp"
#include "gtest/gtest.h"
#include <assert.h>

#include "../include/rovio/ImagePyramid.hpp"
#include "../include/rovio/FeatureManager.hpp"
#include "../include/rovio/MultilevelPatchAlignment.hpp"

using namespace rovio;

class MLPTesting : public virtual ::testing::Test {
 protected:
  static const int nLevels_ = 2;
  static const int patchSize_ = 2;
  static const int imgSize_ = (patchSize_+2)*pow(2,nLevels_-1)+4;
  static const int nMax_ = 20;
  static const int nCam_ = 2;
  static const int dx_ = 2;
  static const int dy_ = 3;
  static constexpr float warpDistance_ = 2.0;

  ImagePyramid<nLevels_> pyr1_;
  ImagePyramid<nLevels_> pyr2_;
  cv::Mat img1_;
  cv::Mat img2_;
  Patch<patchSize_> p_;
  FeatureManager<nLevels_,patchSize_,nCam_> feature_;
  FeatureCoordinates c_;
  cv::Point2f pixel_;
  V3D bearing_;
  FeatureDistance d_;
  FeatureWarping warp_;
  FeatureStatistics<nCam_> stat_;
  MultilevelPatch<nLevels_,patchSize_> mp_;
  PixelCorners pixelCorners_;
  BearingCorners bearingCorners_;
  Eigen::Matrix2f affineTransform_;
  Camera camera_;
  MultilevelPatchAlignment<nLevels_,patchSize_> mpa_;
  MLPTesting(): c_(&camera_), warp_(warpDistance_,&camera_,&c_){
    static_assert(imgSize_*dx_*dy_<255,"imgSize to large for gradients");
    pixel_ = cv::Point2f(patchSize_/2+1,patchSize_/2+1);
    bearing_ = V3D(patchSize_/2+1,patchSize_/2+1,1);
    bearing_.normalize();
    c_.set_c(pixel_);
    affineTransform_.setIdentity();
    warp_.set_affineTransfrom(affineTransform_);
    pixelCorners_ = warp_.get_pixelCorners();
    bearingCorners_ = warp_.get_bearingCorners();

    stat_.localQualityRange_ = 3;
    stat_.localQualityRange_ = 4;
    stat_.minFrameGlobalQuality_ = 5;

    img1_ = cv::Mat::zeros(imgSize_,imgSize_,CV_8UC1);
    uint8_t* img_ptr = (uint8_t*) img1_.data;
    for(int i=0;i<imgSize_;i++){
      for(int j=0;j<imgSize_;j++, ++img_ptr){
        *img_ptr = i*dy_+j*dx_;
      }
    }
    img2_ = cv::Mat::zeros(imgSize_,imgSize_,CV_8UC1);
    img_ptr = (uint8_t*) img2_.data;
    for(int i=0;i<imgSize_;i++){
      for(int j=0;j<imgSize_;j++, ++img_ptr){
        if(j<imgSize_/2 & i<imgSize_/2){
          *img_ptr = 0;
        } else {
          *img_ptr = 255;
        }
      }
    }
    pyr1_.computeFromImage(img1_);
    pyr2_.computeFromImage(img2_);
  }
};

// Test constructors/reset
TEST_F(MLPTesting, constructors) {
  FeatureManager<nLevels_,patchSize_,nCam_> feature;
  ASSERT_EQ(feature.idx_,-1);
}

// Test coordinates getters and setters
TEST_F(MLPTesting, coordinates) {
  c_.set_c(pixel_);
  ASSERT_EQ(c_.valid_c_,true);
  ASSERT_EQ(c_.valid_nor_,false);
  ASSERT_EQ(c_.get_c(),pixel_);
  ASSERT_NEAR((c_.get_nor().getVec()-bearing_).norm(),0.0,1e-8);
  ASSERT_EQ(c_.valid_nor_,true);
  c_.set_nor(LWF::NormalVectorElement(bearing_));
  ASSERT_EQ(c_.valid_c_,false);
  ASSERT_EQ(c_.valid_nor_,true);
  ASSERT_NEAR((c_.get_nor().getVec()-bearing_).norm(),0.0,1e-8);
  ASSERT_EQ(c_.get_c(),pixel_);
  ASSERT_EQ(c_.valid_c_,true);
}

// Test corner getters and setters
TEST_F(MLPTesting, corner) {
  c_.set_c(pixel_);
  warp_.pixelCorners_[0] = cv::Point2f(0,0);
  warp_.pixelCorners_[1] = cv::Point2f(0,0);
  warp_.bearingCorners_[0].setZero();
  warp_.bearingCorners_[1].setZero();
  warp_.affineTransform_.setZero();
  warp_.set_pixelCorners(pixelCorners_);
  ASSERT_EQ(warp_.valid_pixelCorners_,true);
  ASSERT_EQ(warp_.valid_bearingCorners_,false);
  ASSERT_EQ(warp_.valid_affineTransform_,false);
  ASSERT_EQ(warp_.get_pixelCorners()[0],pixelCorners_[0]);
  ASSERT_EQ(warp_.get_pixelCorners()[1],pixelCorners_[1]);
  ASSERT_EQ(warp_.valid_pixelCorners_,true);
  ASSERT_EQ(warp_.valid_bearingCorners_,false);
  ASSERT_EQ(warp_.valid_affineTransform_,false);
  ASSERT_EQ((warp_.get_bearingCorners()[0]-bearingCorners_[0]).norm(),0.0);
  ASSERT_EQ((warp_.get_bearingCorners()[1]-bearingCorners_[1]).norm(),0.0);
  ASSERT_EQ(warp_.valid_bearingCorners_,true);
  ASSERT_EQ(warp_.get_affineTransform(),affineTransform_);
  ASSERT_EQ(warp_.valid_affineTransform_,true);
  warp_.pixelCorners_[0] = cv::Point2f(0,0);
  warp_.pixelCorners_[1] = cv::Point2f(0,0);
  warp_.bearingCorners_[0].setZero();
  warp_.bearingCorners_[1].setZero();
  warp_.affineTransform_.setZero();
  warp_.set_bearingCorners(bearingCorners_);
  ASSERT_EQ(warp_.valid_pixelCorners_,false);
  ASSERT_EQ(warp_.valid_bearingCorners_,true);
  ASSERT_EQ(warp_.valid_affineTransform_,false);
  ASSERT_EQ((warp_.get_bearingCorners()[0]-bearingCorners_[0]).norm(),0.0);
  ASSERT_EQ((warp_.get_bearingCorners()[1]-bearingCorners_[1]).norm(),0.0);
  ASSERT_EQ(warp_.valid_pixelCorners_,false);
  ASSERT_EQ(warp_.valid_bearingCorners_,true);
  ASSERT_EQ(warp_.valid_affineTransform_,false);
  ASSERT_EQ(warp_.get_pixelCorners()[0],pixelCorners_[0]);
  ASSERT_EQ(warp_.get_pixelCorners()[1],pixelCorners_[1]);
  ASSERT_EQ(warp_.valid_pixelCorners_,true);
  ASSERT_EQ(warp_.get_affineTransform(),affineTransform_);
  ASSERT_EQ(warp_.valid_affineTransform_,true);
  warp_.pixelCorners_[0] = cv::Point2f(0,0);
  warp_.pixelCorners_[1] = cv::Point2f(0,0);
  warp_.bearingCorners_[0].setZero();
  warp_.bearingCorners_[1].setZero();
  warp_.affineTransform_.setZero();
  warp_.set_affineTransfrom(affineTransform_);
  ASSERT_EQ(warp_.valid_pixelCorners_,false);
  ASSERT_EQ(warp_.valid_bearingCorners_,false);
  ASSERT_EQ(warp_.valid_affineTransform_,true);
  ASSERT_EQ(warp_.get_affineTransform(),affineTransform_);
  ASSERT_EQ(warp_.valid_pixelCorners_,false);
  ASSERT_EQ(warp_.valid_bearingCorners_,false);
  ASSERT_EQ(warp_.valid_affineTransform_,true);
  ASSERT_EQ(warp_.get_pixelCorners()[0],pixelCorners_[0]);
  ASSERT_EQ(warp_.get_pixelCorners()[1],pixelCorners_[1]);
  ASSERT_EQ(warp_.valid_pixelCorners_,true);
  ASSERT_EQ((warp_.get_bearingCorners()[0]-bearingCorners_[0]).norm(),0.0);
  ASSERT_EQ((warp_.get_bearingCorners()[1]-bearingCorners_[1]).norm(),0.0);
  ASSERT_EQ(warp_.valid_bearingCorners_,true);
}

// Test statistics
TEST_F(MLPTesting, statistics) {
  double localQuality[nCam_];
  for(int i=0;i<nCam_;i++){
    localQuality[i] = 1.0;
  }
  double localVisibility = 1.0;
  ASSERT_EQ(stat_.countTrackingStatistics(UNKNOWN),2);
  ASSERT_EQ(stat_.countTrackingStatistics(NOT_IN_FRAME),0);
  ASSERT_EQ(stat_.countTrackingStatistics(FAILED_ALIGNEMENT),0);
  ASSERT_EQ(stat_.countTrackingStatistics(FAILED_TRACKING),0);
  ASSERT_EQ(stat_.countTrackingStatistics(TRACKED),0);
  ASSERT_EQ(stat_.trackedInSomeFrame(),false);
  ASSERT_EQ(stat_.countTot(),1);
  ASSERT_EQ(stat_.countInFrame(),0);
  ASSERT_EQ(stat_.countTracked(),0);
  stat_.increaseStatistics(0.1);
  localVisibility = (1-1.0/stat_.localVisibilityRange_)*localVisibility;
  ASSERT_EQ(stat_.countTrackingStatistics(UNKNOWN),4);
  ASSERT_EQ(stat_.countTrackingStatistics(NOT_IN_FRAME),0);
  ASSERT_EQ(stat_.countTrackingStatistics(FAILED_ALIGNEMENT),0);
  ASSERT_EQ(stat_.countTrackingStatistics(FAILED_TRACKING),0);
  ASSERT_EQ(stat_.countTrackingStatistics(TRACKED),0);
  ASSERT_EQ(stat_.trackedInSomeFrame(),false);
  ASSERT_EQ(stat_.countTot(),2);
  ASSERT_EQ(stat_.countInFrame(),0);
  ASSERT_EQ(stat_.countTracked(),0);
  stat_.increaseStatistics(0.2);
  localVisibility = (1-1.0/stat_.localVisibilityRange_)*localVisibility;
  stat_.status_[0] = TRACKED;
  stat_.status_[1] = UNKNOWN;
  stat_.increaseStatistics(0.3);
  localVisibility = (1-1.0/stat_.localVisibilityRange_)*localVisibility + 1.0/stat_.localVisibilityRange_;
  localQuality[0] = (1-1.0/stat_.localQualityRange_)*localQuality[0] + 1.0/stat_.localQualityRange_;
  stat_.status_[0] = TRACKED;
  stat_.status_[1] = NOT_IN_FRAME;
  stat_.increaseStatistics(0.4);
  localVisibility = (1-1.0/stat_.localVisibilityRange_)*localVisibility + 1.0/stat_.localVisibilityRange_;
  localQuality[0] = (1-1.0/stat_.localQualityRange_)*localQuality[0] + 1.0/stat_.localQualityRange_;
  stat_.status_[0] = TRACKED;
  stat_.status_[1] = NOT_IN_FRAME;
  stat_.increaseStatistics(0.5);
  localVisibility = (1-1.0/stat_.localVisibilityRange_)*localVisibility + 1.0/stat_.localVisibilityRange_;
  localQuality[0] = (1-1.0/stat_.localQualityRange_)*localQuality[0] + 1.0/stat_.localQualityRange_;
  stat_.status_[0] = FAILED_TRACKING;
  stat_.status_[1] = NOT_IN_FRAME;
  stat_.increaseStatistics(0.6);
  localVisibility = (1-1.0/stat_.localVisibilityRange_)*localVisibility + 1.0/stat_.localVisibilityRange_;
  localQuality[0] = (1-1.0/stat_.localQualityRange_)*localQuality[0];
  stat_.status_[0] = FAILED_TRACKING;
  stat_.status_[1] = NOT_IN_FRAME;
  stat_.increaseStatistics(0.7);
  localVisibility = (1-1.0/stat_.localVisibilityRange_)*localVisibility + 1.0/stat_.localVisibilityRange_;
  localQuality[0] = (1-1.0/stat_.localQualityRange_)*localQuality[0];
  stat_.status_[0] = TRACKED;
  stat_.status_[1] = FAILED_ALIGNEMENT;
  stat_.increaseStatistics(0.8);
  localVisibility = (1-1.0/stat_.localVisibilityRange_)*localVisibility + 1.0/stat_.localVisibilityRange_;
  localQuality[0] = (1-1.0/stat_.localQualityRange_)*localQuality[0] + 1.0/stat_.localQualityRange_;
  localQuality[1] = (1-1.0/stat_.localQualityRange_)*localQuality[1];
  stat_.status_[0] = TRACKED;
  stat_.status_[1] = TRACKED;
  stat_.increaseStatistics(0.9);
  localVisibility = (1-1.0/stat_.localVisibilityRange_)*localVisibility + 1.0/stat_.localVisibilityRange_;
  localQuality[0] = (1-1.0/stat_.localQualityRange_)*localQuality[0] + 1.0/stat_.localQualityRange_;
  localQuality[1] = (1-1.0/stat_.localQualityRange_)*localQuality[1] + 1.0/stat_.localQualityRange_;
  stat_.status_[0] = TRACKED;
  stat_.status_[1] = TRACKED;
  ASSERT_EQ(stat_.countTrackingStatistics(UNKNOWN),5);
  ASSERT_EQ(stat_.countTrackingStatistics(NOT_IN_FRAME),4);
  ASSERT_EQ(stat_.countTrackingStatistics(FAILED_ALIGNEMENT),1);
  ASSERT_EQ(stat_.countTrackingStatistics(FAILED_TRACKING),2);
  ASSERT_EQ(stat_.countTrackingStatistics(TRACKED),8);
  ASSERT_EQ(stat_.trackedInSomeFrame(),true);
  ASSERT_EQ(stat_.countTot(),10);
  ASSERT_EQ(stat_.countInFrame(),11);
  ASSERT_EQ(stat_.countTracked(),6);
  ASSERT_EQ(stat_.localVisibility_,localVisibility);
  ASSERT_EQ(stat_.localQuality_[0],localQuality[0]);
  ASSERT_EQ(stat_.localQuality_[1],localQuality[1]);
  ASSERT_EQ(stat_.getGlobalQuality(),6.0/10.0);
  ASSERT_EQ(stat_.isGoodFeature(0.9,0.1),true);
}

// Test isMultilevelPatchInFrame
TEST_F(MLPTesting, isMultilevelPatchInFrame) {
  double c;
  for(unsigned int l=0;l<nLevels_;l++){
    c = patchSize_/2*pow(2,l)-0.1;
    c_.set_c(cv::Point2f(c,c));
    ASSERT_EQ(mp_.isMultilevelPatchInFrame(pyr1_,c_,l),false);
    c = patchSize_/2*pow(2,l);
    c_.set_c(cv::Point2f(c,c));
    ASSERT_EQ(mp_.isMultilevelPatchInFrame(pyr1_,c_,l),true);
    c = patchSize_/2*pow(2,l)+0.1;
    c_.set_c(cv::Point2f(c,c));
    ASSERT_EQ(mp_.isMultilevelPatchInFrame(pyr1_,c_,l),true);
    c = imgSize_-patchSize_/2*pow(2,l)-0.1;
    c_.set_c(cv::Point2f(c,c));
    ASSERT_EQ(mp_.isMultilevelPatchInFrame(pyr1_,c_,l),true);
    c = imgSize_-patchSize_/2*pow(2,l);
    c_.set_c(cv::Point2f(c,c));
    ASSERT_EQ(mp_.isMultilevelPatchInFrame(pyr1_,c_,l),true);
    c = imgSize_-patchSize_/2*pow(2,l)+0.1;
    c_.set_c(cv::Point2f(c,c));
    ASSERT_EQ(mp_.isMultilevelPatchInFrame(pyr1_,c_,l),false);
  }
  for(unsigned int l=0;l<nLevels_;l++){
    c = (patchSize_/2+1)*pow(2,l)-0.1;
    c_.set_c(cv::Point2f(c,c));
    ASSERT_EQ(mp_.isMultilevelPatchInFrame(pyr1_,c_,l,nullptr,true),false);
    c = (patchSize_/2+1)*pow(2,l);
    c_.set_c(cv::Point2f(c,c));
    ASSERT_EQ(mp_.isMultilevelPatchInFrame(pyr1_,c_,l,nullptr,true),true);
    c = (patchSize_/2+1)*pow(2,l)+0.1;
    c_.set_c(cv::Point2f(c,c));
    ASSERT_EQ(mp_.isMultilevelPatchInFrame(pyr1_,c_,l,nullptr,true),true);
    c = imgSize_-(patchSize_/2+1)*pow(2,l)-0.1;
    c_.set_c(cv::Point2f(c,c));
    ASSERT_EQ(mp_.isMultilevelPatchInFrame(pyr1_,c_,l,nullptr,true),true);
    c = imgSize_-(patchSize_/2+1)*pow(2,l);
    c_.set_c(cv::Point2f(c,c));
    ASSERT_EQ(mp_.isMultilevelPatchInFrame(pyr1_,c_,l,nullptr,true),true);
    c = imgSize_-(patchSize_/2+1)*pow(2,l)+0.1;
    c_.set_c(cv::Point2f(c,c));
    ASSERT_EQ(mp_.isMultilevelPatchInFrame(pyr1_,c_,l,nullptr,true),false);
  }
}

// Test extractMultilevelPatchFromImage (tests computeFromImage as well)
TEST_F(MLPTesting, extractMultilevelPatchFromImage) {
  pyr1_.computeFromImage(img1_,false);
  c_.set_c(cv::Point2f(imgSize_/2,imgSize_/2));
  mp_.extractMultilevelPatchFromImage(pyr1_,c_,nLevels_-1,nullptr,true);
  for(unsigned int l=0;l<nLevels_;l++){
    ASSERT_EQ(mp_.isValidPatch_[l],true);
    float* patch_ptr = mp_.patches_[l].patchWithBorder_;
    for(int i=0;i<patchSize_+2;i++){
      for(int j=0;j<patchSize_+2;j++, ++patch_ptr){
        const cv::Point2f c(i*pow(2,l)+imgSize_/2-(patchSize_/2+1.0-0.5)*pow(2,l)-0.5,
                            j*pow(2,l)+imgSize_/2-(patchSize_/2+1.0-0.5)*pow(2,l)-0.5);
        ASSERT_EQ(*patch_ptr,(float)(uint8_t)(c.x*dy_+c.y*dx_)); // down-sampling of image rounds to float, thus the casting
      }
    }
  }

  pyr1_.computeFromImage(img1_,true);
  c_.set_c(cv::Point2f(imgSize_/2,imgSize_/2));
  mp_.extractMultilevelPatchFromImage(pyr1_,c_,nLevels_-1,nullptr,false);
  for(unsigned int l=0;l<nLevels_;l++){
    ASSERT_EQ(mp_.isValidPatch_[l],true);
    float* patch_ptr = mp_.patches_[l].patch_; // Border effect, thus only valid for inner patch
    for(int i=0;i<patchSize_;i++){
      for(int j=0;j<patchSize_;j++, ++patch_ptr){
        const cv::Point2f c(i*pow(2,l)+imgSize_/2-(patchSize_/2-0.5)*pow(2,l)-0.5,
                            j*pow(2,l)+imgSize_/2-(patchSize_/2-0.5)*pow(2,l)-0.5);
        ASSERT_EQ(*patch_ptr,(float)(c.x*dy_+c.y*dx_)); // down-sampling hits integer thus no casting required
      }
    }
  }
}

// Test computeMultilevelShiTomasiScore
TEST_F(MLPTesting, computeMultilevelShiTomasiScore) {
  c_.set_c(cv::Point2f(imgSize_/2,imgSize_/2));
  mp_.extractMultilevelPatchFromImage(pyr2_,c_,nLevels_-1,nullptr,true);
  double s = 0.5 * (patchSize_ + patchSize_ - sqrtf((patchSize_ + patchSize_) * (patchSize_ + patchSize_) - 4 * (patchSize_ * patchSize_ - 1 * 1)));
  s = s*(255*0.5)*(255*0.5)/(patchSize_*patchSize_);
  int count = 0;
  double scale = 0;
  for(unsigned int l=0;l<nLevels_;l++){
    mp_.getSpecificLevelScore(l);
    ASSERT_EQ(mp_.s_,s);
    count++;
    scale += pow(0.25,l);
  }
  scale /= count;
  mp_.computeMultilevelShiTomasiScore();
  ASSERT_EQ(mp_.s_,s*scale);
}

// Test getLinearAlignEquations
TEST_F(MLPTesting, getLinearAlignEquations) {
  Eigen::MatrixXf A;
  Eigen::MatrixXf b;
  b.setZero();
  c_.set_c(cv::Point2f(imgSize_/2,imgSize_/2));
  mp_.extractMultilevelPatchFromImage(pyr2_,c_);
  c_.set_c(cv::Point2f(imgSize_/2+1,imgSize_/2+1));
  // NOTE: only works for patch size = 2, nLevels = 2
  ASSERT_EQ(mpa_.getLinearAlignEquations(pyr2_,mp_,c_,nullptr,0,nLevels_-1,A,b),true);
  float meanError = (255+255*0.75)/8;
  ASSERT_EQ(b(0),255-meanError);
  ASSERT_EQ(b(1),-meanError);
  ASSERT_EQ(b(2),-meanError);
  ASSERT_EQ(b(3),-meanError);
  ASSERT_EQ(b(4),255*0.75-meanError);
  ASSERT_EQ(b(5),-meanError);
  ASSERT_EQ(b(6),-meanError);
  ASSERT_EQ(b(7),-meanError);
  float mean_diff_dx = 0;
  float mean_diff_dy = 0;
  for(unsigned int l=0;l<nLevels_;l++){
    for(unsigned int i=0;i<patchSize_;i++){
      for(unsigned int j=0;j<patchSize_;j++){
        mean_diff_dx += -pow(0.5,l)*mp_.patches_[l].dx_[i*patchSize_+j];
        mean_diff_dy += -pow(0.5,l)*mp_.patches_[l].dy_[i*patchSize_+j];
      }
    }
  }
  mean_diff_dx /= nLevels_*patchSize_*patchSize_;
  mean_diff_dy /= nLevels_*patchSize_*patchSize_;
  for(unsigned int l=0;l<nLevels_;l++){
    for(unsigned int i=0;i<patchSize_;i++){
      for(unsigned int j=0;j<patchSize_;j++){
        ASSERT_EQ(A(4*l+i*patchSize_+j,0),-pow(0.5,l)*mp_.patches_[l].dx_[i*patchSize_+j]-mean_diff_dx);
        ASSERT_EQ(A(4*l+i*patchSize_+j,1),-pow(0.5,l)*mp_.patches_[l].dy_[i*patchSize_+j]-mean_diff_dy);
      }
    }
  }
}

// Test levelTranformCoordinates and computation of pyramid centers
TEST_F(MLPTesting, levelTranformCoordinates) {
  const int nLevels = 4;
  ImagePyramid<nLevels> pyr;
  FeatureCoordinates c2;
  c_.set_c(cv::Point2f(1,1));
  img1_ = cv::Mat::zeros(16,16,CV_8UC1);
  pyr.computeFromImage(img1_);
  ASSERT_EQ(pyr.centers_[0].x,0.0);
  ASSERT_EQ(pyr.centers_[0].y,0.0);
  ASSERT_EQ(pyr.centers_[1].x,0.0);
  ASSERT_EQ(pyr.centers_[1].y,0.0);
  ASSERT_EQ(pyr.centers_[2].x,0.0);
  ASSERT_EQ(pyr.centers_[2].y,0.0);
  ASSERT_EQ(pyr.centers_[3].x,0.0);
  ASSERT_EQ(pyr.centers_[3].y,0.0);
  for(int l1=0;l1<nLevels;l1++){
    for(int l2=0;l2<nLevels;l2++){
      pyr.levelTranformCoordinates(c_,c2,l1,l2);
      ASSERT_EQ(c2.get_c().x,pow(2.0,l1-l2));
      ASSERT_EQ(c2.get_c().y,pow(2.0,l1-l2));
    }
  }
  img1_ = cv::Mat::zeros(15,10,CV_8UC1);
  pyr.computeFromImage(img1_);
  ASSERT_EQ(pyr.centers_[0].x,0.0);
  ASSERT_EQ(pyr.centers_[0].y,0.0);
  ASSERT_EQ(pyr.centers_[1].x,-0.5);
  ASSERT_EQ(pyr.centers_[1].y,0.0);
  ASSERT_EQ(pyr.centers_[2].x,-1.5);
  ASSERT_EQ(pyr.centers_[2].y,-1.0);
  ASSERT_EQ(pyr.centers_[3].x,-3.5);
  ASSERT_EQ(pyr.centers_[3].y,-1.0);
  for(int l1=0;l1<nLevels;l1++){
    for(int l2=0;l2<nLevels;l2++){
      pyr.levelTranformCoordinates(c_,c2,l1,l2);
      ASSERT_EQ(c2.get_c().x,pow(2.0,l1-l2)+(pyr.centers_[l1].x-pyr.centers_[l2].x)*pow(0.5,l2));
      ASSERT_EQ(c2.get_c().y,pow(2.0,l1-l2)+(pyr.centers_[l1].y-pyr.centers_[l2].y)*pow(0.5,l2));
    }
  }
}

// Test align2D_old
TEST_F(MLPTesting, align2D_old) {
  FeatureCoordinates cAligned;
  c_.set_c(cv::Point2f(imgSize_/2,imgSize_/2));
  mp_.extractMultilevelPatchFromImage(pyr2_,c_,nLevels_-1,nullptr,true);
  ASSERT_EQ(mpa_.align2D_old(cAligned,pyr2_,mp_,c_,nullptr,0,nLevels_-1,100,1e-4),true);
  ASSERT_NEAR(cAligned.get_c().x,imgSize_/2,1e-2);
  ASSERT_NEAR(cAligned.get_c().y,imgSize_/2,1e-2);
  c_.set_c(cv::Point2f(imgSize_/2+1,imgSize_/2+1));
  ASSERT_EQ(mpa_.align2D_old(cAligned,pyr2_,mp_,c_,nullptr,0,nLevels_-1,100,1e-4),true);
  ASSERT_NEAR(cAligned.get_c().x,imgSize_/2,1e-2);
  ASSERT_NEAR(cAligned.get_c().y,imgSize_/2,1e-2);
  ASSERT_EQ(mpa_.align2D_old(cAligned,pyr2_,mp_,c_,nullptr,0,0,100,1e-4),true);
  ASSERT_NEAR(cAligned.get_c().x,imgSize_/2,1e-2);
  ASSERT_NEAR(cAligned.get_c().y,imgSize_/2,1e-2);
  ASSERT_EQ(mpa_.align2D_old(cAligned,pyr2_,mp_,c_,nullptr,1,1,100,1e-4),false);

  Eigen::Matrix2f aff;
  aff << cos(M_PI/2.0), -sin(M_PI/2.0), sin(M_PI/2.0), cos(M_PI/2.0);
  warp_.set_affineTransfrom(aff);
  c_.set_c(cv::Point2f(imgSize_/2,imgSize_/2));
  mp_.extractMultilevelPatchFromImage(pyr2_,c_,nLevels_-1,&warp_,true);
  ASSERT_EQ(mpa_.align2D_old(cAligned,pyr2_,mp_,c_,&warp_,0,nLevels_-1,100,1e-4),true);
  ASSERT_NEAR(cAligned.get_c().x,imgSize_/2,1e-2);
  ASSERT_NEAR(cAligned.get_c().y,imgSize_/2,1e-2);
  c_.set_c(cv::Point2f(imgSize_/2+1,imgSize_/2+1));
  ASSERT_EQ(mpa_.align2D_old(cAligned,pyr2_,mp_,c_,&warp_,0,nLevels_-1,100,1e-4),true);
  ASSERT_NEAR(cAligned.get_c().x,imgSize_/2,1e-2);
  ASSERT_NEAR(cAligned.get_c().y,imgSize_/2,1e-2);
  ASSERT_EQ(mpa_.align2D_old(cAligned,pyr2_,mp_,c_,&warp_,0,0,100,1e-4),true);
  ASSERT_NEAR(cAligned.get_c().x,imgSize_/2,1e-2);
  ASSERT_NEAR(cAligned.get_c().y,imgSize_/2,1e-2);
  ASSERT_EQ(mpa_.align2D_old(cAligned,pyr2_,mp_,c_,&warp_,1,1,100,1e-4),false);
}

// Test align2D
TEST_F(MLPTesting, align2D) {
  FeatureCoordinates cAligned;
  c_.set_c(cv::Point2f(imgSize_/2,imgSize_/2));
  mp_.extractMultilevelPatchFromImage(pyr2_,c_,nLevels_-1,nullptr,true);
  ASSERT_EQ(mpa_.align2D(cAligned,pyr2_,mp_,c_,nullptr,0,nLevels_-1,100,1e-4),true);
  ASSERT_NEAR(cAligned.get_c().x,imgSize_/2,1e-2);
  ASSERT_NEAR(cAligned.get_c().y,imgSize_/2,1e-2);
  c_.set_c(cv::Point2f(imgSize_/2+1,imgSize_/2+1));
  ASSERT_EQ(mpa_.align2D(cAligned,pyr2_,mp_,c_,nullptr,0,nLevels_-1,100,1e-4),true);
  ASSERT_NEAR(cAligned.get_c().x,imgSize_/2,1e-2);
  ASSERT_NEAR(cAligned.get_c().y,imgSize_/2,1e-2);
  ASSERT_EQ(mpa_.align2D(cAligned,pyr2_,mp_,c_,nullptr,0,0,100,1e-4),true);
  ASSERT_NEAR(cAligned.get_c().x,imgSize_/2,1e-2);
  ASSERT_NEAR(cAligned.get_c().y,imgSize_/2,1e-2);
  ASSERT_EQ(mpa_.align2D(cAligned,pyr2_,mp_,c_,nullptr,1,1,100,1e-4),false);

  Eigen::Matrix2f aff;
  aff << cos(M_PI/2.0), -sin(M_PI/2.0), sin(M_PI/2.0), cos(M_PI/2.0);
  warp_.set_affineTransfrom(aff);
  c_.set_c(cv::Point2f(imgSize_/2,imgSize_/2));
  mp_.extractMultilevelPatchFromImage(pyr2_,c_,nLevels_-1,&warp_,true);
  ASSERT_EQ(mpa_.align2D(cAligned,pyr2_,mp_,c_,&warp_,0,nLevels_-1,100,1e-4),true);
  ASSERT_NEAR(cAligned.get_c().x,imgSize_/2,1e-2);
  ASSERT_NEAR(cAligned.get_c().y,imgSize_/2,1e-2);
  c_.set_c(cv::Point2f(imgSize_/2+1,imgSize_/2+1));
  ASSERT_EQ(mpa_.align2D(cAligned,pyr2_,mp_,c_,&warp_,0,nLevels_-1,100,1e-4),true);
  ASSERT_NEAR(cAligned.get_c().x,imgSize_/2,1e-2);
  ASSERT_NEAR(cAligned.get_c().y,imgSize_/2,1e-2);
  ASSERT_EQ(mpa_.align2D(cAligned,pyr2_,mp_,c_,&warp_,0,0,100,1e-4),true);
  ASSERT_NEAR(cAligned.get_c().x,imgSize_/2,1e-2);
  ASSERT_NEAR(cAligned.get_c().y,imgSize_/2,1e-2);
  ASSERT_EQ(mpa_.align2D(cAligned,pyr2_,mp_,c_,&warp_,0,1,100,1e-4),true);

  // Single step comparison
  cv::Point2f c1,c2;
  c_.set_c(cv::Point2f(imgSize_/2,imgSize_/2));
  mp_.extractMultilevelPatchFromImage(pyr2_,c_,nLevels_-1,nullptr,true);
  c_.set_c(cv::Point2f(imgSize_/2+1,imgSize_/2+1));
  mpa_.align2D(cAligned,pyr2_,mp_,c_,nullptr,0,nLevels_-1,1,1e-4);
  c1 = cAligned.get_c();
  mpa_.align2D_old(cAligned,pyr2_,mp_,c_,nullptr,0,nLevels_-1,1,1e-4);
  c2 = cAligned.get_c();
  ASSERT_NEAR(c1.x,c2.x,1e-6);
  ASSERT_NEAR(c1.y,c2.y,1e-6);
  mpa_.align2D(cAligned,pyr2_,mp_,c_,nullptr,0,0,1,1e-4);
  c1 = cAligned.get_c();
  mpa_.align2D_old(cAligned,pyr2_,mp_,c_,nullptr,0,0,1,1e-4);
  c2 = cAligned.get_c();
  ASSERT_NEAR(c1.x,c2.x,1e-6);
  ASSERT_NEAR(c1.y,c2.y,1e-6);
  mpa_.align2D(cAligned,pyr2_,mp_,c_,nullptr,1,1,1,1e-4);
  c1 = cAligned.get_c();
  mpa_.align2D_old(cAligned,pyr2_,mp_,c_,nullptr,1,1,1,1e-4);
  c2 = cAligned.get_c();
  ASSERT_NEAR(c1.x,c2.x,1e-6);
  ASSERT_NEAR(c1.y,c2.y,1e-6);

  aff << cos(M_PI/2.0), -sin(M_PI/2.0), sin(M_PI/2.0), cos(M_PI/2.0);
  warp_.set_affineTransfrom(aff);
  c_.set_c(cv::Point2f(imgSize_/2,imgSize_/2));
  mp_.extractMultilevelPatchFromImage(pyr2_,c_,nLevels_-1,&warp_,true);
  c_.set_c(cv::Point2f(imgSize_/2+1,imgSize_/2+1));
  mpa_.align2D(cAligned,pyr2_,mp_,c_,&warp_,0,nLevels_-1,1,1e-4);
  c1 = cAligned.get_c();
  mpa_.align2D_old(cAligned,pyr2_,mp_,c_,&warp_,0,nLevels_-1,1,1e-4);
  c2 = cAligned.get_c();
  ASSERT_NEAR(c1.x,c2.x,1e-6);
  ASSERT_NEAR(c1.y,c2.y,1e-6);
  mpa_.align2D(cAligned,pyr2_,mp_,c_,&warp_,0,0,1,1e-4);
  c1 = cAligned.get_c();
  mpa_.align2D_old(cAligned,pyr2_,mp_,c_,&warp_,0,0,1,1e-4);
  c2 = cAligned.get_c();
  ASSERT_NEAR(c1.x,c2.x,1e-6);
  ASSERT_NEAR(c1.y,c2.y,1e-6);
  mpa_.align2D(cAligned,pyr2_,mp_,c_,&warp_,1,1,1,1e-4);
  c1 = cAligned.get_c();
  mpa_.align2D_old(cAligned,pyr2_,mp_,c_,&warp_,1,1,1,1e-4);
  c2 = cAligned.get_c();
  ASSERT_NEAR(c1.x,c2.x,1e-6);
  ASSERT_NEAR(c1.y,c2.y,1e-6);
}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
