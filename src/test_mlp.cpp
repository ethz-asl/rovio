#include "rovio/Camera.hpp"
#include "gtest/gtest.h"
#include <assert.h>

#include "../include/rovio/commonVision.hpp"

using namespace rovio;

class MLPTesting : public virtual ::testing::Test {
 protected:
  static const int nLevels_ = 2;
  static const int patchSize_ = 2;
  static const int imgSize_ = (patchSize_+2)*pow(2,nLevels_-1)+4;
  static const int nMax_ = 20;
  static const int dx_ = 2;
  static const int dy_ = 3;

  ImagePyramid<nLevels_> pyr1_;
  ImagePyramid<nLevels_> pyr2_;
  cv::Mat img1_;
  cv::Mat img2_;
  Patch<patchSize_> p_;
  MultilevelPatchFeature<nLevels_,patchSize_> mlp_;
  cv::Point2f c_;
  LWF::NormalVectorElement nor_;
  PixelCorners pixelCorners_;
  BearingCorners bearingCorners_;
  Eigen::Matrix2f affineTransform_;
  Camera camera_;
  MLPTesting(): mlp_(&camera_){
    static_assert(imgSize_*dx_*dy_<255,"imgSize to large for gradients");
    c_ = cv::Point2f(patchSize_/2+1,patchSize_/2+1);
    camera_.pixelToBearing(c_,nor_);
    affineTransform_.setIdentity();
    pixelCorners_[0] = cv::Point2f(mlp_.warpDistance_,0);
    pixelCorners_[1] = cv::Point2f(0,mlp_.warpDistance_);
    cv::Point2f cTemp;
    LWF::NormalVectorElement norTemp;
    cTemp = c_ + pixelCorners_[0];
    camera_.pixelToBearing(cTemp,norTemp);
    norTemp.boxMinus(nor_,bearingCorners_[0]);
    cTemp = c_ + pixelCorners_[1];
    camera_.pixelToBearing(cTemp,norTemp);
    norTemp.boxMinus(nor_,bearingCorners_[1]);

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
  MultilevelPatchFeature<nLevels_,patchSize_> mlp(&camera_);
  ASSERT_EQ(mlp.idx_,-1);
  ASSERT_EQ(mlp.initTime_,0.0);
  ASSERT_EQ(mlp.currentTime_,0.0);
  ASSERT_EQ(mlp.c_,cv::Point2f(0,0));
  ASSERT_EQ(mlp.valid_c_,false);
  ASSERT_EQ(mlp.valid_nor_,false);
  ASSERT_EQ(mlp.valid_pixelCorners_,false);
  ASSERT_EQ(mlp.valid_bearingCorners_,false);
  ASSERT_EQ(mlp.valid_affineTransform_,false);
  ASSERT_EQ(mlp.s_,0.0);
  ASSERT_EQ(mlp.totCount_,0);
  ASSERT_EQ(mlp.inFrameCount_,0);
  ASSERT_EQ(mlp.statistics_.size(),0);
  ASSERT_EQ(mlp.cumulativeMatchingStatus_.size(),0);
  ASSERT_EQ(mlp.cumulativeTrackingStatus_.size(),0);
  ASSERT_EQ(mlp.status_.inFrame_,false);
  ASSERT_EQ(mlp.status_.matchingStatus_,(NOTMATCHED));
  ASSERT_EQ(mlp.status_.trackingStatus_,(NOTTRACKED));
  for(unsigned int i = 0;i<nLevels_;i++){
    ASSERT_EQ(mlp.isValidPatch_[i],false);
  }
}

// Test coordinates getters and setters
TEST_F(MLPTesting, coordinates) {
  mlp_.set_c(c_);
  ASSERT_EQ(mlp_.valid_c_,true);
  ASSERT_EQ(mlp_.valid_nor_,false);
  ASSERT_EQ(mlp_.get_c(),c_);
  ASSERT_EQ(mlp_.get_nor().getVec(),nor_.getVec());
  ASSERT_EQ(mlp_.valid_nor_,true);
  nor_.boxPlus(Eigen::Vector2d(0.001,0.002),nor_);
  camera_.bearingToPixel(nor_,c_);
  mlp_.set_nor(nor_);
  ASSERT_EQ(mlp_.valid_c_,false);
  ASSERT_EQ(mlp_.valid_nor_,true);
  ASSERT_EQ(mlp_.get_nor().getVec(),nor_.getVec());
  ASSERT_EQ(mlp_.get_c(),c_);
  ASSERT_EQ(mlp_.valid_c_,true);
}

// Test corner getters and setters
TEST_F(MLPTesting, corner) {
  mlp_.set_c(c_);
  mlp_.pixelCorners_[0] = cv::Point2f(0,0);
  mlp_.pixelCorners_[1] = cv::Point2f(0,0);
  mlp_.bearingCorners_[0].setZero();
  mlp_.bearingCorners_[1].setZero();
  mlp_.affineTransform_.setZero();
  mlp_.set_pixelCorners(pixelCorners_);
  ASSERT_EQ(mlp_.valid_pixelCorners_,true);
  ASSERT_EQ(mlp_.valid_bearingCorners_,false);
  ASSERT_EQ(mlp_.valid_affineTransform_,false);
  ASSERT_EQ(mlp_.get_pixelCorners()[0],pixelCorners_[0]);
  ASSERT_EQ(mlp_.get_pixelCorners()[1],pixelCorners_[1]);
  ASSERT_EQ(mlp_.valid_pixelCorners_,true);
  ASSERT_EQ(mlp_.valid_bearingCorners_,false);
  ASSERT_EQ(mlp_.valid_affineTransform_,false);
  ASSERT_EQ((mlp_.get_bearingCorners()[0]-bearingCorners_[0]).norm(),0.0);
  ASSERT_EQ((mlp_.get_bearingCorners()[1]-bearingCorners_[1]).norm(),0.0);
  ASSERT_EQ(mlp_.valid_bearingCorners_,true);
  ASSERT_EQ(mlp_.get_affineTransform(),affineTransform_);
  ASSERT_EQ(mlp_.valid_affineTransform_,true);
  mlp_.pixelCorners_[0] = cv::Point2f(0,0);
  mlp_.pixelCorners_[1] = cv::Point2f(0,0);
  mlp_.bearingCorners_[0].setZero();
  mlp_.bearingCorners_[1].setZero();
  mlp_.affineTransform_.setZero();
  mlp_.set_bearingCorners(bearingCorners_);
  ASSERT_EQ(mlp_.valid_pixelCorners_,false);
  ASSERT_EQ(mlp_.valid_bearingCorners_,true);
  ASSERT_EQ(mlp_.valid_affineTransform_,false);
  ASSERT_EQ((mlp_.get_bearingCorners()[0]-bearingCorners_[0]).norm(),0.0);
  ASSERT_EQ((mlp_.get_bearingCorners()[1]-bearingCorners_[1]).norm(),0.0);
  ASSERT_EQ(mlp_.valid_pixelCorners_,false);
  ASSERT_EQ(mlp_.valid_bearingCorners_,true);
  ASSERT_EQ(mlp_.valid_affineTransform_,false);
  ASSERT_EQ(mlp_.get_pixelCorners()[0],pixelCorners_[0]);
  ASSERT_EQ(mlp_.get_pixelCorners()[1],pixelCorners_[1]);
  ASSERT_EQ(mlp_.valid_pixelCorners_,true);
  ASSERT_EQ(mlp_.get_affineTransform(),affineTransform_);
  ASSERT_EQ(mlp_.valid_affineTransform_,true);
  mlp_.pixelCorners_[0] = cv::Point2f(0,0);
  mlp_.pixelCorners_[1] = cv::Point2f(0,0);
  mlp_.bearingCorners_[0].setZero();
  mlp_.bearingCorners_[1].setZero();
  mlp_.affineTransform_.setZero();
  mlp_.set_affineTransfrom(affineTransform_);
  ASSERT_EQ(mlp_.valid_pixelCorners_,false);
  ASSERT_EQ(mlp_.valid_bearingCorners_,false);
  ASSERT_EQ(mlp_.valid_affineTransform_,true);
  ASSERT_EQ(mlp_.get_affineTransform(),affineTransform_);
  ASSERT_EQ(mlp_.valid_pixelCorners_,false);
  ASSERT_EQ(mlp_.valid_bearingCorners_,false);
  ASSERT_EQ(mlp_.valid_affineTransform_,true);
  ASSERT_EQ(mlp_.get_pixelCorners()[0],pixelCorners_[0]);
  ASSERT_EQ(mlp_.get_pixelCorners()[1],pixelCorners_[1]);
  ASSERT_EQ(mlp_.valid_pixelCorners_,true);
  ASSERT_EQ((mlp_.get_bearingCorners()[0]-bearingCorners_[0]).norm(),0.0);
  ASSERT_EQ((mlp_.get_bearingCorners()[1]-bearingCorners_[1]).norm(),0.0);
  ASSERT_EQ(mlp_.valid_bearingCorners_,true);
}

// Test statistics
TEST_F(MLPTesting, statistics) {
  Status status1;
  status1.inFrame_ = true;
  status1.trackingStatus_ = TRACKED;
  Status status2;
  status2.inFrame_ = true;
  status2.trackingStatus_ = FAILED;
  ASSERT_EQ(mlp_.countMatchingStatistics(NOTMATCHED),1);
  ASSERT_EQ(mlp_.countMatchingStatistics(NOTFOUND),0);
  ASSERT_EQ(mlp_.countMatchingStatistics(FOUND),0);
  ASSERT_EQ(mlp_.countTrackingStatistics(NOTTRACKED),1);
  ASSERT_EQ(mlp_.countTrackingStatistics(FAILED),0);
  ASSERT_EQ(mlp_.countTrackingStatistics(TRACKED),0);
  ASSERT_EQ(mlp_.countTot(),1);
  ASSERT_EQ(mlp_.countTotInFrame(),0);
  mlp_.increaseStatistics(0.1);
  ASSERT_EQ(mlp_.countMatchingStatistics(NOTMATCHED),2);
  ASSERT_EQ(mlp_.countMatchingStatistics(NOTFOUND),0);
  ASSERT_EQ(mlp_.countMatchingStatistics(FOUND),0);
  ASSERT_EQ(mlp_.countTrackingStatistics(NOTTRACKED),2);
  ASSERT_EQ(mlp_.countTrackingStatistics(FAILED),0);
  ASSERT_EQ(mlp_.countTrackingStatistics(TRACKED),0);
  ASSERT_EQ(mlp_.countTot(),2);
  ASSERT_EQ(mlp_.countTotInFrame(),0);
  mlp_.increaseStatistics(0.2);
  mlp_.status_ = status1;
  mlp_.increaseStatistics(0.3);
  mlp_.status_ = status1;
  mlp_.increaseStatistics(0.4);
  mlp_.status_ = status1;
  mlp_.increaseStatistics(0.5);
  mlp_.status_ = status2;
  mlp_.increaseStatistics(0.6);
  mlp_.status_ = status2;
  mlp_.increaseStatistics(0.7);
  mlp_.status_ = status1;
  mlp_.increaseStatistics(0.8);
  mlp_.status_ = status1;
  mlp_.increaseStatistics(0.9);
  mlp_.status_ = status1;
  mlp_.increaseStatistics(1.0);
  ASSERT_EQ(mlp_.countMatchingStatistics(NOTMATCHED),11);
  ASSERT_EQ(mlp_.countMatchingStatistics(NOTFOUND),0);
  ASSERT_EQ(mlp_.countMatchingStatistics(FOUND),0);
  ASSERT_EQ(mlp_.countTrackingStatistics(NOTTRACKED),3);
  ASSERT_EQ(mlp_.countTrackingStatistics(FAILED),2);
  ASSERT_EQ(mlp_.countTrackingStatistics(TRACKED),6);
  ASSERT_EQ(mlp_.countMatchingStatistics(NOTMATCHED,5),5);
  ASSERT_EQ(mlp_.countMatchingStatistics(NOTFOUND,5),0);
  ASSERT_EQ(mlp_.countMatchingStatistics(FOUND,5),0);
  ASSERT_EQ(mlp_.countTrackingStatistics(NOTTRACKED,5),1);
  ASSERT_EQ(mlp_.countTrackingStatistics(FAILED,5),1);
  ASSERT_EQ(mlp_.countTrackingStatistics(TRACKED,5),3);
  ASSERT_EQ(mlp_.getLocalQuality(5),0.6);
  ASSERT_EQ(mlp_.getLocalVisibilityQuality(5),0.8);
  ASSERT_EQ(mlp_.getGlobalQuality(5),6.0/11.0);
  ASSERT_EQ(mlp_.isGoodFeature(5,5,0.9,0.1),false);
}

// Test isMultilevelPatchInFrame
TEST_F(MLPTesting, isMultilevelPatchInFrame) {
  double c;
  for(unsigned int l=0;l<nLevels_;l++){
    c = patchSize_/2*pow(2,l)-0.1;
    mlp_.set_c(cv::Point2f(c,c));
    ASSERT_EQ(isMultilevelPatchInFrame(mlp_,pyr1_,l),false);
    c = patchSize_/2*pow(2,l);
    mlp_.set_c(cv::Point2f(c,c));
    ASSERT_EQ(isMultilevelPatchInFrame(mlp_,pyr1_,l),true);
    c = patchSize_/2*pow(2,l)+0.1;
    mlp_.set_c(cv::Point2f(c,c));
    ASSERT_EQ(isMultilevelPatchInFrame(mlp_,pyr1_,l),true);
    c = imgSize_-patchSize_/2*pow(2,l)-0.1;
    mlp_.set_c(cv::Point2f(c,c));
    ASSERT_EQ(isMultilevelPatchInFrame(mlp_,pyr1_,l),true);
    c = imgSize_-patchSize_/2*pow(2,l);
    mlp_.set_c(cv::Point2f(c,c));
    ASSERT_EQ(isMultilevelPatchInFrame(mlp_,pyr1_,l),true);
    c = imgSize_-patchSize_/2*pow(2,l)+0.1;
    mlp_.set_c(cv::Point2f(c,c));
    ASSERT_EQ(isMultilevelPatchInFrame(mlp_,pyr1_,l),false);
  }
  for(unsigned int l=0;l<nLevels_;l++){
    c = (patchSize_/2+1)*pow(2,l)-0.1;
    mlp_.set_c(cv::Point2f(c,c));
    ASSERT_EQ(isMultilevelPatchInFrame(mlp_,pyr1_,l,true),false);
    c = (patchSize_/2+1)*pow(2,l);
    mlp_.set_c(cv::Point2f(c,c));
    ASSERT_EQ(isMultilevelPatchInFrame(mlp_,pyr1_,l,true),true);
    c = (patchSize_/2+1)*pow(2,l)+0.1;
    mlp_.set_c(cv::Point2f(c,c));
    ASSERT_EQ(isMultilevelPatchInFrame(mlp_,pyr1_,l,true),true);
    c = imgSize_-(patchSize_/2+1)*pow(2,l)-0.1;
    mlp_.set_c(cv::Point2f(c,c));
    ASSERT_EQ(isMultilevelPatchInFrame(mlp_,pyr1_,l,true),true);
    c = imgSize_-(patchSize_/2+1)*pow(2,l);
    mlp_.set_c(cv::Point2f(c,c));
    ASSERT_EQ(isMultilevelPatchInFrame(mlp_,pyr1_,l,true),true);
    c = imgSize_-(patchSize_/2+1)*pow(2,l)+0.1;
    mlp_.set_c(cv::Point2f(c,c));
    ASSERT_EQ(isMultilevelPatchInFrame(mlp_,pyr1_,l,true),false);
  }
}

// Test extractMultilevelPatchFromImage (tests computeFromImage as well)
TEST_F(MLPTesting, extractMultilevelPatchFromImage) {
  pyr1_.computeFromImage(img1_,false);
  mlp_.set_c(cv::Point2f(imgSize_/2,imgSize_/2));
  extractMultilevelPatchFromImage(mlp_,pyr1_);
  for(unsigned int l=0;l<nLevels_;l++){
    ASSERT_EQ(mlp_.isValidPatch_[l],true);
    float* patch_ptr = mlp_.patches_[l].patchWithBorder_;
    for(int i=0;i<patchSize_+2;i++){
      for(int j=0;j<patchSize_+2;j++, ++patch_ptr){
        const cv::Point2f c(i*pow(2,l)+imgSize_/2-(patchSize_/2+1.0-0.5)*pow(2,l)-0.5,
                            j*pow(2,l)+imgSize_/2-(patchSize_/2+1.0-0.5)*pow(2,l)-0.5);
        ASSERT_EQ(*patch_ptr,(float)(uint8_t)(c.x*dy_+c.y*dx_)); // down-sampling of image rounds to float, thus the casting
      }
    }
  }
  ASSERT_EQ(mlp_.get_affineTransform(),Eigen::Matrix2f::Identity());

  pyr1_.computeFromImage(img1_,true);
  mlp_.set_c(cv::Point2f(imgSize_/2,imgSize_/2));
  extractMultilevelPatchFromImage(mlp_,pyr1_);
  for(unsigned int l=0;l<nLevels_;l++){
    ASSERT_EQ(mlp_.isValidPatch_[l],true);
    float* patch_ptr = mlp_.patches_[l].patch_; // Border effect, thus only valid for inner patch
    for(int i=0;i<patchSize_;i++){
      for(int j=0;j<patchSize_;j++, ++patch_ptr){
        const cv::Point2f c(i*pow(2,l)+imgSize_/2-(patchSize_/2-0.5)*pow(2,l)-0.5,
                            j*pow(2,l)+imgSize_/2-(patchSize_/2-0.5)*pow(2,l)-0.5);
        ASSERT_EQ(*patch_ptr,(float)(c.x*dy_+c.y*dx_)); // down-sampling hits integer thus no casting required
      }
    }
  }
  ASSERT_EQ(mlp_.get_affineTransform(),Eigen::Matrix2f::Identity());
}

// Test computeMultilevelShiTomasiScore
TEST_F(MLPTesting, computeMultilevelShiTomasiScore) {
  mlp_.set_c(cv::Point2f(imgSize_/2,imgSize_/2));
  extractMultilevelPatchFromImage(mlp_,pyr2_);
  double s = 0.5 * (patchSize_ + patchSize_ - sqrtf((patchSize_ + patchSize_) * (patchSize_ + patchSize_) - 4 * (patchSize_ * patchSize_ - 1 * 1)));
  s = s*(255*0.5)*(255*0.5)/(patchSize_*patchSize_);
  int count = 0;
  double scale = 0;
  for(unsigned int l=0;l<nLevels_;l++){
    mlp_.getSpecificLevelScore(l);
    ASSERT_EQ(mlp_.s_,s);
    count++;
    scale += pow(0.25,l);
  }
  scale /= count;
  mlp_.computeMultilevelShiTomasiScore();
  ASSERT_EQ(mlp_.s_,s*scale);
}

// Test getLinearAlignEquations
TEST_F(MLPTesting, getLinearAlignEquations) {
  Eigen::MatrixXf A;
  Eigen::MatrixXf b;
  b.setZero();
  mlp_.set_c(cv::Point2f(imgSize_/2,imgSize_/2));
  extractMultilevelPatchFromImage(mlp_,pyr2_);
  mlp_.set_c(cv::Point2f(imgSize_/2+1,imgSize_/2+1));
  // NOTE: only works for patch size = 2, nLevels = 2
  ASSERT_EQ(getLinearAlignEquations(mlp_,pyr2_,0,nLevels_-1,false,A,b),true);
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
        mean_diff_dx += -pow(0.5,l)*mlp_.patches_[l].dx_[i*patchSize_+j];
        mean_diff_dy += -pow(0.5,l)*mlp_.patches_[l].dy_[i*patchSize_+j];
      }
    }
  }
  mean_diff_dx /= nLevels_*patchSize_*patchSize_;
  mean_diff_dy /= nLevels_*patchSize_*patchSize_;
  for(unsigned int l=0;l<nLevels_;l++){
    for(unsigned int i=0;i<patchSize_;i++){
      for(unsigned int j=0;j<patchSize_;j++){
        ASSERT_EQ(A(4*l+i*patchSize_+j,0),-pow(0.5,l)*mlp_.patches_[l].dx_[i*patchSize_+j]-mean_diff_dx);
        ASSERT_EQ(A(4*l+i*patchSize_+j,1),-pow(0.5,l)*mlp_.patches_[l].dy_[i*patchSize_+j]-mean_diff_dy);
      }
    }
  }
}

// Test levelTranformCoordinates and computation of pyramid centers
TEST_F(MLPTesting, levelTranformCoordinates) {
  const int nLevels = 4;
  ImagePyramid<nLevels> pyr;
  cv::Point2f c1,c2;
  c2 = cv::Point2f(1,1);
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
      c1 = levelTranformCoordinates(c2,pyr,l1,l2);
      ASSERT_EQ(c1.x,pow(2.0,l1-l2));
      ASSERT_EQ(c1.y,pow(2.0,l1-l2));
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
      c1 = levelTranformCoordinates(c2,pyr,l1,l2);
      ASSERT_EQ(c1.x,pow(2.0,l1-l2)+(pyr.centers_[l1].x-pyr.centers_[l2].x)*pow(0.5,l2));
      ASSERT_EQ(c1.y,pow(2.0,l1-l2)+(pyr.centers_[l1].y-pyr.centers_[l2].y)*pow(0.5,l2));
    }
  }
}

// Test align2D_old
TEST_F(MLPTesting, align2D_old) {
  mlp_.set_c(cv::Point2f(imgSize_/2,imgSize_/2));
  extractMultilevelPatchFromImage(mlp_,pyr2_);
  mlp_.set_c(cv::Point2f(imgSize_/2+1,imgSize_/2+1));
  ASSERT_EQ(align2D_old(mlp_,pyr2_,0,nLevels_-1,false,100,1e-4),true);
  ASSERT_NEAR(mlp_.get_c().x,imgSize_/2,1e-2);
  ASSERT_NEAR(mlp_.get_c().y,imgSize_/2,1e-2);
  mlp_.set_c(cv::Point2f(imgSize_/2+1,imgSize_/2+1));
  ASSERT_EQ(align2D_old(mlp_,pyr2_,0,0,false,100,1e-4),true);
  ASSERT_NEAR(mlp_.get_c().x,imgSize_/2,1e-2);
  ASSERT_NEAR(mlp_.get_c().y,imgSize_/2,1e-2);
  mlp_.set_c(cv::Point2f(imgSize_/2+1,imgSize_/2+1));
  ASSERT_EQ(align2D_old(mlp_,pyr2_,1,1,false,100,1e-4),false);

  Eigen::Matrix2f aff;
  aff << cos(M_PI/2.0), -sin(M_PI/2.0), sin(M_PI/2.0), cos(M_PI/2.0);
  mlp_.set_affineTransfrom(aff);
  mlp_.set_c(cv::Point2f(imgSize_/2,imgSize_/2));
  extractMultilevelPatchFromImage(mlp_,pyr2_,nLevels_-1,true,true);
  mlp_.set_c(cv::Point2f(imgSize_/2+1,imgSize_/2+1));
  ASSERT_EQ(align2D_old(mlp_,pyr2_,0,nLevels_-1,true,100,1e-4),true);
  ASSERT_NEAR(mlp_.get_c().x,imgSize_/2,1e-2);
  ASSERT_NEAR(mlp_.get_c().y,imgSize_/2,1e-2);
  mlp_.set_c(cv::Point2f(imgSize_/2+1,imgSize_/2+1));
  ASSERT_EQ(align2D_old(mlp_,pyr2_,0,0,true,100,1e-4),true);
  ASSERT_NEAR(mlp_.get_c().x,imgSize_/2,1e-2);
  ASSERT_NEAR(mlp_.get_c().y,imgSize_/2,1e-2);
  mlp_.set_c(cv::Point2f(imgSize_/2+1,imgSize_/2+1));
  ASSERT_EQ(align2D_old(mlp_,pyr2_,1,1,true,100,1e-4),false);
}

// Test align2D
TEST_F(MLPTesting, align2D) {
  mlp_.set_c(cv::Point2f(imgSize_/2,imgSize_/2));
  extractMultilevelPatchFromImage(mlp_,pyr2_);
  mlp_.set_c(cv::Point2f(imgSize_/2+1,imgSize_/2+1));
  ASSERT_EQ(align2D(mlp_,pyr2_,0,nLevels_-1,false,100,1e-4),true);
  ASSERT_NEAR(mlp_.get_c().x,imgSize_/2,1e-2);
  ASSERT_NEAR(mlp_.get_c().y,imgSize_/2,1e-2);
  mlp_.set_c(cv::Point2f(imgSize_/2+1,imgSize_/2+1));
  ASSERT_EQ(align2D(mlp_,pyr2_,0,0,false,100,1e-4),true);
  ASSERT_NEAR(mlp_.get_c().x,imgSize_/2,1e-2);
  ASSERT_NEAR(mlp_.get_c().y,imgSize_/2,1e-2);
  mlp_.set_c(cv::Point2f(imgSize_/2+1,imgSize_/2+1));
  ASSERT_EQ(align2D(mlp_,pyr2_,1,1,false,100,1e-4),false);

  Eigen::Matrix2f aff;
  aff << cos(M_PI/2.0), -sin(M_PI/2.0), sin(M_PI/2.0), cos(M_PI/2.0);
  mlp_.set_affineTransfrom(aff);
  mlp_.set_c(cv::Point2f(imgSize_/2,imgSize_/2));
  extractMultilevelPatchFromImage(mlp_,pyr2_,nLevels_-1,true,true);
  mlp_.set_c(cv::Point2f(imgSize_/2+1,imgSize_/2+1));
  ASSERT_EQ(align2D(mlp_,pyr2_,0,nLevels_-1,true,100,1e-4),true);
  ASSERT_NEAR(mlp_.get_c().x,imgSize_/2,1e-2);
  ASSERT_NEAR(mlp_.get_c().y,imgSize_/2,1e-2);
  mlp_.set_c(cv::Point2f(imgSize_/2+1,imgSize_/2+1));
  ASSERT_EQ(align2D(mlp_,pyr2_,0,0,true,100,1e-4),true);
  ASSERT_NEAR(mlp_.get_c().x,imgSize_/2,1e-2);
  ASSERT_NEAR(mlp_.get_c().y,imgSize_/2,1e-2);
  mlp_.set_c(cv::Point2f(imgSize_/2+1,imgSize_/2+1));
  ASSERT_EQ(align2D(mlp_,pyr2_,1,1,true,100,1e-4),false);

  // Single step comparison
  cv::Point2f c1,c2;
  mlp_.set_c(cv::Point2f(imgSize_/2+1,imgSize_/2+1));
  align2D(mlp_,pyr2_,0,1,false,1,1e6);
  c1 = mlp_.get_c();
  mlp_.set_c(cv::Point2f(imgSize_/2+1,imgSize_/2+1));
  align2D_old(mlp_,pyr2_,0,1,false,1,1e6);
  c2 = mlp_.get_c();
  ASSERT_NEAR(c1.x,c2.x,1e-6);
  ASSERT_NEAR(c1.y,c2.y,1e-6);
  mlp_.set_c(cv::Point2f(imgSize_/2+1,imgSize_/2+1));
  align2D(mlp_,pyr2_,0,0,false,1,1e6);
  c1 = mlp_.get_c();
  mlp_.set_c(cv::Point2f(imgSize_/2+1,imgSize_/2+1));
  align2D_old(mlp_,pyr2_,0,0,false,1,1e6);
  c2 = mlp_.get_c();
  ASSERT_NEAR(c1.x,c2.x,1e-6);
  ASSERT_NEAR(c1.y,c2.y,1e-6);
  mlp_.set_c(cv::Point2f(imgSize_/2+1,imgSize_/2+1));
  align2D(mlp_,pyr2_,1,1,false,1,1e6);
  c1 = mlp_.get_c();
  mlp_.set_c(cv::Point2f(imgSize_/2+1,imgSize_/2+1));
  align2D_old(mlp_,pyr2_,1,1,false,1,1e6);
  c2 = mlp_.get_c();
  ASSERT_NEAR(c1.x,c2.x,1e-6);
  ASSERT_NEAR(c1.y,c2.y,1e-6);

  aff << cos(M_PI/2.0), -sin(M_PI/2.0), sin(M_PI/2.0), cos(M_PI/2.0);
  mlp_.set_affineTransfrom(aff);
  mlp_.set_c(cv::Point2f(imgSize_/2,imgSize_/2));
  extractMultilevelPatchFromImage(mlp_,pyr2_,nLevels_-1,true,true);
  mlp_.set_c(cv::Point2f(imgSize_/2+1,imgSize_/2+1));
  align2D(mlp_,pyr2_,0,1,false,1,1e6);
  c1 = mlp_.get_c();
  mlp_.set_c(cv::Point2f(imgSize_/2+1,imgSize_/2+1));
  align2D_old(mlp_,pyr2_,0,1,false,1,1e6);
  c2 = mlp_.get_c();
  ASSERT_NEAR(c1.x,c2.x,1e-6);
  ASSERT_NEAR(c1.y,c2.y,1e-6);
  mlp_.set_c(cv::Point2f(imgSize_/2+1,imgSize_/2+1));
  align2D(mlp_,pyr2_,0,0,false,1,1e6);
  c1 = mlp_.get_c();
  mlp_.set_c(cv::Point2f(imgSize_/2+1,imgSize_/2+1));
  align2D_old(mlp_,pyr2_,0,0,false,1,1e6);
  c2 = mlp_.get_c();
  ASSERT_NEAR(c1.x,c2.x,1e-6);
  ASSERT_NEAR(c1.y,c2.y,1e-6);
  mlp_.set_c(cv::Point2f(imgSize_/2+1,imgSize_/2+1));
  align2D(mlp_,pyr2_,1,1,false,1,1e6);
  c1 = mlp_.get_c();
  mlp_.set_c(cv::Point2f(imgSize_/2+1,imgSize_/2+1));
  align2D_old(mlp_,pyr2_,1,1,false,1,1e6);
  c2 = mlp_.get_c();
  ASSERT_NEAR(c1.x,c2.x,1e-6);
  ASSERT_NEAR(c1.y,c2.y,1e-6);
}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
