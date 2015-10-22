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

#ifndef FEATUREWARPING_HPP_
#define FEATUREWARPING_HPP_

#include "lightweight_filtering/common.hpp"
#include "lightweight_filtering/State.hpp"
#include <opencv2/features2d/features2d.hpp>
#include "rovio/FeatureCoordinates.hpp"

namespace rovio{

/** \brief Vectors, pointing from the feature image location to the midpoints of the patch edges (expressed in pixels).
 *
 *  \see BearingCorners
 */
struct PixelCorners{
  cv::Point2f corners_[2];
  cv::Point2f& operator[](int i){
    assert(i>=0 && i<2);
    return corners_[i];
  };
  const cv::Point2f& operator[](int i) const{
    assert(i>=0 && i<2);
    return corners_[i];
  };
  void setIdentity(){
    corners_[0].x = 1;
    corners_[1].x = 0;
    corners_[0].y = 0;
    corners_[1].y = 1;
  }
};

/** \brief Difference (see boxminus for bearing vectors) between feature bearing vector and bearing vectors
 *  of the midpoints of the corresponding patch edges.
 *
 *  \see PixelCorners
 */
struct BearingCorners{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector2d corners_[2];
  Eigen::Vector2d& operator[](int i){
    assert(i>=0 && i<2);
    return corners_[i];
  };
  const Eigen::Vector2d& operator[](int i) const{
    assert(i>=0 && i<2);
    return corners_[i];
  };
};

/** \brief Coordinates of the four patch corners.
 */
struct PatchCorners{
  FeatureCoordinates f_[4];
  FeatureCoordinates& operator()(int x,int y){
    assert(x>=0 && x<2);
    assert(y>=0 && y<2);
    return f_[2*x+y];
  };
  const FeatureCoordinates& operator()(int x,int y) const{
    assert(x>=0 && x<2);
    assert(y>=0 && y<2);
    return f_[2*x+y];
  };
};

/** \brief Helper function for transforming a cv::Point2f to a Eigen::Vector2f
 *
 * @param p - cv::Point2f
 * @return Eigen::Vector2f
 */
static Eigen::Vector2f pointToVec2f(const cv::Point2f& p){
  return Eigen::Vector2f(p.x,p.y);
}


/** \brief Helper function for transforming a cv::Point2f to a Eigen::Vector2f
 *
 * @param v - Eigen::Vector2f
 * @return cv::Point2f
 */
static cv::Point2f vecToPoint2f(const Eigen::Vector2f& v){
  return cv::Point2f(v(0),v(1));
}

/** \brief Class taking care of tracking the feature warping
 */
class FeatureWarping{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  float warpDistance_; /**<Distance used for computing the affine matrix numerically.*/
  mutable PixelCorners pixelCorners_;  /**<Vectors, pointing from the feature coordinates to the midpoint of the patch edges.*/
  mutable bool valid_pixelCorners_;  /**<Specifies if the current pixel corners \ref pixelCorners_ are valid.*/
  mutable BearingCorners bearingCorners_;  /**<Vectors, pointing from the feature's bearing vector to the bearing vectors
                                               belonging to the pixel coordinates of the midpoint of the patch edges.*/
  mutable bool valid_bearingCorners_;  /**<Specifies if the current bearing corners \ref bearingCorners_ are valid.*/
  mutable Eigen::Matrix2f affineTransform_;  /**<Affine transformation matrix from current patch to the current frame. */
  mutable bool valid_affineTransform_;  /**<Specifies if the affine transformation \ref affineTransform_ is valid.*/
  bool isIdentity_; /**<Is the warping equal to identity.*/
  mutable PatchCorners patchCornersTemp_; /**<Four corners of patch.*/
  mutable LWF::NormalVectorElement norTemp_;

  /** \brief Constructor
   */
  FeatureWarping(const float warpDistance = 1.0): warpDistance_(warpDistance){
    reset();
  }

  /** \brief Resets the FeatureWarping.
   *
   */
  void reset(){
    affineTransform_.setIdentity();
    valid_pixelCorners_ = false;
    valid_bearingCorners_ = false;
    valid_affineTransform_ = true;
    isIdentity_ = true;
  }

  /** \brief Get the four corners of the MultilevelPatchFeature.
   *
   * Note: Validity can be checked with com_bearingCorners(mpCoordinates)
   * @param mpCoordinates - pointer to coordinates of center point
   * @param s - stretch factor
   * @return the four corners.
   */
  const PatchCorners& get_patchCorners(const FeatureCoordinates* mpCoordinates, float s = 1.0) const{
    assert(mpCoordinates != nullptr);
    get_bearingCorners(mpCoordinates);
    for(int x=0;x<2;x++){
      for(int y=0;y<2;y++){
        mpCoordinates->get_nor().boxPlus(((x*2-1)*bearingCorners_[0]+(y*2-1)*bearingCorners_[1])*s/warpDistance_,norTemp_);
        patchCornersTemp_(x,y).set_nor(norTemp_);
        patchCornersTemp_(x,y).mpCamera_ = mpCoordinates->mpCamera_;
        patchCornersTemp_(x,y).camID_ = mpCoordinates->camID_;
      }
    }
    return patchCornersTemp_;
  }

  /** \brief Get the PixelCorners of the MultilevelPatchFeature.
   *
   * @return Whether the computation was successfull.
   */
  bool com_pixelCorners(const FeatureCoordinates* mpCoordinates) const{
    if(!valid_pixelCorners_){
      if(valid_affineTransform_) {
        for(unsigned int i=0;i<2;i++){
          pixelCorners_[i].x = affineTransform_(0,i)*warpDistance_;  // Parallelism is preserved in an affine transformation!
          pixelCorners_[i].y = affineTransform_(1,i)*warpDistance_;  // Parallelism is preserved in an affine transformation!
        }
        valid_pixelCorners_ = true;
      } else if(valid_bearingCorners_){
        assert(mpCoordinates != nullptr);
        assert(mpCoordinates->mpCamera_ != nullptr);
        cv::Point2f tempPixel;
        LWF::NormalVectorElement tempNormal;
        if(mpCoordinates->com_nor() && mpCoordinates->com_c()){
          valid_pixelCorners_ = true;
          for(unsigned int i=0;i<2;i++){
            mpCoordinates->get_nor().boxPlus(bearingCorners_[i],tempNormal);
            if(!mpCoordinates->mpCamera_->bearingToPixel(tempNormal,tempPixel)){
              valid_pixelCorners_ = false;
              break;
            }
            pixelCorners_[i] = tempPixel - mpCoordinates->get_c();
          }
        }
      }
    }
    return valid_pixelCorners_;
  }

  /** \brief Get the PixelCorners of the MultilevelPatchFeature.
   *
   * @return the valid PixelCorners.
   */
  const PixelCorners& get_pixelCorners(const FeatureCoordinates* mpCoordinates) const{
    if(!com_pixelCorners(mpCoordinates)){
      std::cout << "    \033[31mERROR: No valid corner data!\033[0m" << std::endl;
    }
    return pixelCorners_;
  }

  /** \brief Computes the BearingCorners of the MultilevelPatchFeature.
   *
   * @return Whether the computation was successfull.
   */
  bool com_bearingCorners(const FeatureCoordinates* mpCoordinates) const{
    if(!valid_bearingCorners_){
      assert(mpCoordinates != nullptr);
      assert(mpCoordinates->mpCamera_ != nullptr);
      cv::Point2f tempPixel;
      LWF::NormalVectorElement tempNormal;
      if(com_pixelCorners(mpCoordinates) && mpCoordinates->com_nor() && mpCoordinates->com_c()){
        valid_bearingCorners_ = true;
        for(unsigned int i=0;i<2;i++){
          tempPixel = mpCoordinates->get_c()+pixelCorners_[i];
          if(!mpCoordinates->mpCamera_->pixelToBearing(tempPixel,tempNormal)){
            valid_bearingCorners_ = false;
            break;
          }
          tempNormal.boxMinus(mpCoordinates->get_nor(),bearingCorners_[i]);
        }
      }
    }
    return valid_bearingCorners_;
  }

  /** \brief Get the BearingCorners of the MultilevelPatchFeature.
   *
   * @return the valid BearingCorners.
   */
  const BearingCorners& get_bearingCorners(const FeatureCoordinates* mpCoordinates) const{
    if(!com_bearingCorners(mpCoordinates)){
      std::cout << "    \033[31mERROR: No valid corner data!\033[0m" << std::endl;
    }
    return bearingCorners_;
  }

  /** \brief Computes the affine transformation of the current patch, if not available computes it from the
   *  current pixelCorners or bearingCorners.
   *
   * @return Whether the computation was successfull.
   */
  bool com_affineTransform(const FeatureCoordinates* mpCoordinates) const{
    if(!valid_affineTransform_){
      if(com_pixelCorners(mpCoordinates)){
        for(unsigned int i=0;i<2;i++){
          affineTransform_(0,i) = pixelCorners_[i].x/warpDistance_;
          affineTransform_(1,i) = pixelCorners_[i].y/warpDistance_;
        }
        valid_affineTransform_ = true;
      }
    }
    return valid_affineTransform_;
  }

  /** \brief Get the affine transformation of the current patch.
   *
   * @return the affine transformation matrix \ref affineTransform_.
   */
  const Eigen::Matrix2f& get_affineTransform(const FeatureCoordinates* mpCoordinates) const{
    if(!com_affineTransform(mpCoordinates)){
      std::cout << "    \033[31mERROR: No valid corner data!\033[0m" << std::endl;
    }
    return affineTransform_;
  }

  /** \brief Set the \ref pixelCorners_ of the MultilevelPatchFeature.
   *
   * Note: The validity of the bearingCorners_ and of the affineTransform_ is set to invalid
   *       (\ref valid_bearingCorners_ = false; \ref valid_affineTransform_ = false).
   * \see get_affineTransform()
   * @param pixelCorners
   */
  void set_pixelCorners(const PixelCorners& pixelCorners){
    pixelCorners_ = pixelCorners;
    valid_pixelCorners_ = true;
    valid_bearingCorners_ = false;
    valid_affineTransform_ = false;
    isIdentity_ = false;
  }

  /** \brief Set the \ref bearingCorners_ of the MultilevelPatchFeature.
   *
   * Note: The validity of the pixelCorners_ and of the affineTransform_ is set to invalid
   *       (\ref valid_pixelCorners_ = false; \ref valid_affineTransform_ = false).
   * @param bearingCorners
   */
  void set_bearingCorners(const BearingCorners& bearingCorners){
    bearingCorners_ = bearingCorners;
    valid_bearingCorners_ = true;
    valid_pixelCorners_ = false;
    valid_affineTransform_ = false;
    isIdentity_ = false;
  }

  /** \brief Set the affine transformation \ref affineTransform_ of the MultilevelPatchFeature.
   *
   * Note: The validity of the pixelCorners_ and of the bearingCorners_ is set to invalid
   *       (\ref valid_pixelCorners_ = false; \ref bearingCorners_ = false).
   * @param affineTransform - Affine transformation matrix between patch and current image.
   */
  void set_affineTransfrom(const Eigen::Matrix2f& affineTransform){
    affineTransform_ = affineTransform;
    valid_affineTransform_ = true;
    valid_bearingCorners_ = false;
    valid_pixelCorners_ = false;
    isIdentity_ = false;
  }

};

}


#endif /* FEATUREWARPING_HPP_ */
