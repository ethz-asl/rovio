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

#include <Eigen/Dense>
#include <opencv2/features2d/features2d.hpp>
#include "lightweight_filtering/State.hpp"
#include "rovio/FeatureCoordinates.hpp"

namespace rovio{

/** \brief Vectors, pointing from the feature image location to the midpoints of the patch edges (expressed in pixels).
 *
 *  \see BearingCorners
 */
struct PixelCorners{
  cv::Point2f corners_[2];
  cv::Point2f& operator[](unsigned int i){
    return corners_[i];
  };
  const cv::Point2f& operator[](unsigned int i) const{
    return corners_[i];
  };
};

/** \brief Difference (see boxminus for bearing vectors) between feature bearing vector and bearing vectors
 *  of the midpoints of the corresponding patch edges.
 *
 *  \see PixelCorners
 */
struct BearingCorners{
  Eigen::Vector2d corners_[2];
  Eigen::Vector2d& operator[](unsigned int i){
    return corners_[i];
  };
  const Eigen::Vector2d& operator[](unsigned int i) const{
    return corners_[i];
  };
};

/** \brief Class taking care of tracking the feature warping
 */
class FeatureWarping{
 public:
  const Camera* mpCamera_;  /**<Pointer to the associated camera object.*/
  const FeatureCoordinates* mpCoordinates_;  /**<Pointer to the feature coordinates.*/
  const float warpDistance_; /**<Distance used for computing the affine matrix numerically.*/
  mutable PixelCorners pixelCorners_;  /**<Vectors, pointing from the feature coordinates to the midpoint of the patch edges.*/
  mutable bool valid_pixelCorners_;  /**<Specifies if the current pixel corners \ref pixelCorners_ are valid.*/
  mutable BearingCorners bearingCorners_;  /**<Vectors, pointing from the feature's bearing vector to the bearing vectors
                                               belonging to the pixel coordinates of the midpoint of the patch edges.*/
  mutable bool valid_bearingCorners_;  /**<Specifies if the current bearing corners \ref bearingCorners_ are valid.*/
  mutable Eigen::Matrix2f affineTransform_;  /**<Affine transformation matrix from current patch to the current frame. */
  mutable bool valid_affineTransform_;  /**<Specifies if the affine transformation \ref affineTransform_ is valid.*/

  /** \brief Constructor
   */
  FeatureWarping(const float warpDistance = 1.0,const Camera* mpCamera = nullptr, const FeatureCoordinates* mpCoordinates = nullptr): mpCamera_(mpCamera), mpCoordinates_(mpCoordinates), warpDistance_(warpDistance){
    reset();
  }

  /** \brief Resets the FeatureWarping.
   *
   */
  void reset(){
    valid_pixelCorners_ = false;
    valid_bearingCorners_ = false;
    valid_affineTransform_ = false;
  }


  /** \brief Get the PixelCorners of the MultilevelPatchFeature.
   *
   * @return the valid PixelCorners.
   */
  const PixelCorners& get_pixelCorners() const{
    if(!valid_pixelCorners_){
      if(valid_bearingCorners_){
        if(mpCamera_ == nullptr) throw cameraNullPtrException;
        cv::Point2f tempPixel;
        LWF::NormalVectorElement tempNormal;
        for(unsigned int i=0;i<2;i++){
          mpCoordinates_->get_nor().boxPlus(bearingCorners_[i],tempNormal);
          if(!mpCamera_->bearingToPixel(tempNormal,tempPixel)){
            std::cout << "ERROR: Problem during bearing corner to pixel mapping!" << std::endl;
          }
          pixelCorners_[i] = tempPixel - mpCoordinates_->get_c();
        }
        valid_pixelCorners_ = true;
      } else if(valid_affineTransform_) {
        for(unsigned int i=0;i<2;i++){
          pixelCorners_[i].x = affineTransform_(0,i)*warpDistance_;  // Parallelism is preserved in an affine transformation!
          pixelCorners_[i].y = affineTransform_(1,i)*warpDistance_;  // Parallelism is preserved in an affine transformation!
        }
        valid_pixelCorners_ = true;
      } else {
        std::cout << "ERROR: No valid corner data!" << std::endl;
      }
    }
    return pixelCorners_;
  }

  /** \brief Get the BearingCorners of the MultilevelPatchFeature.
   *
   * @return the valid BearingCorners.
   */
  const BearingCorners& get_bearingCorners() const{
    if(!valid_bearingCorners_){
      if(mpCamera_ == nullptr) throw cameraNullPtrException;
      cv::Point2f tempPixel;
      LWF::NormalVectorElement tempNormal;
      get_pixelCorners();
      for(unsigned int i=0;i<2;i++){
        tempPixel = mpCoordinates_->get_c()+pixelCorners_[i];
        if(!mpCamera_->pixelToBearing(tempPixel,tempNormal)){
          std::cout << "ERROR: Problem during pixel corner to bearing mapping!" << std::endl;
        }
        tempNormal.boxMinus(mpCoordinates_->get_nor(),bearingCorners_[i]);
      }
      valid_bearingCorners_ = true;
    }
    return bearingCorners_;
  }

  /** \brief Get the affine transformation of the current patch, if not available computes it from the
   *  current pixelCorners or bearingCorners.
   *
   * @return the affine transformation matrix \ref affineTransform_.
   */
  const Eigen::Matrix2f& get_affineTransform() const{
    if(!valid_affineTransform_){
      get_pixelCorners();
      for(unsigned int i=0;i<2;i++){
        affineTransform_(0,i) = pixelCorners_[i].x/warpDistance_;
        affineTransform_(1,i) = pixelCorners_[i].y/warpDistance_;
      }
      valid_affineTransform_ = true;
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
  }

};

}


#endif /* FEATUREWARPING_HPP_ */
