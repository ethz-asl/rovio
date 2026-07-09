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

#ifndef ROVIO_MULTILEVELPATCH_HPP_
#define ROVIO_MULTILEVELPATCH_HPP_

#include "rovio/Patch.hpp"
#include "rovio/FeatureCoordinates.hpp"
#include "rovio/ImagePyramid.hpp"

namespace rovio{

/** \brief A multilevel patch class.
 *
 *    @tparam nLevels   - Number of pyramid levels on which the feature is defined.
 *    @tparam patchSize - Edge length of the patches in pixels. Value must be a multiple of 2!.
 *                         Note: The patches edge length (in pixels) is the same for each patch, independently from the pyramid level.
 */
template<int nLevels,int patchSize>
class MultilevelPatch{
 public:
  static const int nLevels_ = nLevels;  /**<Number of pyramid levels on which the feature is defined.*/
  Patch<patchSize> patches_[nLevels_];  /**<Array, holding the patches on each pyramid level.*/
  bool isValidPatch_[nLevels_];  /**<Array, specifying if there is a valid patch stored at the corresponding location in \ref patches_.*/
  mutable Eigen::Matrix3f H_;  /**<Hessian matrix, corresponding to the multilevel patches.*/
  mutable float e0_;  /**<Smaller eigenvalue of H_.*/
  mutable float e1_;  /**<Larger eigenvalue of H_.*/
  mutable float s_;  /**<Shi-Tomasi score of the multilevel patch feature. @todo define and store method of computation*/

  /** Constructor
   */
  MultilevelPatch(){
    reset();
  }

  /** Destructor
   */
  virtual ~MultilevelPatch(){}

  /** \brief Resets the MultilevelPatch.
   *
   * @param idx - feature ID
   * @initTime  - Time at initialization.
   */
  void reset(){
    H_.setIdentity();
    e0_ = 0;
    e1_ = 0;
    s_ = 0;
    for(unsigned int i = 0;i<nLevels_;i++){
      isValidPatch_[i] = false;
    }
  }

  /** \brief Computes and sets the multilevel Shi-Tomasi Score \ref s_, considering a defined pyramid level interval.
   *
   * @param l1 - Start level (l1<l2)
   * @param l2 - End level (l1<l2)
   */
  void computeMultilevelShiTomasiScore(const int l1 = 0, const int l2 = nLevels_-1) const{
    H_.setZero();
    int count = 0;
    for(int i=l1;i<=l2;i++){
      if(isValidPatch_[i]){
        H_ += pow(0.25,i)*patches_[i].getHessian();
        count++;
      }
    }
    if(count > 0){
      const float dXX = H_(0,0)/(count*patchSize*patchSize);
      const float dYY = H_(1,1)/(count*patchSize*patchSize);
      const float dXY = H_(0,1)/(count*patchSize*patchSize);
      e0_ = 0.5 * (dXX + dYY - sqrtf((dXX + dYY) * (dXX + dYY) - 4 * (dXX * dYY - dXY * dXY)));
      e1_ = 0.5 * (dXX + dYY + sqrtf((dXX + dYY) * (dXX + dYY) - 4 * (dXX * dYY - dXY * dXY)));
      s_ = e0_+e1_;
    } else {
      e0_ = 0;
      e1_ = 0;
      s_ = -1;
    }
  }

  /** \brief Computes and sets the Shi-Tomasi Score \ref s_, considering only one specific pyramid level.
   *
   * @param l - Considered pyramid level.
   */
  void getSpecificLevelScore(const int l) const{
    if(isValidPatch_[l]){
      s_ = patches_[l].getScore();
    } else {
      s_ = -1;
    }
  }

  /** \brief Draws the patches of the MultilevelPatch into an image.
   *
   * @param drawImg    - Image in which should be drawn.
   * @param c          - Center pixel coordinates of the patch (on level 0)
   * @param stretch    - %Patch drawing magnification factor.
   * @param withBorder - Draw either the patches Patch::patch_ (withBorder = false) or the expanded patches
   *                     Patch::patchWithBorder_ (withBorder = true) .
   */
  void drawMultilevelPatch(cv::Mat& drawImg,const cv::Point2i& c,int stretch = 1,const bool withBorder = false) const{
    for(int l=nLevels_-1;l>=0;l--){
      if(isValidPatch_[l]){
        cv::Point2i corner = cv::Point2i((patchSize/2+(int)withBorder)*(pow(2,nLevels_-1)-pow(2,l)),(patchSize/2+(int)withBorder)*(pow(2,nLevels_-1)-pow(2,l)));
        patches_[l].drawPatch(drawImg,c+corner,stretch*pow(2,l),withBorder);
      }
    }
  }

  /** \brief Draws the patch borders into an image.
   *
   * @param drawImg     - Image in which the patch borders should be drawn.
   * @param s           - Scaling factor.
   * @param color       - Line color.
   */
  void drawMultilevelPatchBorder(cv::Mat& drawImg,const FeatureCoordinates& c,const float s, const cv::Scalar& color) const{
    patches_[0].drawPatchBorder(drawImg,c,s*pow(2.0,nLevels-1),color);
  }

  /** \brief Computes the RMSE (Root Mean Squared Error) with respect to the patches of an other MultilevelPatch
   *         for an specific pyramid level interval.
   *
   * @param mp        - \ref MultilevelPatch, which patches should be used for the RMSE computation.
   * @param l1         - Start pyramid level (l1<l2)
   * @param l2         - End pyramid level (l1<l2)
   * @return the RMSE value for the patches in the set pyramid level interval.
   */
  float computeAverageDifference(const MultilevelPatch<nLevels,patchSize>& mp, const int l1, const int l2) const{
    float offset = 0.0f;
    for(int l = l1; l <= l2; l++){
      const float* it_patch = patches_[l].patch_;
      const float* it_patch_in = mp.patches_[l].patch_;
      for(int y=0; y<patchSize; ++y){
        for(int x=0; x<patchSize; ++x, ++it_patch, ++it_patch_in){
          offset += *it_patch - *it_patch_in;
        }
      }
    }
    offset = offset/(patchSize*patchSize*(l2-l1+1));
    float error = 0.0f;
    for(int l = l1; l <= l2; l++){
      const float* it_patch = patches_[l].patch_;
      const float* it_patch_in = mp.patches_[l].patch_;
      for(int y=0; y<patchSize; ++y){
        for(int x=0; x<patchSize; ++x, ++it_patch, ++it_patch_in){
          error += std::pow(*it_patch - *it_patch_in - offset,2);
        }
      }
    }
    error = error/(patchSize*patchSize*(l2-l1+1));
    return std::sqrt(error);
  }

  /** \brief Checks if the MultilevelPatchFeature's patches are fully located within the corresponding images.
   *
   * @param pyr         - Image pyramid, which should be checked to fully contain the patches.
   * @param l           - Maximal pyramid level which should be checked (Note: The maximal level is the critical level.)
   * @param mpCoor      - Coordinates of the patch in the reference image (subpixel coordinates possible).
   * @param mpWarp      - Affine warping matrix. If nullptr not warping is considered.
   * @param withBorder  - If true, the check is executed with the expanded patch dimensions (incorporates the general patch dimensions).
   *                      If false, the check is only executed with the general patch dimensions.
   */
  static bool isMultilevelPatchInFrame(const ImagePyramid<nLevels>& pyr,const FeatureCoordinates& c, const int l = nLevels-1,const bool withBorder = false){
    if(!c.isInFront() || !c.com_warp_c()) return false;
    const auto coorTemp = pyr.levelTranformCoordinates(c,0,l);
    return Patch<patchSize>::isPatchInFrame(pyr.imgs_[l],coorTemp,withBorder);
  }

  /** \brief Extracts a multilevel patch from a given image pyramid.
   *
   * @param pyr         - Image pyramid from which the patch data should be extracted.
   * @param l           - Patches are extracted from pyramid level 0 to l.
   * @param mpCoor      - Coordinates of the patch in the reference image (subpixel coordinates possible).
   * @param mpWarp      - Affine warping matrix. If nullptr not warping is considered.
   * @param withBorder  - If true, both, the general patches and the corresponding expanded patches are extracted.
   */
  void extractMultilevelPatchFromImage(const ImagePyramid<nLevels>& pyr,const FeatureCoordinates& c, const int l = nLevels-1,const bool withBorder = false){
    for(unsigned int i=0;i<=l;i++){
      const auto coorTemp = pyr.levelTranformCoordinates(c,0,i);
      isValidPatch_[i] = true;
      patches_[i].extractPatchFromImage(pyr.imgs_[i],coorTemp,withBorder);
    }
  }
};

}


#endif /* ROVIO_MULTILEVELPATCH_HPP_ */
