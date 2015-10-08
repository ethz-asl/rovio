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

#ifndef FEATUREMANAGER_HPP_
#define FEATUREMANAGER_HPP_

#include "rovio/FeatureCoordinates.hpp"
#include "rovio/FeatureDistance.hpp"
#include "rovio/FeatureWarping.hpp"
#include "rovio/FeatureStatistics.hpp"
#include "rovio/MultilevelPatch.hpp"

namespace rovio{

/** \brief A feature manager class.
 *
 *    @tparam nLevels    - Number of pyramid levels on which the feature is defined.
 *    @tparam patch_size - Edge length of the patches in pixels. Value must be a multiple of 2!.
 *                         Note: The patches edge length (in pixels) is the same for each patch, independently from the pyramid level.
 *    @tparam nCam       - Number of cameras
 */
template<int nLevels,int patch_size, int nCam>
class FeatureManager{
 public:
  int idx_;  /**<Feature ID.*/
  Eigen::MatrixXf A_;  /**<A matrix of the linear system of equations, needed for the multilevel patch alignment.*/
  Eigen::MatrixXf b_;  /**<b matrix/vector of the linear system of equations, needed for the multilevel patch alignment.*/
  Eigen::ColPivHouseholderQR<Eigen::MatrixXf> mColPivHouseholderQR_;

  FeatureCoordinates log_previous_;
  FeatureCoordinates log_prediction_;
  FeatureCoordinates log_predictionC0_;
  FeatureCoordinates log_predictionC1_;
  FeatureCoordinates log_predictionC2_;
  FeatureCoordinates log_predictionC3_;
  FeatureCoordinates log_meas_;
  FeatureCoordinates log_current_;

  FeatureCoordinates* mpCoordinates_;
  FeatureDistance* mpDistance_;
  FeatureWarping* mpWarping_;
  FeatureStatistics<nCam>* mpStatistics_;
  MultilevelPatch<nLevels,patch_size>* mpMultilevelPatch_;

  /** Constructor
   */
  FeatureManager(){
    idx_ = -1;
    mpCoordinates_ = nullptr;
    mpDistance_ = nullptr;
    mpWarping_ = nullptr;
    mpStatistics_ = nullptr;
    mpMultilevelPatch_ = nullptr;
  }

  /** Destructor
   */
  ~FeatureManager(){}

  /** \brief Resets the FeatureManager.
   *
   * @param idx - feature ID
   * @initTime  - Time at initialization.
   */
  void reset(const int idx = -1, const double initTime = 0.0){
    idx_ = idx;
    mpCoordinates_->resetCoordinates();
    mpDistance_->reset();
    mpWarping_->reset();
    mpStatistics_->resetStatistics(initTime);
    mpMultilevelPatch_->reset();
  }

  /** \brief Computes the RMSE (Root Mean Squared Error) with respect to the patch extracted from the reference image
   *         for an specific pyramid level interval.
   *
   * @param pyr        - Image pyramid of reference image
   * @param l1         - Start pyramid level (l1<l2)
   * @param l2         - End pyramid level (l1<l2)
   * @return the RMSE value for the patches in the set pyramid level interval.
   */
  float computeAverageDifferenceReprojection(const ImagePyramid<nLevels>& pyr, const int l1, const int l2, const bool doWarping = false) const{
    MultilevelPatch<nLevels,patch_size> mlpReprojected;
    extractMultilevelPatchFromImage(mlpReprojected,pyr,l2,false,doWarping);
    return computeAverageDifference(mlpReprojected,l1,l2);
  }
};


/** \brief Class, storing and handling multiple features
 *
 * @tparam nLevels   - Total number of pyramid levels for each MultilevelPatchFeature in the set.
 * @tparam patch_size - Edge length of the patches in pixels. Value must be a multiple of 2!
 * @tparam nCam       - Number of cameras.
 * @tparam nMax       - Maximum number of MultilevelPatchFeature in the set.
 */
template<int nLevels,int patch_size, int nCam,int nMax>
class FeatureSetManager{
 public:
  FeatureManager<nLevels,patch_size,nCam> features_[nMax];  /**<Array of features.*/
  bool isValid_[nMax];  /**<Array, defining if there is a valid MultilevelPatchFeature at the considered array index. */
  int maxIdx_;  /**<Current maximum array/set index. Number of MultilevelPatchFeature, which have already been inserted into the set. */

  /** \brief Constructor
   */
  FeatureSetManager(){
    reset();
  }

  /** \brief Destructor
     */
  ~FeatureSetManager(){}

  /** \brief Resets the MultilevelPatchSet.
   */
  void reset(){
    maxIdx_ = 0;
    for(unsigned int i=0;i<nMax;i++){
      isValid_[i] = false;
    }
  }

  /** \brief Resets the MultilevelPatchSet.
   *
   * @param ind - Free array/set index.
   * @return true, if a free index was found.
   */
  bool getFreeIndex(int& ind) const{
    for(unsigned int i=0;i<nMax;i++){
      if(isValid_[i] == false){
        ind = i;
        return true;
      }
    }
    return false;
  }

  /** \brief Get the number of valid MultilevelPatchFeature in the set.
   *
   * @return the number of valid MultilevelPatchFeature in the set.
   */
  int getValidCount() const{
    int count = 0;
    for(unsigned int i=0;i<nMax;i++){
      if(isValid_[i] == true) ++count;
    }
    return count;
  }

  /** \brief Makes a new feature
   *
   * @return the array/set index, where the MultilevelPatchFeature has been stored. The value -1 is returned,
   *         if the feature could not be inserted, because the maximal number of features have already been reached.
   */
  int makeNewFeature(const int camID){
    int newInd = -1;
    if(getFreeIndex(newInd)){
      features_[newInd].idx_ = maxIdx_++;
      isValid_[newInd] = true;
    } else {
      std::cout << "Feature Manager: maximal number of feature reached" << std::endl;
    }
    return newInd;
  }

  /** \brief Get the average Shi-Tomasi Score of all MultilevelPatchFeature in the set.
   *
   * @return the average Shi-Tomasi Score of all MultilevelPatchFeature in the set.
   */
  float getAverageScore(){
    float averageScore = 0;
    int count = 0;
    for(unsigned int i=0;i<nMax;i++){
      if(isValid_[i] == true){
        averageScore += std::max(features_[i].mpMultilevelPatch_.s_,0.0f);
        ++count;
      }
    }
    if(count>0) averageScore /= count;
    return averageScore;
  }
};

}


#endif /* FEATUREMANAGER_HPP_ */
