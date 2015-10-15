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
#include "rovio/MultiCamera.hpp"

namespace rovio{

/** \brief A feature manager class.
 *
 *    @tparam nLevels    - Number of pyramid levels on which the feature is defined.
 *    @tparam patchSize - Edge length of the patches in pixels. Value must be a multiple of 2!.
 *                         Note: The patches edge length (in pixels) is the same for each patch, independently from the pyramid level.
 *    @tparam nCam       - Number of cameras
 */
template<int nLevels,int patchSize, int nCam>
class FeatureManager{
 public:
  int idx_;  /**<Feature ID.*/

  FeatureCoordinates log_previous_;
  FeatureCoordinates log_prediction_;
  FeatureCoordinates log_meas_;

  FeatureCoordinates* mpCoordinates_;
  FeatureDistance* mpDistance_;
  FeatureWarping* mpWarping_;
  FeatureStatistics<nCam>* mpStatistics_;
  MultilevelPatch<nLevels,patchSize>* mpMultilevelPatch_;

  // Internal objects, only used if no external objects are linked
  FeatureCoordinates* _mpCoordinates;
  FeatureDistance* _mpDistance;
  FeatureWarping* _mpWarping;
  FeatureStatistics<nCam>* _mpStatistics;
  MultilevelPatch<nLevels,patchSize>* _mpMultilevelPatch;

  /** Constructor
   */
  FeatureManager(){
    idx_ = -1;
    mpCoordinates_ = nullptr;
    mpDistance_ = nullptr;
    mpWarping_ = nullptr;
    mpStatistics_ = nullptr;
    mpMultilevelPatch_ = nullptr;
    _mpCoordinates = nullptr;
    _mpDistance = nullptr;
    _mpWarping = nullptr;
    _mpStatistics = nullptr;
    _mpMultilevelPatch = nullptr;
  }

  /** Destructor
   */
  ~FeatureManager(){
    delete _mpCoordinates;
    delete _mpDistance;
    delete _mpWarping;
    delete _mpStatistics;
    delete _mpMultilevelPatch;
  }

  FeatureManager& operator= (const FeatureManager &other){
    idx_ = other.idx_;
    log_previous_ = other.log_previous_;
    log_prediction_ = other.log_prediction_;
    return *this;
  }

  /** \brief Allocates all pointer which are still nullptr
   */
  void allocateMissing(){
    if(mpCoordinates_ == nullptr){
      _mpCoordinates = new FeatureCoordinates();
      mpCoordinates_ = _mpCoordinates;
    }
    if(mpDistance_ == nullptr){
      _mpDistance = new FeatureDistance();
      mpDistance_ = _mpDistance;
    }
    if(mpWarping_ == nullptr){
      _mpWarping = new FeatureWarping();
      mpWarping_ = _mpWarping;
    }
    if(mpStatistics_ == nullptr){
      _mpStatistics = new FeatureStatistics<nCam>();
      mpStatistics_ = _mpStatistics;
    }
    if(mpMultilevelPatch_ == nullptr){
      _mpMultilevelPatch = new MultilevelPatch<nLevels,patchSize>();
      mpMultilevelPatch_ = _mpMultilevelPatch;
    }
  }
};


/** \brief Class, storing and handling multiple features
 *
 * @tparam nLevels   - Total number of pyramid levels for each MultilevelPatchFeature in the set.
 * @tparam patchSize - Edge length of the patches in pixels. Value must be a multiple of 2!
 * @tparam nCam       - Number of cameras.
 * @tparam nMax       - Maximum number of MultilevelPatchFeature in the set.
 */
template<int nLevels,int patchSize, int nCam,int nMax>
class FeatureSetManager{
 public:
  FeatureManager<nLevels,patchSize,nCam> features_[nMax];  /**<Array of features.*/
  bool isValid_[nMax];  /**<Array, defining if there is a valid MultilevelPatchFeature at the considered array index. */
  int maxIdx_;  /**<Current maximum array/set index. Number of MultilevelPatchFeature, which have already been inserted into the set. */
  const MultiCamera<nCam>* mpMultiCamera_;

  /** \brief Constructor
   */
  FeatureSetManager(const MultiCamera<nCam>* mpMultiCamera){
    mpMultiCamera_ = mpMultiCamera;
    reset();
  }

  /** \brief Destructor
     */
  ~FeatureSetManager(){}

  /** \brief Allocates all pointer which are still nullptr
   */
  void allocateMissing(){
    for(unsigned int i=0;i<nMax;i++){
      features_[i].allocateMissing();
    }
  }

  /** \brief Sets all camera pointer in for FeatureCoordinates
   */
  void setAllCameraPointers(){
    for(unsigned int i=0;i<nMax;i++){
      if(features_[i].mpCoordinates_->camID_ < nCam && features_[i].mpCoordinates_->camID_ >= 0){
        features_[i].mpCoordinates_->mpCamera_ = &mpMultiCamera_->cameras_[features_[i].mpCoordinates_->camID_];
      } else {
        features_[i].mpCoordinates_->mpCamera_ = &mpMultiCamera_->cameras_[0];
      }
    }
  }

  /** \brief @todo
   */
  void setCamera(MultiCamera<nCam>* mpMultiCamera){
    mpMultiCamera_ = mpMultiCamera;
  }

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
        averageScore += std::max(features_[i].mpMultilevelPatch_->s_,0.0f);
        ++count;
      }
    }
    if(count>0) averageScore /= count;
    return averageScore;
  }

  /** \brief Adds the best MultilevelPatchFeature%s from a candidates list to an existing MultilevelPatchSet.
   *
   *  This function takes a given feature candidate list and builds, in a first step,
   *  MultilevelPatchFeature%s out of it using a given image pyramid. For each MultilevelPatchFeature the
   *  corresponding Shi-Tomasi Score is computed.
   *  In a second step, the candidate MultilevelPatchFeature%s are sorted and
   *  placed into buckets, depending on their individual Shi-Tomasi Score. MultilevelPatchFeature%s in a high bucket
   *  (high bucket index) have higher Shi-Tomasi Scores than MultilevelPatchFeature%s which have been placed into a
   *  low bucket.
   *  In a third step, the MultilevelPatchFeature%s in the buckets are reordered, depending on their distance
   *  to already existing features in the given MultilevelPatchSet. A small distance to an existing feature is punished,
   *  by moving the concerned candidate MultilevelPatchFeature into a lower bucket.
   *  Finally the existing MultilevelPatchSet is expanded with the best (high bucket index) candidate MultilevelPatchFeature%s.
   *
   * @param candidates             - List of candidate feature coordinates.
   * @param pyr                    - Image pyramid used to extract the MultilevelPatchFeature%s from the candidates list.
   * @param camID                  - %Camera ID
   * @param initTime               - Current time (time at which the MultilevelPatchFeature%s are created from the candidates list).
   * @param l1                     - Start pyramid level for the Shi-Tomasi Score computation of MultilevelPatchFeature%s extracted from the candidates list.
   * @param l2                     - End pyramid level for the Shi-Tomasi Score computation of MultilevelPatchFeature%s extracted from the candidates list.
   * @param maxN                   - Maximal number of features which should be added to the mlpSet.
   * @param nDetectionBuckets      - Number of buckets.
   * @param scoreDetectionExponent - Choose it between [0 1]. 1 : Candidate features are sorted linearly into the buckets, depending on their Shi-Tomasi score.
   *                                                          0 : All candidate features are filled into the highest bucket.
   *                                 A small scoreDetectionExponent forces more candidate features into high buckets.
   * @param penaltyDistance        - If a candidate feature has a smaller distance to an existing feature in the mlpSet, it is punished (shifted in an lower bucket) dependent of its actual distance to the existing feature.
   * @param zeroDistancePenalty    - A candidate feature in a specific bucket is shifted zeroDistancePenalty-buckets back to a lower bucket if it has zero distance to an existing feature in the mlpSet.
   * @param requireMax             - Should the adding of maxN be enforced?
   * @param minScore               - Shi-Tomasi Score threshold for the best (highest Shi-Tomasi Score) MultilevelPatchFeature extracted from the candidates list.
   *                                 If the best MultilevelPatchFeature has a Shi-Tomasi Score less than or equal this threshold, the function aborts and returns an empty map.
   *
   * @return an unordered_set, holding the indizes of the MultilevelPatchSet, at which the new MultilevelPatchFeature%s have been added (from the candidates list).
   */
  // TODO: work more on bearing vectors (in general)
  // TODO: add corner motion dependency
  // TODO: check inFrame, only if COVARIANCE not too large
  std::unordered_set<unsigned int> addBestCandidates(const std::vector<FeatureCoordinates>& candidates, const ImagePyramid<nLevels>& pyr, const int camID, const double initTime,
                                                     const int l1, const int l2, const int maxN, const int nDetectionBuckets, const double scoreDetectionExponent,
                                                     const double penaltyDistance, const double zeroDistancePenalty, const bool requireMax, const float minScore){
    std::unordered_set<unsigned int> newSet;
    std::vector<MultilevelPatch<nLevels,patchSize>> multilevelPatches;
    multilevelPatches.reserve(candidates.size());

    // Create MultilevelPatches from the candidates list and compute their Shi-Tomasi Score.
    float maxScore = -1.0;
    for(int i=0;i<candidates.size();i++){
      multilevelPatches.emplace_back();
      if(multilevelPatches.back().isMultilevelPatchInFrame(pyr,candidates[i],l2,nullptr,true)){
        multilevelPatches.back().extractMultilevelPatchFromImage(pyr,candidates[i],l2,nullptr,true);
        multilevelPatches.back().computeMultilevelShiTomasiScore(l1,l2);
        if(multilevelPatches.back().s_ > maxScore) maxScore = multilevelPatches.back().s_;
      } else {
        multilevelPatches.back().s_ = -1;
      }
    }
    if(maxScore <= minScore){
      return newSet;
    }

    // Make buckets and fill based on score
    std::vector<std::unordered_set<int>> buckets(nDetectionBuckets,std::unordered_set<int>());
    unsigned int newBucketID;
    float relScore;
    for(int i=0;i<candidates.size();i++){
      relScore = (multilevelPatches[i].s_-minScore)/(maxScore-minScore);
      if(relScore > 0.0){
        newBucketID = std::ceil((nDetectionBuckets-1)*(pow(relScore,static_cast<float>(scoreDetectionExponent))));
        if(newBucketID>nDetectionBuckets-1) newBucketID = nDetectionBuckets-1;
        buckets[newBucketID].insert(i);
      }
    }

    // Move buckets based on current features
    double d2;
    double t2 = pow(penaltyDistance,2);
    bool doDelete;
    FeatureCoordinates featureCoordinates;
    FeatureDistance featureDistance;
    featureCoordinates.mpCamera_ = &mpMultiCamera_->cameras_[camID];
    for(unsigned int i=0;i<nMax;i++){
      if(isValid_[i]){
        mpMultiCamera_->transformFeature(camID,*(features_[i].mpCoordinates_),*(features_[i].mpDistance_),featureCoordinates,featureDistance);
        if(featureCoordinates.isInFront()){
          for (unsigned int bucketID = 1;bucketID < nDetectionBuckets;bucketID++) {
            for (auto it_cand = buckets[bucketID].begin();it_cand != buckets[bucketID].end();) {
              doDelete = false;
              d2 = std::pow(featureCoordinates.get_c().x - candidates[*it_cand].get_c().x,2) + std::pow(featureCoordinates.get_c().y - candidates[*it_cand].get_c().y,2);  // Squared distance between the existing feature and the candidate feature.
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
      }
    }

    // Incrementally add features and update candidate buckets (Check distance of candidates with respect to the newly inserted feature).
    int addedCount = 0;
    for (int bucketID = nDetectionBuckets-1;bucketID >= 0+static_cast<int>(!requireMax);bucketID--) {
      while(!buckets[bucketID].empty() && addedCount < maxN && getValidCount() != nMax) {
        const int nf = *(buckets[bucketID].begin());
        buckets[bucketID].erase(nf);
        const int ind = makeNewFeature(camID);
        *(features_[ind].mpCoordinates_) = candidates[nf];
        features_[ind].mpCoordinates_->camID_ = camID;
        features_[ind].mpCoordinates_->mpCamera_ = &mpMultiCamera_->cameras_[camID];
        *(features_[ind].mpMultilevelPatch_) = multilevelPatches[nf];
        features_[ind].mpDistance_->reset();
        features_[ind].mpWarping_->reset();
        if(ind >= 0){
          newSet.insert(ind);
        }
        addedCount++;
        for (unsigned int bucketID2 = 1;bucketID2 <= bucketID;bucketID2++) {
          for (auto it_cand = buckets[bucketID2].begin();it_cand != buckets[bucketID2].end();) {
            doDelete = false;
            d2 = std::pow(candidates[nf].get_c().x - candidates[*it_cand].get_c().x,2) + std::pow(candidates[nf].get_c().y - candidates[*it_cand].get_c().y,2);
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
};

///** \brief Prunes the (corner/feature) candidate list by erasing candidates, which are too close to an existing feature. // TODO: overthink
// *
// * @param mlpSet             - MultilevelPatchSet, containing the existing MultilevelPatchFeature%s .
// * @param candidates         - List of extracted corner coordinates/candidates (defined on pyramid level 0).
// * @param candidateID        - Camera ID, in which the candidates have been extracted.
// */
//void pruneCandidates(const MultilevelPatchSet<nLevels,patchSize,nMax>& mlpSet, std::list<cv::Point2f>& candidates, const int candidateID){
//  constexpr float t2 = patchSize*patchSize;  // TODO: param
//  bool prune;
//  const MultilevelPatchFeature<nLevels,patchSize>* mpFeature;
//  FeatureCoordinates featureCoordinates;
//  for (auto it = candidates.begin(); it != candidates.end();) {
//    prune = false;
//    for(unsigned int i=0;i<nMax;i++){
//      if(mlpSet.isValid_[i]){
//        mpFeature = &mlpSet.features_[i];
//        featureCoordinates = static_cast<FeatureCoordinates>(*mpFeature);
//        featureCoordinates.set_nor(featureCoordinates.get_nor_other(candidateID));
//        featureCoordinates.camID_ = candidateID;
//        if(featureCoordinates.isInFront() && pow(it->x-featureCoordinates.get_c().x,2) + pow(it->y-featureCoordinates.get_c().y,2) < t2){
//          prune = true;
//          break;
//        }
//      }
//    }
//    if(prune){
//      it = candidates.erase(it);
//    } else {
//      it++;
//    }
//  }
//}

}


#endif /* FEATUREMANAGER_HPP_ */
