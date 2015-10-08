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

#ifndef ROVIO_COMMON_VISION_HPP_
#define ROVIO_COMMON_VISION_HPP_

#include <Eigen/Dense>
#include <Eigen/Cholesky>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <unordered_set>
#include <Eigen/Eigenvalues>
#include "lightweight_filtering/State.hpp"
#include "rovio/Camera.hpp"

namespace rovio{

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
 * @tparam nLevels   - Total number of pyramid levels for each MultilevelPatchFeature.
 * @tparam patch_size - Edge length of the patches in pixels. Value must be a multiple of 2!
 * @tparam nMax       - Maximum number of MultilevelPatchFeature%s in the MultilevelPatchSet.
 *
 * @param mlpSet                 - MultilevelPatchSet which should be expanded/filled with the best MultilevelPatchFeature%s from the candidates list.
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
template<int nLevels,int patch_size,int nMax>
std::unordered_set<unsigned int> addBestCandidates(MultilevelPatchSet<nLevels,patch_size,nMax>& mlpSet, std::list<cv::Point2f>& candidates, const ImagePyramid<nLevels>& pyr, const int camID, const double initTime,
                                                   const int l1, const int l2, const int maxN, const int nDetectionBuckets, const double scoreDetectionExponent,
                                                   const double penaltyDistance, const double zeroDistancePenalty, const bool requireMax, const float minScore){
  std::unordered_set<unsigned int> newSet;
  std::list<MultilevelPatchFeature<nLevels,patch_size>> candidatesWithPatch;

  // Create MultilevelPatchFeature from the candidates list and compute their Shi-Tomasi Score.
  float maxScore = -1.0;
  for(auto it = candidates.begin(); it != candidates.end(); ++it){
    candidatesWithPatch.emplace_back();
    candidatesWithPatch.rbegin()->reset(-1,initTime);
    candidatesWithPatch.rbegin()->set_c(*it);
    if(isMultilevelPatchInFrame(*candidatesWithPatch.rbegin(),pyr,nLevels-1,true)){
      extractMultilevelPatchFromImage(*candidatesWithPatch.rbegin(),pyr,nLevels-1,true);
      candidatesWithPatch.rbegin()->computeMultilevelShiTomasiScore(l1,l2);
      if(candidatesWithPatch.rbegin()->s_ > maxScore) maxScore = candidatesWithPatch.rbegin()->s_;
    }
    else {
      candidatesWithPatch.rbegin()->s_ = -1;
    }
  }
  if(maxScore <= minScore){
    return newSet;
  }

  // Make buckets and fill based on score
  std::vector<std::unordered_set<MultilevelPatchFeature<nLevels,patch_size>*>> buckets(nDetectionBuckets,std::unordered_set<MultilevelPatchFeature<nLevels,patch_size>*>());
  unsigned int newBucketID;
  float relScore;
  for (auto it_cand = candidatesWithPatch.begin(); it_cand != candidatesWithPatch.end(); ++it_cand) {
    relScore = (it_cand->s_-minScore)/(maxScore-minScore);
    if(relScore > 0.0){
      newBucketID = std::ceil((nDetectionBuckets-1)*(pow(relScore,static_cast<float>(scoreDetectionExponent))));
      if(newBucketID>nDetectionBuckets-1) newBucketID = nDetectionBuckets-1;
      buckets[newBucketID].insert(&(*it_cand));
    }
  }

  // Move buckets based on current features
  double d2;
  double t2 = pow(penaltyDistance,2);
  bool doDelete;
  MultilevelPatchFeature<nLevels,patch_size>* mpFeature;
  FeatureCoordinates featureCoordinates;
  for(unsigned int i=0;i<nMax;i++){
    if(mlpSet.isValid_[i]){
      mpFeature = &mlpSet.features_[i];
      featureCoordinates = static_cast<FeatureCoordinates>(*mpFeature);
      featureCoordinates.set_nor(featureCoordinates.get_nor_other(camID));
      featureCoordinates.camID_ = camID;
      if(featureCoordinates.isInFront()){
        for (unsigned int bucketID = 1;bucketID < nDetectionBuckets;bucketID++) {
          for (auto it_cand = buckets[bucketID].begin();it_cand != buckets[bucketID].end();) {
            doDelete = false;
            d2 = std::pow(featureCoordinates.get_c().x - (*it_cand)->c_.x,2) + std::pow(featureCoordinates.get_c().y - (*it_cand)->c_.y,2);  // Squared distance between the existing feature and the candidate feature.
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

  // Incrementally add features and update candidate buckets (Check distance of candidates with respect to the
  // newly inserted feature).
  MultilevelPatchFeature<nLevels,patch_size>* mpNewFeature;
  int addedCount = 0;
  for (int bucketID = nDetectionBuckets-1;bucketID >= 0+static_cast<int>(!requireMax);bucketID--) {
    while(!buckets[bucketID].empty() && addedCount < maxN && mlpSet.getValidCount() != nMax) {
      mpNewFeature = *(buckets[bucketID].begin());
      buckets[bucketID].erase(mpNewFeature);
      const int ind = mlpSet.addFeature(*mpNewFeature,camID);
      if(ind >= 0){
        newSet.insert(ind);
      }
      addedCount++;
      for (unsigned int bucketID2 = 1;bucketID2 <= bucketID;bucketID2++) {
        for (auto it_cand = buckets[bucketID2].begin();it_cand != buckets[bucketID2].end();) {
          doDelete = false;
          d2 = std::pow(mpNewFeature->get_c().x - (*it_cand)->c_.x,2) + std::pow(mpNewFeature->get_c().y - (*it_cand)->c_.y,2);
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

/** \brief Extract FastCorner coordinates from an Image Pyramid.
 *
 * @tparam nLevels          - Total number of levels of the image pyramid.
 *
 * @param pyr                - Image pyramid, which serves for the corner extraction.
 * @param candidates         - List of the extracted corner coordinates (defined on pyramid level 0).
 * @param l                  - Pyramid level at which the corners should be extracted.
 * @param detectionThreshold - Detection threshold of the used cv::FastFeatureDetector.
 *                             See http://docs.opencv.org/trunk/df/d74/classcv_1_1FastFeatureDetector.html
 */
template <int nLevels>
void detectFastCorners(const ImagePyramid<nLevels>& pyr, std::list<cv::Point2f>& candidates, int l, int detectionThreshold) {
  std::vector<cv::KeyPoint> keypoints;
  cv::FastFeatureDetector feature_detector_fast(detectionThreshold, true);
  feature_detector_fast.detect(pyr.imgs_[l], keypoints);
  cv::Point2f level_c;
  for (auto it = keypoints.cbegin(), end = keypoints.cend(); it != end; ++it) {
    level_c = cv::Point2f(it->pt.x, it->pt.y);
    candidates.push_back(levelTranformCoordinates(level_c,pyr,l,0));
  }
}

/** \brief Prunes the (corner/feature) candidate list by erasing candidates, which are too close at an existing feature.
 *
 * @tparam nLevels          - Total number of levels of the image pyramid.
 * @tparam patch_size        - Edge length of the patches in pixels. Value must be a multiple of 2!
 * @tparam nMax              - Maximum number of MultilevelPatchFeature%s in the MultilevelPatchSet.
 *
 * @param mlpSet             - MultilevelPatchSet, containing the existing MultilevelPatchFeature%s .
 * @param candidates         - List of extracted corner coordinates/candidates (defined on pyramid level 0).
 * @param candidateID        - Camera ID, in which the candidates have been extracted.
 */
template<int nLevels,int patch_size,int nMax>
void pruneCandidates(const MultilevelPatchSet<nLevels,patch_size,nMax>& mlpSet, std::list<cv::Point2f>& candidates, const int candidateID){ // TODO: add corner motion dependency
  constexpr float t2 = patch_size*patch_size;  // TODO: param
  bool prune;
  const MultilevelPatchFeature<nLevels,patch_size>* mpFeature;
  FeatureCoordinates featureCoordinates;
  for (auto it = candidates.begin(); it != candidates.end();) {
    prune = false;
    for(unsigned int i=0;i<nMax;i++){
      if(mlpSet.isValid_[i]){
        mpFeature = &mlpSet.features_[i];
        featureCoordinates = static_cast<FeatureCoordinates>(*mpFeature);
        featureCoordinates.set_nor(featureCoordinates.get_nor_other(candidateID));
        featureCoordinates.camID_ = candidateID;
        if(featureCoordinates.isInFront() && pow(it->x-featureCoordinates.get_c().x,2) + pow(it->y-featureCoordinates.get_c().y,2) < t2){ // TODO: check inFrame, only if covariance not too large
          prune = true;
          break;
        }
      }
    }
    if(prune){
      it = candidates.erase(it);
    } else {
      it++;
    }
  }
}

/** \brief Get the depth value from the triangulation of two bearing vectors.
 *
 *  @param C1fP    - Bearing vector in the reference frame C1 (unit length!).
 *  @param C2fP    - Bearing vector in another frame C2 (unit length!).
 *  @param C2rC2C1 - Position vector, pointing from C2 to C1, expressed in cooridantes of C2.
 *  @param qC2C1   - Quaternion, expressing the orientation of C1 in the C2.
 *  @param d       - Triangulated depth value along the bearing vector C1fP.
 *  @return true, if triangulation successful. This means the angle between the projection rays has not been too small.
 */
bool getDepthFromTriangulation(const V3D& C1fP, const V3D& C2fP, const V3D& C2rC2C1, const QPD& qC2C1, double* d)
{
  Eigen::Matrix<double,3,2> B;
  B <<  qC2C1.rotate(C1fP), C2fP;
  const Eigen::Matrix2d BtB = B.transpose() * B;
  if(BtB.determinant() < 0.000001)
    return false;                      // Projection rays almost parallel.
  const Eigen::Vector2d dv = - BtB.inverse() * B.transpose() * C2rC2C1;
  *d = fabs(dv[0]);
  return true;
}

/** \brief Get the depth uncertainty tau of a triangulated depth value.
 *
 *  Consider a bearing vector C1fP in a reference frame and a bearing vector C2fP in a partner frame have been used
 *  to triangulate a depth value d (along the bearing vector C1fP). Let's call the so gained 3D landmark position P.
 *  In order to get depth uncertainty value of d (along the bearing vector C1fP), a constant pixel error
 *  of the detection of C2fP can be projected to the ray of C1fP (to the side which is farther to the reference frame).
 *  Let's call 3D point corresponding to the maximal pixel error P_plus.
 *  The depth uncertainty tau is then defined as \f$tau=|(P\_plus - P)|\f$.
 *
 *  @param C1fP           - Bearing vector in the reference frame (unit length!).
 *  @param C2fP           - Bearing vector in another frame (unit length!).
 *  @param C2rC2C1        - Position vector, pointing from C2 to C1, expressed in cooridantes of C2.
 *  @param d              - Triangulated depth value along the bearing vector C1fP.
 *  @param px_error_angle - Angle between the bearing vector C2fP and the bearing vector corresponding to the maximal
 *                          pixel error. <br>
 *                          Compute it as: <br>
 *                           \f$px\_error\_angle = 2 \cdot \arctan{\frac{px\_error}{2 \cdot fx}}\f$ <br>
 *                          ,where px_error is the assumed pixel error (e.g. 1 pixel) and
 *                          fx the focal length (expressed in pixel) of the camera belonging to C2fP.
 * @return the depth uncertainty value \f$tau=|(P\_plus - P)|\f$.
 */
float getDepthUncertaintyTau(const V3D& C1fP, const V3D& C1rC1C2, const float d, const float px_error_angle)
{
  float t_0 = C1rC1C2(0);
  float t_1 = C1rC1C2(1);
  float t_2 = C1rC1C2(2);
  float a_0 = C1fP(0) * d - t_0;
  float a_1 = C1fP(1) * d - t_1;
  float a_2 = C1fP(2) * d - t_2;
  float t_norm = std::sqrt(t_0 * t_0 + t_1 * t_1 + t_2 * t_2);
  float a_norm = std::sqrt(a_0 * a_0 + a_1 * a_1 + a_2 * a_2);
  float alpha = std::acos((C1fP(0) * t_0 + C1fP(1) * t_1 + C1fP(2) * t_2) / t_norm);
  float beta = std::acos(( a_0 * (-t_0) + a_1 * (-t_1) + a_2 * (-t_2) ) / (t_norm * a_norm));
  float beta_plus = beta + px_error_angle;
  float gamma_plus = M_PI - alpha - beta_plus;                             // Triangle angles sum to PI.
  float d_plus = t_norm * std::sin(beta_plus) / std::sin(gamma_plus);      // Law of sines.
  return (d_plus - d);                                                     // Tau.
}

}


#endif /* ROVIO_COMMON_VISION_HPP_ */
