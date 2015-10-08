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

#ifndef FEATURESTATISTICS_HPP_
#define FEATURESTATISTICS_HPP_

namespace rovio{

/** \brief Defines the tracking status of a MultilevelPatchFeature.
 *
 *  TrackingStatus described how the feature is used within the filter.
 */
enum TrackingStatus{
  UNKNOWN, /**< Unknown*/
  NOT_IN_FRAME, /**< Feature is not in frame*/
  FAILED_ALIGNEMENT, /**< The pre-alignment failed*/
  FAILED_TRACKING, /**< The tracking in the filter failed, e.g. outlier*/
  TRACKED /**< Successfull tracking with filter*/
};

template<int nCam>
class FeatureStatistics{
 public:
  double initTime_;  /**<Time of feature initialization.*/
  double currentTime_;  /**<Time of last feature measurement.*/
  int totCount_;  /**<Number of timesteps which have passed since feature initialization.*/
  int trackedCount_;  /**<Number of timesteps where the feature was successfully tracked.*/
  int localQualityRange_; /**Range for local quality evaluation.*/
  double localQuality_[nCam]; /**Quality value in the range [0, 1] (ratio between tracking number and visibility number).
   *         1 means very good tracking quality. How is the tracking within the local range, FAILED_ALIGNEMENT or FAILED_TRACKING is worse than not in image.*/
  int localVisibilityRange_; /**Range for local visibility evaluation.*/
  double localVisibility_; /**Quality value in the range [0, 1], 1 means that the feature was always visible in some frame.*/
  int minFrameGlobalQuality_ = 100; /**Minimum of frames for maximal quality.*/

  /** \brief Get the local visibility quality of the MultilevelPatchFeature.
   *
   * @param localRange - Have only a look at the last "localRange" frames.
   * @return quality value in the range [0, 1] (ratio between visibility number and localRange).
   *         1 means that the feature was always visible in the considered frames.
   */


  TrackingStatus status_[nCam];  /**<MultilevelPatchFeature tracking and mapping status.*/
  std::map<TrackingStatus,int> cumulativeTrackingStatus_[nCam];  /**< Count for specific tracking status.*/
  std::map<double,TrackingStatus> statistics_[nCam];  /**< Accumulation of status (index is the time)*/

  /** \brief Constructor.
   *
   *  Initializes the feature statistics
   * */
  FeatureStatistics(const double& currentTime = 0.0){
    resetStatistics(currentTime);
    localQualityRange_ = 10;
    localVisibilityRange_ = 100;
    minFrameGlobalQuality_ = 100;
  };
  ~FeatureStatistics(){};

  /** \brief Resets the statistics
   */
  void resetStatistics(const double& currentTime){
    for(int i=0;i<nCam;i++){
      cumulativeTrackingStatus_[i][UNKNOWN] = 0;
      cumulativeTrackingStatus_[i][NOT_IN_FRAME] = 0;
      cumulativeTrackingStatus_[i][FAILED_ALIGNEMENT] = 0;
      cumulativeTrackingStatus_[i][FAILED_TRACKING] = 0;
      cumulativeTrackingStatus_[i][TRACKED] = 0;
      statistics_[i].clear();
      status_[i] = UNKNOWN;
      localQuality_[i] = 1.0;
    }
    totCount_ = 0;
    trackedCount_ = 0;
    initTime_ = currentTime;
    currentTime_ = currentTime;
    localVisibility_ = 1.0;
  }

  /** \brief Increases the MultilevelPatchFeature statistics and resets the \ref status_.
   *
   * @param currentTime - Current time.
   */
  void increaseStatistics(const double& currentTime){
    if(currentTime <= currentTime_) std::cout << "ERROR: timing is not incremental" << std::endl;
    if(trackedInSomeFrame()){
      trackedCount_++;
    }

    // Increase local visibility
    if(inSomeFrame()){
      localVisibility_ = localVisibility_*(1-1.0/localVisibilityRange_) + 1.0/localVisibilityRange_;
    } else {
      localVisibility_ = localVisibility_*(1-1.0/localVisibilityRange_);
    }

    for(int i=0;i<nCam;i++){
      // Increase local quality
      if(status_[i] == TRACKED){
        localQuality_[i] = localQuality_[i]*(1-1.0/localQualityRange_) + 1.0/localQualityRange_;
      } else if(status_[i] == FAILED_ALIGNEMENT || status_[i] == FAILED_TRACKING){
        localQuality_[i] = localQuality_[i]*(1-1.0/localQualityRange_);
      }

      // Store
      cumulativeTrackingStatus_[i][status_[i]]++;
      statistics_[i][currentTime_] = status_[i];
      status_[i] = UNKNOWN;
    }
    totCount_++;
    currentTime_ = currentTime;
  }

  /** \brief How many times did a specific \ref TrackingStatus occur (current status included) in a specific camera?
   *
   * @param s - \ref TrackingStatus of interest.
   * @param camID - \ref Camera ID of camera of interest. If -1 (or not specified) counts for all camera.
   * @param n - \ref Last n frames. If 0 (or not specified) counts for all frames.
   * @return the number of how many times the \ref TrackingStatus s occured.
   */
  int countTrackingStatistics(const TrackingStatus s, const int camID = -1, const int n = 0) const{
    int count = 0;
    int startID = 0;
    int endID = nCam-1;
    if(camID != -1){
      if(camID < 0 || camID >= nCam){
        std::cout << "ERROR: wrong camera ID in countTrackingStatistics" << std::endl;
        return 0;
      }
      startID = camID;
      endID = camID;
    }
    for(int i=startID;i<=endID;i++){
      if(n==0){
        count += cumulativeTrackingStatus_[i].at(s) + (int)(status_[i] == s);
      } else {
        auto it = statistics_[i].rbegin();
        for(int j=0;j<n-1 && it != statistics_[i].rend();++j){
          if(it->second == s) count++;
          ++it;
        }
        count += (int)(status_[i] == s);
      }
    }
    return count;
  }

  /** \brief Returns whether the feature is supposed to be visible in some frame
   *
   * @return Is the feature visible in some frame
   */
  bool inSomeFrame() const{
    for(int i=0;i<nCam;i++){
      if(status_[i] == FAILED_ALIGNEMENT || status_[i] == FAILED_TRACKING || status_[i] == TRACKED){
        return true;
      }
    }
    return false;
  }

  /** \brief Returns whether the feature is tracked in some frame
   *
   * @return Is the feature tracked in some frame
   */
  bool trackedInSomeFrame() const{
    for(int i=0;i<nCam;i++){
      if(status_[i] == TRACKED){
        return true;
      }
    }
    return false;
  }

  /** \brief Returns the total count of timesteps since feature initialization (including the current).
   *
   * @return Total count of timesteps
   */
  int countTot() const{
    return totCount_+1;
  }

  /** \brief Returns the total count of timesteps the feature was successfully tracked (including the current).
   *
   * @return Total count of successfull trackings
   */
  int countTracked() const{
    return trackedCount_+(int)trackedInSomeFrame();
  }

  /** \brief Returns the count of frames since feature initialization where the feature was in the frame (including the current).
   *
   * @param camID - \ref Camera ID of camera of interest. If -1 (or not specified) counts for all camera.
   * @param n - \ref Last n frames. If 0 (or not specified) counts for all frames.
   * @return In frame count
   */
  int countInFrame(const int camID = -1, const int n = 0) const{
    return countTrackingStatistics(FAILED_ALIGNEMENT,camID,n)+countTrackingStatistics(FAILED_TRACKING,camID,n)+countTrackingStatistics(TRACKED,camID,n);
  }

  /** \brief How was the overall tracking of the feature. What is the tracking ratio of the feature, whereby the maximal value
   * is only obtained if a minimum of "minFrameGlobalQuality_" frames has passed since initialization (punishes feature with low feature
   * count)
   *
   * @return quality value in the range [0, 1] (tracking ratio of feature).
   *         1 means that the feature was always tracked
   *         gets further penalized if a feature has not a minimum of "minFrameGlobalQuality_" frames
   */
  double getGlobalQuality() const{
    const double trackingRatio = static_cast<double>(countTracked())/countTot();
    return trackingRatio*std::min(static_cast<double>(countTot())/minFrameGlobalQuality_,1.0);
  }

  /** \brief Compute the average of the local qualities for the tracking in each camera
   *
   * @return quality value in the range [0, 1] (tracking ratio of feature for frame where the feature is predicted to be in).
   *         1 means that the feature was always tracked
   */
  double getAverageLocalQuality() const{
    double q = 0;;
    for(int i=0;i<nCam;i++){
      q += localQuality_[i];
    }
    return q/nCam;
  }


  /** \brief Is the current feature a good feature. Combines different quality criteria for deciding if it is a good feature.
   * The product of local quality and visibility quality is compared with a threshold. This threshold depends on the global
   * quality (lower if the global quality is good).
   *
   * @param localRange           local range for local quality
   * @param localVisibilityRange local range for visibility quality
   * @param upper                if the global quality is bad (0) than the combination of local and visibility quality must be above this
   * @param lower                if the global quality is very good (1) than the combination of local and visibility quality must be above this
   * @return
   */
  bool isGoodFeature(const double upper = 0.8, const double lower = 0.1) const{
    return getAverageLocalQuality()*localVisibility_ > upper-(upper-lower)*getGlobalQuality();

    // TODO: consider information quality (neibours, shape)
  }
};

}


#endif /* FEATURESTATISTICS_HPP_ */
