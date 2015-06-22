/*
 * Backend.hpp
 *
 *  Created on: Jun 17, 2015
 *      Author: andreas
 */

#ifndef INCLUDE_ROVIO_BACKEND_HPP_
#define INCLUDE_ROVIO_BACKEND_HPP_

#include <condition_variable>
#include <ctime>
#include <deque>
#include <mutex>
#include <opencv2/core/core.hpp>
#include <rosbag/bag.h>
#include <thread>
#include <unordered_map>

#include "rovio/BackendMsg.h"
#include "rovio/Camera.hpp"
#include "rovio/commonVision.hpp"

namespace rovio{

class BackendParams {
 public:
  template<int nCam> friend class Backend;
  typedef std::shared_ptr<BackendParams> Ptr;

  // Parameter which can be set via rovio.info
  bool active_ = false;                      /**<Turn Backend ON or OFF.*/
  bool storeMissionData_ = false;            /**<Store mission data to file.*/
  bool detectDrift_ = false;                 /**<Detect drift of rovio.*/
  double scoreDetectionExponent_ = 0.5;      /**<Influences the distribution of the mlp's into buckets. Choose between [0,1].*/
  int penaltyDistance_ = 20;                 /**<Features are punished (strength inter alia dependent of zeroDistancePenalty), if smaller distance to existing feature.*/
  int zeroDistancePenalty_ = 100;
  // Parameter which can not be set via rovio.info
  static constexpr int nMaxFeatures_ = 300;  /**<Maximal number of best features per frame.*/

 private:
  int nCam_ = -1;
};

template<int nCam>
class Backend {
 public:

  Backend(const std::shared_ptr<BackendParams>& params) {
    //Set parameters.
    params_ = params;
    params_->nCam_ = nCam;
    storageVQ_ = nullptr;
    missionStorageThread_ = nullptr;
  }

  ~Backend() {
    if (params_->active_ && params_->storeMissionData_) {
      bag_.close();
      delete storageVQ_;
      delete missionStorageThread_;
    }
  };

  void initialize() {

    // Feature: Store Mission Data
    if (params_->active_ && params_->storeMissionData_) {
      // Create path string and bagfile name.
      rovioPackagePath_ = ros::package::getPath("rovio");
      std::ostringstream bagName;
      std::time_t time = std::time(nullptr);
      char datestr[100];
      std::strftime(datestr, sizeof(datestr), "%F_%T", std::localtime(&time));
      bagName <<rovioPackagePath_<<"/storage/"<<"Mission_"<<datestr<<".bag";
      // Create bagfile and open it.
      bag_.open(bagName.str(), rosbag::bagmode::Write);
      // Create vertex queue for storage.
      storageVQ_ = new VertexQueue(-1);
      // Create and start storage thread.
      missionStorageThread_ = new std::thread(&rovio::Backend<nCam>::missionStorageThread, this);
    }

    // Feature: Detect Drift
    // ...
  };

  ///////////////////////
  // Params            //
  ///////////////////////
  std::shared_ptr<BackendParams> params_;

  ///////////////////////
  // Type Definitions  //
  ///////////////////////

  struct Feature {
   public:
    Feature() {
      camID_ = 0;
      nObservations_ = 0;
      globalID_ = 0;
      id_ = 0;
    };
    ~Feature(){};

    // Data Handling
    int camID_;
    int nObservations_;  /**<How many times the feature was seen before (not including the current observation).*/
    unsigned long globalID_;    /**<Global feature ID. Corresponding features have the same unique ID.*/

    // Feature Data
    double id_;    /**<Inverse depth.*/
    cv::Point2f c_;  /**<Pixel coordinate in vertical direction.*/
    LWF::NormalVectorElement CfP_;  /**<Bearing vector.*/
  };

  struct Vertex {
   public:
    Vertex() {
      mpCameras_ = nullptr;
      features_.reserve(BackendParams::nMaxFeatures_);
    };
    ~Vertex(){};

    // Camera parameters
    const Camera* mpCameras_;
    // State
    V3D WrWM_;
    QPD qWM_;
    V3D MvM_;
    V3D acb_;
    V3D gyb_;
    V3D MrMC_[nCam];
    QPD qCM_[nCam];
    // IMU Data
    V3D MaM_;
    V3D MwM_;
    // Feature Data
    std::unordered_multimap<unsigned long,std::shared_ptr<Feature>> features_;
  };

  struct BearingWithPose {
    V3D WrWM_;
    QPD qMW_;
    V3D MrMC_;
    QPD qCM_;
    LWF::NormalVectorElement CfP_;
  };

  template <int nLevels, int patchSize>
  struct FeatureTracking {
   public:
    FeatureTracking() {
      for (unsigned int i = 0; i < BackendParams::nMaxFeatures_; i++) {
        LWF::NormalVectorElement nor;
        nor.setFromVector(V3D(0,0,1));
        mlps_.features_[i].set_nor(nor);
        mlps_.features_[i].bearingCorners_[0].setZero();
        mlps_.features_[i].bearingCorners_[1].setZero();
        mlps_.features_[i].camID_ = 0;
        mlps_.features_[i].setDepth(1.0);
      }
    }
    BearingWithPose bearingsWithPosesAtInit_[BackendParams::nMaxFeatures_];  /**<Array, containing bearing vectors & poses at initialization.*/
    MultilevelPatchSet<nLevels, patchSize, BackendParams::nMaxFeatures_> mlps_;  /**<Current multilevel patch set, containing the tracked features.*/
  };

  class VertexQueue {
   public:
    VertexQueue() {nMax_ = 10;};
    VertexQueue(int nMax) {
      if(nMax <= 0) {
        nMax_ = queue_.max_size();
      }
      else {
        nMax_ = nMax;
      }
    };
    ~VertexQueue(){};

    // Data members
    int nMax_;

    // Functions
    void pushBack(const std::shared_ptr<Vertex>& vertex)
    {
      queue_.push_back(vertex);
      if(queue_.size() > nMax_)
        queue_.pop_front();
    }

    void popFront()
    {
      queue_.pop_front();
    }

    void popBack()
    {
      queue_.pop_back();
    }

    int getSize()
    {
      return this->queue_.size();
    };

    std::shared_ptr<Vertex> get (const unsigned int i)
    {
      if ( i < queue_.size()) {
        return queue_.at(i);
      }
      else {
        return std::shared_ptr<Vertex>(nullptr);
      }
    };

    std::shared_ptr<Vertex> getFront()
    {
      if (queue_.size() > 0) {
        return queue_.front();
      }
      else {
        return std::shared_ptr<Vertex>(nullptr);
      }
    };

    std::shared_ptr<Vertex> getBack()
    {
      if (queue_.size() > 0) {
        return queue_.back();
      }
      else {
        return std::shared_ptr<Vertex>(nullptr);
      }
    };

    typename std::deque<std::shared_ptr<Vertex>>::iterator getBeginIterator()
    {
      return queue_.begin();
    };

    typename std::deque<std::shared_ptr<Vertex>>::iterator getEndIterator()
    {
      return queue_.end();
    };

    bool isEmpty()
    {
      return queue_.empty();
    };

   private:
    std::deque<std::shared_ptr<Vertex>> queue_;
  };

  ///////////////////////
  // Data Members      //
  ///////////////////////
  std::string rovioPackagePath_;

  // Vertex Data Storage
  rosbag::Bag bag_;
  std::thread* missionStorageThread_;
  std::mutex mutexStorageVQ_;
  std::condition_variable condVarStorageVQ_;
  VertexQueue* storageVQ_;

  //////////////////////////////
  // Public Members Functions //
  //////////////////////////////
  void storeVertex(const std::shared_ptr<Vertex>& vertex)
  {
    std::lock_guard<std::mutex> lock(mutexStorageVQ_);
    storageVQ_->pushBack(vertex);
    condVarStorageVQ_.notify_one();
  }


 private:

  void missionStorageThread()
  {
    int i = 0;
    while (true) {

      // Get the front vertex of the storage vertex queue
      // and delete corresponding queue element afterwards.
      std::shared_ptr<Vertex> vertex;
      std::unique_lock<std::mutex> lock(mutexStorageVQ_);
      condVarStorageVQ_.wait(lock, [&]{return !storageVQ_->isEmpty();});
      vertex = storageVQ_->getBack();
      storageVQ_->popBack();
      lock.unlock();

      // Storage of the vertex data in the mission file.
      BackendMsg be_msg;
      be_msg.test = "Hallo" + std::to_string(i);
      bag_.write("mytopic", ros::Time::now(),be_msg);
      i++;
    }
  };
};

} // namespace rovio
#endif /* INCLUDE_ROVIO_BACKEND_HPP_ */
