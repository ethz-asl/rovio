/*
 * Backend.hpp
 *
 *  Created on: Jun 17, 2015
 *      Author: andreas
 */

#ifndef INCLUDE_ROVIO_BACKEND_HPP_
#define INCLUDE_ROVIO_BACKEND_HPP_

#include <condition_variable>
#include <deque>
#include <mutex>
#include <rosbag/bag.h>
#include <thread>
#include <unordered_map>

#include <rovio/BackendMsg.h>

namespace rovio{

class Backend {
 public:

  // Parameters
  static constexpr bool active_ = true;  /**<Turn Backend ON or OFF.*/
  static constexpr bool storeMissionData_ = true;  /**<Store mission data to file.*/
  static constexpr bool detectDrift_ = false;  /**<Detect drift of rovio.*/

  static constexpr int nCam_ = 1;  // Number of cameras.
  static constexpr int nMaxFeatures_ = 300; // Maximal number of best features per frame.
  static constexpr float scoreDetectionExponent_ = 0.5;  // Influences the distribution of the mlp's into buckets. Choose between [0,1].
  static constexpr int penaltyDistance_ = 20;  // Features are punished (strength inter alia dependent of zeroDistancePenalty), if smaller distance to existing feature.
  static constexpr int zeroDistancePenalty_ = 100;

  // Constructor
  Backend() {

    if (active_ && storeMissionData_) {
      bag_.open("test.bag", rosbag::bagmode::Write);
      storageVQ_ = new VertexQueue(-1);
      missionStorageThread_ = new std::thread(&rovio::Backend::missionStorageThread, this);
    }

  };

  ~Backend() {

    if (active_ && storeMissionData_) {
      bag_.close();
    }
    delete storageVQ_;
    delete missionStorageThread_;

  };

  ///////////////////////
  // Type Definitions  //
  ///////////////////////
  struct Feature {
   public:
    typedef std::shared_ptr<Feature> Ptr;
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
    typedef std::shared_ptr<Vertex> Ptr;
    Vertex() {
      mpCameras_ = nullptr;
      features_.reserve(nMaxFeatures_);
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
    V3D MrMC_[nCam_];
    QPD qCM_[nCam_];
    // IMU Data
    V3D MaM_;
    V3D MwM_;
    // Feature Data
    std::unordered_multimap<unsigned long, Feature::Ptr> features_;
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
      for (unsigned int i=0; i<nMaxFeatures_; i++) {
        LWF::NormalVectorElement nor;
        nor.setFromVector(V3D(0,0,1));
        mlps_.features_[i].set_nor(nor);
        mlps_.features_[i].bearingCorners_[0].setZero();
        mlps_.features_[i].bearingCorners_[1].setZero();
        mlps_.features_[i].camID_ = 0;
        mlps_.features_[i].setDepth(1.0);
      }
    }
    BearingWithPose bearingsWithPosesAtInit_[nMaxFeatures_];  /**<Array, containing bearing vectors & poses at initialization.*/
    MultilevelPatchSet<nLevels, patchSize, nMaxFeatures_> mlps_;  /**<Current multilevel patch set, containing the tracked features.*/
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
    void pushBack(const Vertex::Ptr& vertex)
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

    Vertex::Ptr get (const unsigned int i)
    {
      if ( i < queue_.size()) {
        return queue_.at(i);
      }
      else {
        return Vertex::Ptr(nullptr);
      }
    };

    Vertex::Ptr getFront()
    {
      if (queue_.size() > 0) {
        return queue_.front();
      }
      else {
        return Vertex::Ptr(nullptr);
      }
    };

    Vertex::Ptr getBack()
    {
      if (queue_.size() > 0) {
        return queue_.back();
      }
      else {
        return Vertex::Ptr(nullptr);
      }
    };

    std::deque<Vertex::Ptr>::iterator getBeginIterator()
    {
      return queue_.begin();
    };

    std::deque<Vertex::Ptr>::iterator getEndIterator()
    {
      return queue_.end();
    };

    bool isEmpty()
    {
      return queue_.empty();
    };

   private:
    std::deque<Vertex::Ptr> queue_;
  };

  ///////////////////////
  // Data Members      //
  ///////////////////////

  // Vertex Data Storage
  rosbag::Bag bag_;
  std::thread* missionStorageThread_;
  std::mutex mutexStorageVQ_;
  std::condition_variable condVarStorageVQ_;
  VertexQueue* storageVQ_;

  //////////////////////////////
  // Public Members Functions //
  //////////////////////////////
  void storeVertex(const Vertex::Ptr& vertex)
  {
    std::lock_guard<std::mutex> lock(mutexStorageVQ_);
    storageVQ_->pushBack(vertex);
    condVarStorageVQ_.notify_one();
  }


 private:

  void missionStorageThread()
  {
    while (true) {

      // Get the front vertex of the storage vertex queue
      // and delete corresponding queue element afterwards.
      Vertex::Ptr vertex;
      std::unique_lock<std::mutex> lock(mutexStorageVQ_);
      condVarStorageVQ_.wait(lock, [&]{return !storageVQ_->isEmpty();});
      vertex = storageVQ_->getBack();
      storageVQ_->popBack();
      lock.unlock();

      // Storage of the vertex data in the mission file.

    }
 };
};



} // namespace rovio
#endif /* INCLUDE_ROVIO_BACKEND_HPP_ */
