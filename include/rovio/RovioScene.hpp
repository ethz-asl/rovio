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

#ifndef ROVIO_ROVIOSCENE_HPP_
#define ROVIO_ROVIOSCENE_HPP_

#include "kindr/rotations/RotationEigen.hpp"
#include <Eigen/Dense>

#include "commonVision.hpp"
#include "rovio/Scene.hpp"

namespace rot = kindr::rotations::eigen_impl;

namespace rovio {

template<typename FILTER>
class RovioScene{
 public:
  typedef typename FILTER::mtFilterState mtFilterState;
  typedef typename mtFilterState::mtState mtState;
  FILTER* mpFilter_;

  rovio::Scene mScene;
  SceneObject* mpSensor_[mtState::nCam_];
  SceneObject* mpGroundtruth_;
  SceneObject* mpLines_[mtState::nCam_];
  SceneObject* mpDepthVar_[mtState::nCam_];
  SceneObject* mpPatches_[mtState::nMax_];
  RovioScene(){
    mpFilter_ = nullptr;
    for(int camID=0;camID<mtState::nCam_;camID++){
      mpSensor_[camID] = nullptr;
      mpGroundtruth_ = nullptr;
      mpLines_[camID] = nullptr;
      mpDepthVar_[camID] = nullptr;
    }
    for(int i=0;i<mtState::nMax_;i++){
      mpPatches_[i] = nullptr;
    }
  }
  void addKeyboardCB(unsigned char Key, std::function<void()> f){
    mScene.addKeyboardCB(Key,f);
  }
  void addSpecialKeyboardCB(int Key, std::function<void()> f){
    mScene.addSpecialKeyboardCB(Key,f);
  }
  void initScene(int argc, char** argv, const std::string& mVSFileName,const std::string& mFSFileName,FILTER* mpFilter){
    initGlut(argc,argv,mScene);
    mScene.init(argc, argv,mVSFileName,mFSFileName);
    mpFilter_ = mpFilter;

    rovio::SceneObject* mpGroundplane1 = mScene.addSceneObject();
    mpGroundplane1->makeGroundPlaneMesh(0.25,40);
    mpGroundplane1->setColorFull(Eigen::Vector4f(0.6f,0.6f,0.6f,1.0f));
    mpGroundplane1->lineWidth_ = 1.0f;
    mpGroundplane1->W_r_WB_(2) = -1.0f;
    rovio::SceneObject* mpGroundplane2 = mScene.addSceneObject();
    mpGroundplane2->makeGroundPlaneMesh(1.0,10);
    mpGroundplane2->setColorFull(Eigen::Vector4f(0.8f,0.8f,0.8f,1.0f));
    mpGroundplane2->lineWidth_ = 3.0f;
    mpGroundplane2->W_r_WB_(2) = -1.0f;
    for(int camID=0;camID<mtState::nCam_;camID++){
      mpSensor_[camID] = mScene.addSceneObject();
      mpSensor_[camID]->makeCoordinateFrame(1.0f);
      mpSensor_[camID]->lineWidth_ = 5.0f;

      mpLines_[camID] = mScene.addSceneObject();
      mpLines_[camID]->makeLine();
      mpLines_[camID]->lineWidth_ = 2.0f;
      mpLines_[camID]->mode_ = GL_LINES;

      mpDepthVar_[camID] = mScene.addSceneObject();
      mpDepthVar_[camID]->makeLine();
      mpDepthVar_[camID]->lineWidth_ = 3.0f;
      mpDepthVar_[camID]->mode_ = GL_LINES;
    }
    mpGroundtruth_ = mScene.addSceneObject();
    mpGroundtruth_->makeCoordinateFrame(1.0f);
    mpGroundtruth_->lineWidth_ = 5.0f;
    Eigen::Vector4f color;
    for(int i=0;i<3;i++){
      color = Eigen::Vector4f(0.0f,0.0f,0.0f,1.0f);
      color(i) = 0.6f;
      mpGroundtruth_->vertices_[i*2].color_.fromEigen(color);
      mpGroundtruth_->vertices_[i*2+1].color_.fromEigen(color);
    }
    mpGroundtruth_->allocateBuffer();
    for(int i=0;i<mtState::nMax_;i++){
      mpPatches_[i] = mScene.addSceneObject();
    }

    mScene.setView(Eigen::Vector3f(-5.0f,-5.0f,5.0f),Eigen::Vector3f(0.0f,0.0f,0.0f));
    mScene.setYDown();
  }
  void setIdleFunction(void (*idleFunc)()){
    mScene.setIdleFunction(idleFunc);
  }
  void drawScene(mtFilterState& filterState){
    const mtState& state = filterState.state_;
    const typename mtFilterState::mtFilterCovMat& cov = filterState.cov_;

    if(filterState.plotGroundtruth_){
      mpGroundtruth_->q_BW_ = filterState.groundtruth_qCB_.inverted()*filterState.groundtruth_qCJ_*filterState.groundtruth_qJI_;
      mpGroundtruth_->W_r_WB_ = (filterState.groundtruth_IrIJ_ + filterState.groundtruth_qJI_.inverseRotate(filterState.groundtruth_JrJC_)
                                - (filterState.groundtruth_qCB_.inverted()*filterState.groundtruth_qCJ_*filterState.groundtruth_qJI_).inverseRotate(filterState.groundtruth_BrBC_)).template cast<float>();
      mpGroundtruth_->draw_ = true;
    } else {
      mpGroundtruth_->draw_ = false;
    }

    for(unsigned int camID=0;camID<mtState::nCam_;camID++){
      mpSensor_[camID]->W_r_WB_ = state.WrWC(camID).template cast<float>();
      mpSensor_[camID]->q_BW_ = state.qCW(camID);

      std::vector<Eigen::Vector3f> points;
      std::vector<Eigen::Vector3f> lines;
      mpLines_[camID]->clear();
      mpDepthVar_[camID]->clear();
      double d, d_far, d_near, d_p, p_d, p_d_p;
      const double stretchFactor = mtState::patchSize_*std::pow(2.0,mtState::nLevels_-1)/(filterState.mlps_.features_[0].warpDistance_*2.0);
      for(unsigned int i=0;i<mtState::nMax_;i++){
        if(filterState.mlps_.isValid_[i] && filterState.mlps_.features_[i].camID_ == camID){
          state.aux().depthMap_.map(filterState.state_.dep(i),d,d_p,p_d,p_d_p);
          const double sigma = cov(mtState::template getId<mtState::_dep>(i),mtState::template getId<mtState::_dep>(i));
          state.aux().depthMap_.map(filterState.state_.dep(i)-sigma,d_far,d_p,p_d,p_d_p);
          if(state.aux().depthMap_.type_ == DepthMap::INVERSE && (d_far > 1000 || d_far <= 0.0)) d_far = 1000;
          state.aux().depthMap_.map(filterState.state_.dep(i)+sigma,d_near,d_p,p_d,p_d_p);
          const LWF::NormalVectorElement middle = filterState.state_.CfP(i);
          LWF::NormalVectorElement corner[4];
          Eigen::Vector3d cornerVec[4];
          const BearingCorners& bearingCorners = filterState.mlps_.features_[i].get_bearingCorners();
          for(int x=0;x<2;x++){
            for(int y=0;y<2;y++){
              const Eigen::Vector2d dif = stretchFactor*((2*x-1)*filterState.state_.aux().bearingCorners_[i][0]+(2*y-1)*filterState.state_.aux().bearingCorners_[i][1]); // TODO: factor 4
              middle.boxPlus(dif,corner[y*2+x]);
              cornerVec[y*2+x] = corner[y*2+x].getVec()*d;
            }
          }
          const Eigen::Vector3d pos = middle.getVec()*d;
          const Eigen::Vector3d pos_far = middle.getVec()*d_far;
          const Eigen::Vector3d pos_near = middle.getVec()*d_near;

          mpLines_[camID]->prolonge(cornerVec[0].cast<float>());
          mpLines_[camID]->prolonge(cornerVec[1].cast<float>());
          mpLines_[camID]->prolonge(cornerVec[1].cast<float>());
          mpLines_[camID]->prolonge(cornerVec[3].cast<float>());
          mpLines_[camID]->prolonge(cornerVec[3].cast<float>());
          mpLines_[camID]->prolonge(cornerVec[2].cast<float>());
          mpLines_[camID]->prolonge(cornerVec[2].cast<float>());
          mpLines_[camID]->prolonge(cornerVec[0].cast<float>());

          mpDepthVar_[camID]->prolonge(pos_far.cast<float>());
          mpDepthVar_[camID]->prolonge(pos_near.cast<float>());
          if(filterState.mlps_.features_[i].status_.inFrame_){
            if(filterState.mlps_.features_[i].status_.trackingStatus_ == TRACKED){
              std::next(mpDepthVar_[camID]->vertices_.rbegin())->color_.fromEigen(Eigen::Vector4f(0.0f,1.0f,0.0f,1.0f));
              mpDepthVar_[camID]->vertices_.rbegin()->color_.fromEigen(Eigen::Vector4f(0.0f,1.0f,0.0f,1.0f));
              for(int j=0;j<8;j++){
                std::next(mpLines_[camID]->vertices_.rbegin(),j)->color_.fromEigen(Eigen::Vector4f(0.0f,1.0f,0.0f,1.0f));
              }
            } else {
              std::next( mpDepthVar_[camID]->vertices_.rbegin())->color_.fromEigen(Eigen::Vector4f(1.0f,0.0f,0.0f,1.0f));
              mpDepthVar_[camID]->vertices_.rbegin()->color_.fromEigen(Eigen::Vector4f(1.0f,0.0f,0.0f,1.0f));
              for(int j=0;j<8;j++){
                std::next(mpLines_[camID]->vertices_.rbegin(),j)->color_.fromEigen(Eigen::Vector4f(1.0f,0.0f,0.0f,1.0f));
              }
            }
          } else {
            std::next( mpDepthVar_[camID]->vertices_.rbegin())->color_.fromEigen(Eigen::Vector4f(0.5f,0.5f,0.5f,1.0f));
            mpDepthVar_[camID]->vertices_.rbegin()->color_.fromEigen(Eigen::Vector4f(0.5f,0.5f,0.5f,1.0f));
            for(int j=0;j<8;j++){
              std::next(mpLines_[camID]->vertices_.rbegin(),j)->color_.fromEigen(Eigen::Vector4f(0.5f,0.5f,0.5f,1.0f));
            }
          }

          mpPatches_[i]->clear();
          mpPatches_[i]->makeTexturedRectangle(1.0f,1.0f);
          cv::Mat patch = cv::Mat::zeros(mtState::patchSize_*pow(2,mtState::nLevels_-1),mtState::patchSize_*pow(2,mtState::nLevels_-1),CV_8UC1);
          filterState.mlps_.features_[i].drawMultilevelPatch(patch,cv::Point2i(0,0),1,false);
          mpPatches_[i]->setTexture(patch);
          for(int x=0;x<2;x++){
            for(int y=0;y<2;y++){
              mpPatches_[i]->vertices_[y*2+x].pos_.fromEigen(cornerVec[y*2+x].cast<float>());
            }
          }
          mpPatches_[i]->allocateBuffer();
          mpPatches_[i]->W_r_WB_ = mpSensor_[camID]->W_r_WB_;
          mpPatches_[i]->q_BW_ = mpSensor_[camID]->q_BW_;
        }
        mpLines_[camID]->W_r_WB_ = mpSensor_[camID]->W_r_WB_;
        mpLines_[camID]->q_BW_ = mpSensor_[camID]->q_BW_;
        mpLines_[camID]->allocateBuffer();
        mpDepthVar_[camID]->W_r_WB_ = mpSensor_[camID]->W_r_WB_;
        mpDepthVar_[camID]->q_BW_ = mpSensor_[camID]->q_BW_;
        mpDepthVar_[camID]->allocateBuffer();
      }
    }
  }
};

}


#endif /* ROVIO_ROVIOSCENE_HPP_ */
