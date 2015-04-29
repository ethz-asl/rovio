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

#include "rovio/RovioNode.hpp"
#include "rovio/Scene.hpp"

rovio::Scene mScene;

int main(int argc, char** argv){
  ros::init(argc, argv, "TestFilter");
  ros::NodeHandle nh;
  rovio::RovioNode rovioNode(nh);

  initGlut(argc,argv,mScene);

  std::string rootdir = ros::package::getPath("rovio");
  std::string mVSFileName = rootdir + "/shaders/shader.vs";
  std::string mFSFileName = rootdir + "/shaders/shader.fs";

  mScene.init(argc, argv,mVSFileName,mFSFileName);

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
  rovioNode.mpSensor_ = mScene.addSceneObject();
  rovioNode.mpSensor_->makeCoordinateFrame(1.0f);
  rovioNode.mpSensor_->lineWidth_ = 5.0f;

  for(int i=0;i<TestFilter::mtState::nMax_;i++){
    rovioNode.mpPatches_[i] = mScene.addSceneObject();
  }

  rovioNode.mpLines_ = mScene.addSceneObject();
  rovioNode.mpLines_->makeLine();
  rovioNode.mpLines_->lineWidth_ = 2.0f;
  rovioNode.mpLines_->mode_ = GL_LINES;

  rovioNode.mpDepthVar_ = mScene.addSceneObject();
  rovioNode.mpDepthVar_->makeLine();
  rovioNode.mpDepthVar_->lineWidth_ = 3.0f;
  rovioNode.mpDepthVar_->mode_ = GL_LINES;

  mScene.setView(Eigen::Vector3f(-5.0f,-5.0f,5.0f),Eigen::Vector3f(0.0f,0.0f,0.0f));
  mScene.setYDown();
  mScene.setIdleFunction(ros::spinOnce);
  glutMainLoop();

  return 0;
}
