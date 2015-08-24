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

#include <ros/package.h>

#include "rovio/rovioFilter.hpp"
#include "rovio/RovioNode.hpp"
#ifdef MAKE_SCENE
#include "rovio/RovioScene.hpp"
#endif

static constexpr unsigned int nMax_ = 25;    // Maximal number of considered features in the filter state.
static constexpr int nLevels_ = 4;           // Total number of pyramid levels considered.
static constexpr int patchSize_ = 8;         // Edge length of the patches (in pixel). Must be a multiple of 2!
static constexpr int nCam_ = 1;              // Used total number of cameras.
typedef rovio::RovioFilter<rovio::FilterState<nMax_,nLevels_,patchSize_,nCam_>> mtFilter;

#ifdef MAKE_SCENE
rovio::RovioScene<mtFilter> mRovioScene;

void idleFunc(){
  ros::spinOnce();
  mRovioScene.drawScene(mRovioScene.mpFilter_->safe_);
}
#endif

int main(int argc, char** argv){
  ros::init(argc, argv, "TestFilter");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  std::string rootdir = ros::package::getPath("rovio");

  // Filter
  mtFilter* mpFilter = new mtFilter;
  mpFilter->readFromInfo(rootdir + "/cfg/rovio.info");

  // Node
  rovio::RovioNode<mtFilter> rovioNode(nh, nh_private, mpFilter);
  rovioNode.makeTest();


#ifdef MAKE_SCENE
  // Scene
  std::string mVSFileName = rootdir + "/shaders/shader.vs";
  std::string mFSFileName = rootdir + "/shaders/shader.fs";
  mRovioScene.initScene(argc,argv,mVSFileName,mFSFileName,mpFilter);
  mRovioScene.setIdleFunction(idleFunc);
  mRovioScene.addKeyboardCB('r',[&rovioNode]() mutable {rovioNode.isInitialized_=false;});
  glutMainLoop();
#else
  ros::spin();
#endif

  delete mpFilter;
  return 0;
}
