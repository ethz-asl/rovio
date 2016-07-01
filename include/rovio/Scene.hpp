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

#ifndef ROVIO_SCENE_HPP_
#define ROVIO_SCENE_HPP_

#include "lightweight_filtering/common.hpp"
#include <opencv2/features2d/features2d.hpp>
#include "GL/glew.h"
#include <GL/freeglut.h>
#include <memory>
#include <fstream>
#include <functional>

namespace rovio{

struct PersProjInfo{
  float FOV_;
  float width_;
  float height_;
  float zNear_;
  float zFar_;
};

struct DirectionalLight
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3f color_;
  float ambientIntensity_;
  Eigen::Vector3f direction_;
  float diffuseIntensity_;
};


static bool ReadShaderFile(const char* pFileName, std::string& outFile){
  std::ifstream f(pFileName);
  bool ret = false;
  if (f.is_open()) {
      std::string line;
      while (getline(f, line)) {
          outFile.append(line);
          outFile.append("\n");
      }
      f.close();
      ret = true;
  }
  return ret;
}

static Eigen::Matrix4f InitTransform(const Eigen::Vector3f& v,const kindr::RotationQuaternionPF& q){
  Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
  m.block<3,3>(0,0) = kindr::RotationMatrixPF(q).matrix();
  m.block<3,1>(0,3) = -q.rotate(v);
  return m;
}

static Eigen::Matrix4f InitInverseTransform(const Eigen::Vector3f& v,const kindr::RotationQuaternionPF& q){
  Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
  m.block<3,3>(0,0) = kindr::RotationMatrixPF(q).inverted().matrix();
  m.block<3,1>(0,3) = v;
  return m;
}

static Eigen::Matrix4f InitPersProjTransform(const PersProjInfo& p){
  Eigen::Matrix4f m;
  const float ar         = p.width_ / p.height_;
  const float zRange     = p.zFar_ - p.zNear_;
  const float tanHalfFOV = tanf(p.FOV_ / 2.0f);

  m(0,0) = 1.0f/(tanHalfFOV * ar); m(0,1) = 0.0f;            m(0,2) = 0.0f;            m(0,3) = 0.0;
  m(1,0) = 0.0f;                   m(1,1) = 1.0f/tanHalfFOV; m(1,2) = 0.0f;            m(1,3) = 0.0;
  m(2,0) = 0.0f;                   m(2,1) = 0.0f;            m(2,2) = (-p.zNear_ - p.zFar_)/zRange ; m(2,3) = 2.0f*p.zFar_*p.zNear_/zRange;
  m(3,0) = 0.0f;                   m(3,1) = 0.0f;            m(3,2) = -1.0f;            m(3,3) = 0.0;
  return m;
}

template<int N>
struct Arrayf{
  Arrayf(){
    for(int i=0;i<N;i++){
      data_[i] = 0.0f;
    }
  }
  Arrayf(const Eigen::Matrix<float,N,1>& in){
    fromEigen(in);
  }
  void fromEigen(const Eigen::Matrix<float,N,1>& in){
    for(int i=0;i<N;i++){
      data_[i] = in(i);
    }
  }
  Eigen::Matrix<float,N,1> toEigen(){
    Eigen::Matrix<float,N,1> out;
    for(int i=0;i<N;i++){
      out(i) = data_[i];
    }
    return out;
  }
  float data_[N];
};

struct Vertex
{
  Vertex():
    pos_(Eigen::Vector3f(0.0f,0.0f,0.0f)),
    normal_(Eigen::Vector3f(0.0f,0.0f,1.0f)),
    color_(Eigen::Vector4f(1.0f,1.0f,1.0f,1.0f)){}
  Vertex(float x, float y, float z):
    pos_(Eigen::Vector3f(x,y,z)),
    normal_(Eigen::Vector3f(0.0f,0.0f,1.0f)),
    color_(Eigen::Vector4f(1.0f,1.0f,1.0f,1.0f)){}
  Vertex(const Eigen::Vector3f& vec):
    pos_(vec),
    normal_(Eigen::Vector3f(0.0f,0.0f,1.0f)),
    color_(Eigen::Vector4f(1.0f,1.0f,1.0f,1.0f)){}
  Arrayf<3> pos_;
  Arrayf<3> normal_;
  Arrayf<4> color_;
};

class SceneObject{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  SceneObject();
  virtual ~SceneObject();
  void initTexture(const int& cols, const int& rows);
  void setTexture(const int& cols, const int& rows, const float* ptr);
  void setTexture(const cv::Mat& img);
  void allocateBuffer();
  void makeCubeFull(const float& l);
  void makeTetrahedron(const float& l);
  void makeCubeMesh(const float& l);
  void makeLine();
  void makeLine(const std::vector<Eigen::Vector3f>& line);
  void makePoints();
  void makePoints(const std::vector<Eigen::Vector3f>& points);
  void prolonge(const Eigen::Vector3f& point);
  void prolonge(const std::vector<Eigen::Vector3f>& points);
  void clear();
  void makeCoordinateFrame(const float& l);
  void makeRectangle(const float& x,const float& y);
  void makeTexturedRectangle(const float& x,const float& y);
  void makeGroundPlane(const float& l,const int& N,const Eigen::Vector4f& color1,const Eigen::Vector4f& color2);
  void makeGroundPlaneMesh(const float& l,const int& N);
  void setColorFull(const Eigen::Vector4f& color);
  const Eigen::Matrix4f GetWorldTrans();
  void CalcNormals();
  void loadOptions();
  Eigen::Vector3f W_r_WB_;
  kindr::RotationQuaternionPF q_BW_;
  std::vector<Vertex> vertices_;
  std::vector<unsigned int> indices_;
  GLuint VBO_;
  GLuint IBO_;
  GLuint textureID_;
  GLenum mode_;
  bool mbCullFace_;
  float lineWidth_;
  float pointSize_;
  bool useTexture_;
  bool draw_;
};

class Scene{
 public:
  std::map<unsigned char, std::function<void()>> keyboardCallbacks_;
  std::map<int, std::function<void()>> specialKeyboardCallbacks_;
  Scene();
  virtual ~Scene();
  int init(int argc, char** argv, const std::string& mVSFileName,const std::string& mFSFileName);
  std::shared_ptr<SceneObject> addSceneObject();
  void makeTestScene();
  void RenderSceneCB();
  void setView(const Eigen::Vector3f& pos, const Eigen::Vector3f& target);
  void setYDown(float step = 0.1);
  void SpecialKeyboardCB(int Key, int x, int y);
  void addSpecialKeyboardCB(int Key, std::function<void()> f);
  void KeyboardCB(unsigned char Key, int x, int y);
  void addKeyboardCB(unsigned char Key, std::function<void()> f);
  void MotionCB(int x, int y);
  void MouseCB(int button, int state, int x, int y);
  void AddShader(GLuint ShaderProgram, const char* pShaderText, GLenum ShaderType);
  void CompileShaders(const std::string& mVSFileName,const std::string& mFSFileName);
  void setIdleFunction(void (*idleFunc)());
  void (*mIdleFunc)();
 private:
  float stepScale_;

  std::vector<std::shared_ptr<SceneObject>> mSceneObjects_;

  GLuint V_TF_B_location_;
  GLuint W_TF_B_location_;
  GLuint lightColor_location_;
  GLuint lightAmbientIntensity_location_;
  GLuint lightDiffuseIntensity_location_;
  GLuint lightDirection_location_;
  GLuint useTexture_location_;
  GLuint sampler_location_;

  PersProjInfo mPersProjInfo_;
  DirectionalLight mDirectionalLight_;

  Eigen::Vector3f W_r_WC_;
  kindr::RotationQuaternionPF q_CW_;

  Eigen::Matrix4f C_TF_W_;
  Eigen::Matrix4f V_TF_C_;
  Eigen::Matrix4f V_TF_B_;
  Eigen::Matrix4f W_TF_B_;

  Eigen::Vector2i mMousePos_;
  bool enableMouseMotion_;
};

static Scene* mpScene = nullptr;

static void initGlut(int argc, char** argv, Scene& scene){
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_DOUBLE|GLUT_RGBA);
  glutInitWindowSize(1600, 900);
  glutInitWindowPosition(100, 100);
  glutCreateWindow("Scene");

  mpScene = &scene;
  glutDisplayFunc([](){mpScene->RenderSceneCB();});
  glutIdleFunc([](){
    if(mpScene->mIdleFunc != nullptr){
      mpScene->mIdleFunc();
    }
    mpScene->RenderSceneCB();
  });
  glutSpecialFunc([](int Key, int x, int y){mpScene->SpecialKeyboardCB(Key,x,y);});
  glutMotionFunc([](int x, int y){mpScene->MotionCB(x,y);});
  glutMouseFunc([](int button, int state, int x, int y){mpScene->MouseCB(button,state,x,y);});
  glutKeyboardFunc([](unsigned char Key, int x, int y){mpScene->KeyboardCB(Key,x,y);});
}

}


#endif /* ROVIO_SCENE_HPP_ */
