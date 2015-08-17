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

#include <functional>
#include <Eigen/Dense>
#include "lightweight_filtering/common.hpp"

#include "GL/glew.h"
#include <GL/freeglut.h>

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
    Eigen::Vector3f color_;
    float ambientIntensity_;
    Eigen::Vector3f direction_;
    float diffuseIntensity_;
};


bool ReadShaderFile(const char* pFileName, std::string& outFile){
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

static Eigen::Matrix4f InitTransform(const Eigen::Vector3f& v,const rot::RotationQuaternionPF& q){
  Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
  m.block<3,3>(0,0) = rot::RotationMatrixPF(q).matrix();
  m.block<3,1>(0,3) = -q.rotate(v);
  return m;
}

static Eigen::Matrix4f InitInverseTransform(const Eigen::Vector3f& v,const rot::RotationQuaternionPF& q){
  Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
  m.block<3,3>(0,0) = rot::RotationMatrixPF(q).inverted().matrix();
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
  SceneObject(){
    glGenBuffers(1, &VBO_);
    glGenBuffers(1, &IBO_);
    glGenTextures(1, &textureID_);
    glBindTexture(GL_TEXTURE_2D, textureID_);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    W_r_WB_   = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    q_BW_.setIdentity();
    mode_ = GL_TRIANGLES;
    mbCullFace_ = true;
    lineWidth_ = 2.0f;
    pointSize_ = 5.0f;
    useTexture_ = false;
    draw_ = true;
  }
  void setTexture(const int& cols, const int& rows, const float* ptr){
    glBindTexture(GL_TEXTURE_2D, textureID_);
    glTexImage2D(GL_TEXTURE_2D,0,GL_LUMINANCE,cols,rows,0,GL_LUMINANCE,GL_FLOAT,ptr);
  }
  void setTexture(const cv::Mat& img){
    glBindTexture(GL_TEXTURE_2D, textureID_);
    glTexImage2D(GL_TEXTURE_2D,0,GL_LUMINANCE,img.cols,img.rows,0,GL_LUMINANCE,GL_UNSIGNED_BYTE,img.data);
  }
  void allocateBuffer(){
    glBindBuffer(GL_ARRAY_BUFFER, VBO_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(Vertex)*vertices_.size(), vertices_.data(), GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, IBO_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int)*indices_.size(), indices_.data(), GL_DYNAMIC_DRAW);
  }
  void makeCubeFull(const float& l){
    clear();
    vertices_.reserve(24);
    indices_.reserve(36);
    Eigen::Vector3f d1;
    Eigen::Vector3f d2;
    Eigen::Vector3f d3;
    int count = 0;
    for(int i=0;i<3;i++){
      d1.setZero();
      d1(i) = 1.0;
      d2.setZero();
      d2((i+1)%3) = 1.0;
      d3 = d1.cross(d2);
      for(int z=0;z<2;z++){
        for(int x=0;x<2;x++){
          for(int j=0;j<2;j++){
            int y = (x+j+z+1)%2;
            vertices_.push_back(Vertex(((2*z-1)*d1+(2*x-1)*d2+(2*y-1)*d3)*0.5*l));
          }
        }
        indices_.push_back(count);
        indices_.push_back(count+2);
        indices_.push_back(count+1);
        indices_.push_back(count);
        indices_.push_back(count+3);
        indices_.push_back(count+2);
        count +=4;
      }
    }
    CalcNormals();
    allocateBuffer();
    mode_ = GL_TRIANGLES;
    mbCullFace_ = true;
  }
  void makeTetrahedron(const float& l){
    clear();
    vertices_.reserve(12);
    indices_.reserve(12);
    Eigen::Vector3f corners[4];
    corners[0] = Eigen::Vector3f(1.0f,0.0f,-0.707f);
    corners[1] = Eigen::Vector3f(-1.0f,0.0f,-0.707f);
    corners[2] = Eigen::Vector3f(0.0f,1.0f,0.707f);
    corners[3] = Eigen::Vector3f(0.0f,-1.0f,0.707f);
    int count = 0;
    for(int i=0;i<4;i++){
      for(int j=0;j<3;j++){
        vertices_.push_back(Eigen::Vector3f(corners[(i+j)%4]*0.5*l));
      }
      indices_.push_back(count+0+i%2);
      indices_.push_back(count+1-i%2);
      indices_.push_back(count+2);
      count += 3;
    }
    CalcNormals();
    allocateBuffer();
    mode_ = GL_TRIANGLES;
    mbCullFace_ = true;
  }
  void makeCubeMesh(const float& l){
    clear();
    vertices_.reserve(8);
    indices_.reserve(12);
    for(int x=0;x<2;x++){
      for(int y=0;y<2;y++){
        for(int z=0;z<2;z++){
          vertices_.push_back(Vertex((2*x-1)*0.5*l,(2*y-1)*0.5*l,(2*z-1)*0.5*l));
        }
      }
    }
    int count = 0;
    for(int i=0;i<3;i++){
      for(int j=0;j<4;j++){
        indices_.push_back((j*(int)pow(2,i))%7);
        indices_.push_back((j*(int)pow(2,i))%7+pow(2,(i+2)%3));
      }
    }
    allocateBuffer();
    mode_ = GL_LINES;
  }
  void makeLine(){
    std::vector<Eigen::Vector3f> emptyLine;
    makeLine(emptyLine);
  }
  void makeLine(const std::vector<Eigen::Vector3f>& line){
    clear();
    prolonge(line);
    mode_ =  GL_LINE_STRIP;
  }
  void makePoints(){
    std::vector<Eigen::Vector3f> empty;
    makePoints(empty);
  }
  void makePoints(const std::vector<Eigen::Vector3f>& points){
    clear();
    prolonge(points);
    mode_ =  GL_POINTS;
  }
  void prolonge(const Eigen::Vector3f& point){
    std::vector<Eigen::Vector3f> line(1,point);
    prolonge(line);
  }
  void prolonge(const std::vector<Eigen::Vector3f>& points){
    vertices_.reserve(vertices_.size()+points.size());
    indices_.reserve(indices_.size()+points.size());
    const int s = vertices_.size();
    for(int i=0;i<points.size();i++){
      vertices_.push_back(points[i]);
      indices_.push_back(s+i);
    }
    allocateBuffer();
  }
  void clear(){
    vertices_.clear();
    indices_.clear();
  }
  void makeCoordinateFrame(const float& l){
    clear();
    vertices_.reserve(6);
    indices_.reserve(6);
    Eigen::Vector3f d;
    Eigen::Vector4f color;
    for(int i=0;i<3;i++){
      color = Eigen::Vector4f(0.0f,0.0f,0.0f,1.0f);
      color(i) = 1.0f;
      d.setZero();
      vertices_.push_back(d);
      vertices_[i*2].color_.fromEigen(color);
      indices_.push_back(i*2);
      d(i) = l;
      vertices_.push_back(d);
      vertices_[i*2+1].color_.fromEigen(color);
      indices_.push_back(i*2+1);
    }
    allocateBuffer();
    mode_ = GL_LINES;
  }
  void makeRectangle(const float& x,const float& y){
    clear();
    vertices_.reserve(4);
    indices_.reserve(6);
    vertices_.push_back(Eigen::Vector3f(-0.5*x,-0.5*y,0.0f));
    vertices_.push_back(Eigen::Vector3f(0.5*x,-0.5*y,0.0f));
    vertices_.push_back(Eigen::Vector3f(-0.5*x,0.5*y,0.0f));
    vertices_.push_back(Eigen::Vector3f(0.5*x,0.5*y,0.0f));
    indices_.push_back(0);
    indices_.push_back(1);
    indices_.push_back(2);
    indices_.push_back(3);
    indices_.push_back(1);
    indices_.push_back(2);
    allocateBuffer();
    mode_ = GL_TRIANGLES;
    mbCullFace_ = false;
  }
  void makeTexturedRectangle(const float& x,const float& y){
    makeRectangle(x,y);
    vertices_[0].color_.data_[0] = 0.0f;
    vertices_[0].color_.data_[1] = 0.0f;
    vertices_[1].color_.data_[0] = 1.0f;
    vertices_[1].color_.data_[1] = 0.0f;
    vertices_[2].color_.data_[0] = 0.0f;
    vertices_[2].color_.data_[1] = 1.0f;
    vertices_[3].color_.data_[0] = 1.0f;
    vertices_[3].color_.data_[1] = 1.0f;
    useTexture_ = true;
    allocateBuffer();
  }
  void makeGroundPlane(const float& l,const int& N,const Eigen::Vector4f& color1,const Eigen::Vector4f& color2){
    Eigen::Vector3f globalCorner(-0.5*l*N,-0.5*l*N,0.0f);
    clear();
    vertices_.reserve(4*N*N);
    indices_.reserve(6*N*N);
    int count = 0;
    for(int x=0;x<N;x++){
      for(int y=0;y<N;y++){
        Eigen::Vector3f corner = globalCorner+Eigen::Vector3f(l*x,l*y,0.0f);
        for(int i=0;i<2;i++){
          for(int j=0;j<2;j++){
            vertices_.push_back(Eigen::Vector3f(corner+Eigen::Vector3f(l*i,l*j,0.0f)));
            if((x+y)%2==0){
              vertices_.rbegin()->color_ = color1;
            } else {
              vertices_.rbegin()->color_ = color2;
            }
          }
        }
        indices_.push_back(count+0);
        indices_.push_back(count+1);
        indices_.push_back(count+2);
        indices_.push_back(count+3);
        indices_.push_back(count+1);
        indices_.push_back(count+2);
        count += 4;
      }
    }
    allocateBuffer();
    mode_ = GL_TRIANGLES;
    mbCullFace_ = false;
  }
  void makeGroundPlaneMesh(const float& l,const int& N){
    Eigen::Vector3f globalCorner(-0.5*l*N,-0.5*l*N,0.0f);
    clear();
    vertices_.reserve(4*(N+1));
    indices_.reserve(4*(N+1));
    for(int i=0;i<N+1;i++){
      vertices_.push_back(Eigen::Vector3f(globalCorner+Eigen::Vector3f(l*i,0.0f,0.0f)));
      vertices_.push_back(Eigen::Vector3f(globalCorner+Eigen::Vector3f(l*i,l*N,0.0f)));
      vertices_.push_back(Eigen::Vector3f(globalCorner+Eigen::Vector3f(0.0f,l*i,0.0f)));
      vertices_.push_back(Eigen::Vector3f(globalCorner+Eigen::Vector3f(l*N,l*i,0.0f)));
      indices_.push_back(i*4+0);
      indices_.push_back(i*4+1);
      indices_.push_back(i*4+2);
      indices_.push_back(i*4+3);
    }
    allocateBuffer();
    mode_ = GL_LINES;
  }
  void setColorFull(const Eigen::Vector4f& color){
    for (unsigned int i = 0 ; i < vertices_.size() ; i++) {
      vertices_[i].color_.fromEigen(color);
    }
    allocateBuffer();
  }
  const Eigen::Matrix4f GetWorldTrans(){
    return InitInverseTransform(W_r_WB_,q_BW_);
  }
  void CalcNormals(){
    std::vector<Eigen::Vector3f> normals(vertices_.size(),Eigen::Vector3f::Zero());
    for (unsigned int i = 0 ; i < indices_.size() ; i += 3) {
      unsigned int Index0 = indices_[i];
      unsigned int Index1 = indices_[i + 1];
      unsigned int Index2 = indices_[i + 2];
      Eigen::Vector3f v1 = vertices_[Index1].pos_.toEigen() - vertices_[Index0].pos_.toEigen();
      Eigen::Vector3f v2 = vertices_[Index2].pos_.toEigen() - vertices_[Index0].pos_.toEigen();
      Eigen::Vector3f Normal = v1.cross(v2);
      Normal.normalize();

      normals[Index0] += Normal;
      normals[Index1] += Normal;
      normals[Index2] += Normal;
    }

    for (unsigned int i = 0 ; i < vertices_.size() ; i++) {
      normals[i].normalize();
      vertices_[i].normal_.fromEigen(normals[i]);
    }
  }
  void loadOptions(){
    if(mbCullFace_){
      glEnable(GL_CULL_FACE);
    } else {
      glDisable(GL_CULL_FACE);
    }
    glLineWidth(lineWidth_);
    glPointSize(pointSize_);
  }
  Eigen::Vector3f W_r_WB_;
  rot::RotationQuaternionPF q_BW_;
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
  Scene(){
    stepScale_ = 0.5f;
    W_r_WC_ = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    q_CW_.setIdentity();
    mMousePos_(0)  = mPersProjInfo_.width_ / 2;
    mMousePos_(1)  = mPersProjInfo_.height_ / 2;
    enableMouseMotion_ = false;
    mDirectionalLight_.color_ = Eigen::Vector3f(1.0f, 1.0f, 1.0f);
    mDirectionalLight_.ambientIntensity_ = 1.0f;
    mDirectionalLight_.diffuseIntensity_ = 0.0f;
    mDirectionalLight_.direction_ = Eigen::Vector3f(0.5f, -1.0f, 0.0f);
    mDirectionalLight_.direction_.normalize();
    mIdleFunc = nullptr;
    addKeyboardCB('q',glutLeaveMainLoop);
    addSpecialKeyboardCB(101,[&]() mutable {W_r_WC_ -= (q_CW_.inverseRotate(Eigen::Vector3f(0.0,0.0,1.0)) * stepScale_);}); // UP
    addSpecialKeyboardCB(103,[&]() mutable {W_r_WC_ += (q_CW_.inverseRotate(Eigen::Vector3f(0.0,0.0,1.0)) * stepScale_);}); // DOWN
    addSpecialKeyboardCB(100,[&]() mutable {W_r_WC_ -= (q_CW_.inverseRotate(Eigen::Vector3f(1.0,0.0,0.0)) * stepScale_);}); // LEFT
    addSpecialKeyboardCB(102,[&]() mutable {W_r_WC_ += (q_CW_.inverseRotate(Eigen::Vector3f(1.0,0.0,0.0)) * stepScale_);}); // RIGHT
    addSpecialKeyboardCB(104,[&]() mutable {W_r_WC_ += (q_CW_.inverseRotate(Eigen::Vector3f(0.0,1.0,0.0)) * stepScale_);}); // PAGE_UP
    addSpecialKeyboardCB(105,[&]() mutable {W_r_WC_ -= (q_CW_.inverseRotate(Eigen::Vector3f(0.0,1.0,0.0)) * stepScale_);}); // PAGE_DOWN
  };
  ~Scene(){
    for(int i=0;i<mSceneObjects_.size();i++){
      delete mSceneObjects_[i];
    }
    mSceneObjects_.clear();
  };
  int init(int argc, char** argv, const std::string& mVSFileName,const std::string& mFSFileName){
    // Must be done after glut is initialized!
    GLenum res = glewInit();
    if (res != GLEW_OK) {
      fprintf(stderr, "Error: '%s'\n", glewGetErrorString(res));
      return 1;
    }

    glClearColor(0.9f, 1.0f, 0.8f, 0.0f);
    glFrontFace(GL_CW);
    glCullFace(GL_BACK);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glClearDepth(1.0f);

    CompileShaders(mVSFileName,mFSFileName);

    mPersProjInfo_.FOV_ = M_PI/2;
    mPersProjInfo_.width_ = 1280;
    mPersProjInfo_.height_ = 960;
    mPersProjInfo_.zNear_ = -0.1f;
    mPersProjInfo_.zFar_ = -100.0f;

    return 0;
  }
  SceneObject* addSceneObject(){
    mSceneObjects_.push_back(new SceneObject());
    return (*mSceneObjects_.rbegin());
  }
  void makeTestScene(){
    SceneObject* mpSceneObject;
    mpSceneObject = addSceneObject();
    mpSceneObject->makeCubeMesh(1.80001f);
    mpSceneObject->setColorFull(Eigen::Vector4f(0.0f,1.0f,0.0f,1.0f));
    mpSceneObject->W_r_WB_ = Eigen::Vector3f(0.0f, 0.0f, 3.0f);
    mpSceneObject = addSceneObject();
    mpSceneObject->makeCubeFull(1.8f);
    mpSceneObject->setColorFull(Eigen::Vector4f(1.0f,1.0f,0.0f,1.0f));
    mpSceneObject->W_r_WB_ = Eigen::Vector3f(0.0f, 0.0f, 3.0f);
    mpSceneObject = addSceneObject();
    mpSceneObject->makeTetrahedron(1.2f);
    mpSceneObject->setColorFull(Eigen::Vector4f(1.0f,0.0f,0.0f,1.0f));
    mpSceneObject->W_r_WB_ = Eigen::Vector3f(0.0f, 1.50001f, 3.0f);
    mpSceneObject = addSceneObject();
    std::vector<Eigen::Vector3f> line;
    line.push_back(Eigen::Vector3f(4.0f,0.0f,0.0f));
    line.push_back(Eigen::Vector3f(0.0f,0.0f,0.0f));
    line.push_back(Eigen::Vector3f(0.0f,4.0f,0.0f));
    line.push_back(Eigen::Vector3f(4.0f,4.0f,0.0f));
    line.push_back(Eigen::Vector3f(4.0f,0.0f,0.0f));
    mpSceneObject->makeLine(line);
    mpSceneObject->prolonge(Eigen::Vector3f(4.0f,0.0f,4.0f));
    mpSceneObject->setColorFull(Eigen::Vector4f(1.0f,0.0f,1.0f,1.0f));
    mpSceneObject->W_r_WB_ = Eigen::Vector3f(0.0f, 0.0f, 3.0f);
    mpSceneObject = addSceneObject();
    mpSceneObject->makePoints(line);
    mpSceneObject->prolonge(Eigen::Vector3f(4.0f,0.0f,4.0f));
    mpSceneObject->setColorFull(Eigen::Vector4f(0.0f,0.0f,1.0f,1.0f));
    mpSceneObject->W_r_WB_ = Eigen::Vector3f(0.0f, 0.0f, 3.0f);
    mpSceneObject = addSceneObject();
    mpSceneObject->makeCoordinateFrame(1.0);
    mpSceneObject->W_r_WB_ = Eigen::Vector3f(0.0f, 1.5f, 3.0f);
    mpSceneObject = addSceneObject();
    mpSceneObject->makeGroundPlane(1.0,10,Eigen::Vector4f(1.0f,1.0f,1.0f,1.0f),Eigen::Vector4f(1.0f,0.0f,1.0f,1.0f));
    mpSceneObject->W_r_WB_ = Eigen::Vector3f(0.0f, 0.0f, 8.0f);
  }
  void RenderSceneCB()
  {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glUniform3f(lightColor_location_, mDirectionalLight_.color_(0), mDirectionalLight_.color_(1), mDirectionalLight_.color_(2));
    glUniform1f(lightAmbientIntensity_location_, mDirectionalLight_.ambientIntensity_);
    glUniform3f(lightDirection_location_, mDirectionalLight_.direction_(0), mDirectionalLight_.direction_(1), mDirectionalLight_.direction_(2));
    glUniform1f(lightDiffuseIntensity_location_, mDirectionalLight_.diffuseIntensity_);

    V_TF_C_ = InitPersProjTransform(mPersProjInfo_);
    C_TF_W_ = InitTransform(W_r_WC_,q_CW_);

    for(int i=0;i<mSceneObjects_.size();i++){
      if(mSceneObjects_[i]->draw_){
        mSceneObjects_[i]->loadOptions();

        glUniform1i(useTexture_location_,mSceneObjects_[i]->useTexture_);
        if(mSceneObjects_[i]->useTexture_){
          glActiveTexture(GL_TEXTURE0);
          glBindTexture(GL_TEXTURE_2D,mSceneObjects_[i]->textureID_);
          glUniform1i(sampler_location_, 0);
        }

        W_TF_B_ = mSceneObjects_[i]->GetWorldTrans();
        V_TF_B_ = V_TF_C_*C_TF_W_*W_TF_B_;

        glUniformMatrix4fv(V_TF_B_location_, 1, GL_FALSE, (const GLfloat*)V_TF_B_.data());
        glUniformMatrix4fv(W_TF_B_location_, 1, GL_FALSE, (const GLfloat*)W_TF_B_.data());

        glEnableVertexAttribArray(0);
        glEnableVertexAttribArray(1);
        glEnableVertexAttribArray(2);
        glBindBuffer(GL_ARRAY_BUFFER, mSceneObjects_[i]->VBO_);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (const GLvoid*)0);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (const GLvoid*)12);
        glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, sizeof(Vertex), (const GLvoid*)24);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mSceneObjects_[i]->IBO_);
        glDrawElements(mSceneObjects_[i]->mode_, mSceneObjects_[i]->indices_.size(), GL_UNSIGNED_INT, 0);
        glDisableVertexAttribArray(0);
        glDisableVertexAttribArray(1);
        glDisableVertexAttribArray(2);
      }
    }
    glutSwapBuffers();
  }
  void setView(const Eigen::Vector3f& pos, const Eigen::Vector3f& target){
    W_r_WC_ = pos;
    q_CW_.setFromVectors(Eigen::Vector3f(0.0f,0.0f,-1.0f),Eigen::Vector3f(target-pos));
  }
  void setYDown(float step = 0.1){
    float min = 1.0f;
    float minAng = 0.0;
    float angle = 0.0;
    rot::RotationQuaternionPF q;
    while(angle<=2*M_PI){
      q = rot::EulerAnglesYprPF(angle,0,0);
      if((q*q_CW_).inverseRotate(Eigen::Vector3f(0,-1,0))(2) < min){
        minAng = angle;
        min = (q*q_CW_).inverseRotate(Eigen::Vector3f(0,-1,0))(2);
      }
      angle += step;
    }
    q_CW_ = rot::EulerAnglesYprPF(minAng,0,0)*q_CW_;
  }
  void SpecialKeyboardCB(int Key, int x, int y)
  {
    if (specialKeyboardCallbacks_.count(Key)>0){
      specialKeyboardCallbacks_[Key]();
    }
  }
  void addSpecialKeyboardCB(int Key, std::function<void()> f){
    specialKeyboardCallbacks_[Key] = f;
  }
  void KeyboardCB(unsigned char Key, int x, int y)
  {
    if (keyboardCallbacks_.count(Key)>0){
      keyboardCallbacks_[Key]();
    }
  }
  void addKeyboardCB(unsigned char Key, std::function<void()> f){
    keyboardCallbacks_[Key] = f;
  }
  void MotionCB(int x, int y)
  {
    if(enableMouseMotion_){
      const int DeltaX = x - mMousePos_(0);
      const int DeltaY = y - mMousePos_(1);

      mMousePos_(0) = x;
      mMousePos_(1) = y;

      Eigen::Vector3f vec;
      vec.setZero();
      vec(0) = -(float)DeltaY / 50.0f;
      vec(1) = -(float)DeltaX/ 50.0f;

      q_CW_ = q_CW_.boxPlus(vec);
      q_CW_.fix();
    }
  }
  void MouseCB(int button, int state, int x, int y)
  {
    if((button == GLUT_LEFT_BUTTON ) && (state == GLUT_DOWN)){
      enableMouseMotion_ = true;
      mMousePos_(0) = x;
      mMousePos_(1) = y;
    }
    if((button == GLUT_LEFT_BUTTON ) && (state == GLUT_UP)){
      enableMouseMotion_ = false;
    }
  }
  void AddShader(GLuint ShaderProgram, const char* pShaderText, GLenum ShaderType)
  {
      GLuint ShaderObj = glCreateShader(ShaderType);
      if (ShaderObj == 0) {
          fprintf(stderr, "Error creating shader type %d\n", ShaderType);
          exit(1);
      }

      const GLchar* p[1];
      p[0] = pShaderText;
      GLint Lengths[1];
      Lengths[0]= strlen(pShaderText);
      glShaderSource(ShaderObj, 1, p, Lengths);
      glCompileShader(ShaderObj);
      GLint success;
      glGetShaderiv(ShaderObj, GL_COMPILE_STATUS, &success);
      if (!success) {
          GLchar InfoLog[1024];
          glGetShaderInfoLog(ShaderObj, 1024, NULL, InfoLog);
          fprintf(stderr, "Error compiling shader type %d: '%s'\n", ShaderType, InfoLog);
          exit(1);
      }

      glAttachShader(ShaderProgram, ShaderObj);
  }
  void CompileShaders(const std::string& mVSFileName,const std::string& mFSFileName){
    GLuint ShaderProgram = glCreateProgram();
    if (ShaderProgram == 0) {
        fprintf(stderr, "Error creating shader program\n");
        exit(1);
    }

    std::string vs, fs;
    if (!ReadShaderFile(mVSFileName.c_str(), vs)) {
        exit(1);
    };
    if (!ReadShaderFile(mFSFileName.c_str(), fs)) {
        exit(1);
    };

    AddShader(ShaderProgram, vs.c_str(), GL_VERTEX_SHADER);
    AddShader(ShaderProgram, fs.c_str(), GL_FRAGMENT_SHADER);

    GLint Success = 0;
    GLchar ErrorLog[1024] = { 0 };
    glLinkProgram(ShaderProgram);
    glGetProgramiv(ShaderProgram, GL_LINK_STATUS, &Success);
    if (Success == 0) {
      glGetProgramInfoLog(ShaderProgram, sizeof(ErrorLog), NULL, ErrorLog);
      fprintf(stderr, "Error linking shader program: '%s'\n", ErrorLog);
          exit(1);
    }
    glValidateProgram(ShaderProgram);
    glGetProgramiv(ShaderProgram, GL_VALIDATE_STATUS, &Success);
    if (!Success) {
        glGetProgramInfoLog(ShaderProgram, sizeof(ErrorLog), NULL, ErrorLog);
        fprintf(stderr, "Invalid shader program: '%s'\n", ErrorLog);
        exit(1);
    }
    glUseProgram(ShaderProgram);

    V_TF_B_location_ = glGetUniformLocation(ShaderProgram, "V_TF_B");
    W_TF_B_location_ = glGetUniformLocation(ShaderProgram, "W_TF_B");
    assert(V_TF_B_location_ != 0xFFFFFFFF);
    assert(W_TF_B_location_ != 0xFFFFFFFF);


    lightColor_location_ = glGetUniformLocation(ShaderProgram, "gDirectionalLight.Color");
    lightAmbientIntensity_location_ = glGetUniformLocation(ShaderProgram, "gDirectionalLight.AmbientIntensity");
    lightDiffuseIntensity_location_ = glGetUniformLocation(ShaderProgram, "gDirectionalLight.DiffuseIntensity");
    lightDirection_location_ = glGetUniformLocation(ShaderProgram, "gDirectionalLight.Direction");
    useTexture_location_ = glGetUniformLocation(ShaderProgram, "useTexture");
    sampler_location_ = glGetUniformLocation(ShaderProgram, "gSampler");
    assert(lightColor_location_ != 0xFFFFFFFF);
    assert(lightAmbientIntensity_location_ != 0xFFFFFFFF);
    assert(lightDiffuseIntensity_location_ != 0xFFFFFFFF);
    assert(lightDirection_location_ != 0xFFFFFFFF);
    assert(useTexture_location_ != 0xFFFFFFFF);
    assert(sampler_location_ != 0xFFFFFFFF);
  }
  void setIdleFunction(void (*idleFunc)()){
    mIdleFunc = idleFunc;
  }
  void (*mIdleFunc)();
 private:
  float stepScale_;

  std::vector<SceneObject*> mSceneObjects_;

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
  rot::RotationQuaternionPF q_CW_;

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
