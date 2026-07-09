#include <rovio/Scene.hpp>

namespace rovio{

  SceneObject::SceneObject(){
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
  SceneObject::~SceneObject(){};
  void SceneObject::initTexture(const int& cols, const int& rows){
    useTexture_ = true;
    glBindTexture(GL_TEXTURE_2D, textureID_);
    glTexImage2D(GL_TEXTURE_2D,0,GL_LUMINANCE,cols,rows,0,GL_LUMINANCE,GL_UNSIGNED_BYTE,nullptr);
  }
  void SceneObject::setTexture(const int& cols, const int& rows, const float* ptr){
    glBindTexture(GL_TEXTURE_2D, textureID_);
    glTexImage2D(GL_TEXTURE_2D,0,GL_LUMINANCE,cols,rows,0,GL_LUMINANCE,GL_FLOAT,ptr);
  }
  void SceneObject::setTexture(const cv::Mat& img){
    glBindTexture(GL_TEXTURE_2D, textureID_);
    glTexSubImage2D(GL_TEXTURE_2D,0,0,0,img.cols,img.rows,GL_LUMINANCE,GL_UNSIGNED_BYTE,img.data);
  }
  void SceneObject::allocateBuffer(){
    glBindBuffer(GL_ARRAY_BUFFER, VBO_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(Vertex)*vertices_.size(), vertices_.data(), GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, IBO_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int)*indices_.size(), indices_.data(), GL_DYNAMIC_DRAW);
  }
  void SceneObject::makeCubeFull(const float& l){
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
  void SceneObject::makeTetrahedron(const float& l){
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
  void SceneObject::makeCubeMesh(const float& l){
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
  void SceneObject::makeLine(){
    std::vector<Eigen::Vector3f> emptyLine;
    makeLine(emptyLine);
  }
  void SceneObject::makeLine(const std::vector<Eigen::Vector3f>& line){
    clear();
    prolonge(line);
    mode_ =  GL_LINE_STRIP;
  }
  void SceneObject::makePoints(){
    std::vector<Eigen::Vector3f> empty;
    makePoints(empty);
  }
  void SceneObject::makePoints(const std::vector<Eigen::Vector3f>& points){
    clear();
    prolonge(points);
    mode_ =  GL_POINTS;
  }
  void SceneObject::prolonge(const Eigen::Vector3f& point){
    std::vector<Eigen::Vector3f> line(1,point);
    prolonge(line);
  }
  void SceneObject::prolonge(const std::vector<Eigen::Vector3f>& points){
    vertices_.reserve(vertices_.size()+points.size());
    indices_.reserve(indices_.size()+points.size());
    const int s = vertices_.size();
    for(int i=0;i<points.size();i++){
      vertices_.push_back(points[i]);
      indices_.push_back(s+i);
    }
    allocateBuffer();
  }
  void SceneObject::clear(){
    vertices_.clear();
    indices_.clear();
  }
  void SceneObject::makeCoordinateFrame(const float& l){
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
  void SceneObject::makeRectangle(const float& x,const float& y){
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
  void SceneObject::makeTexturedRectangle(const float& x,const float& y){
    makeRectangle(x,y);
    vertices_[0].color_.data_[0] = 0.0f;
    vertices_[0].color_.data_[1] = 0.0f;
    vertices_[1].color_.data_[0] = 1.0f;
    vertices_[1].color_.data_[1] = 0.0f;
    vertices_[2].color_.data_[0] = 0.0f;
    vertices_[2].color_.data_[1] = 1.0f;
    vertices_[3].color_.data_[0] = 1.0f;
    vertices_[3].color_.data_[1] = 1.0f;
    allocateBuffer();
  }
  void SceneObject::makeGroundPlane(const float& l,const int& N,const Eigen::Vector4f& color1,const Eigen::Vector4f& color2){
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
  void SceneObject::makeGroundPlaneMesh(const float& l,const int& N){
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
  void SceneObject::setColorFull(const Eigen::Vector4f& color){
    for (unsigned int i = 0 ; i < vertices_.size() ; i++) {
      vertices_[i].color_.fromEigen(color);
    }
    allocateBuffer();
  }
  const Eigen::Matrix4f SceneObject::GetWorldTrans(){
    return InitInverseTransform(W_r_WB_,q_BW_);
  }
  void SceneObject::CalcNormals(){
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
  void SceneObject::loadOptions(){
    if(mbCullFace_){
      glEnable(GL_CULL_FACE);
    } else {
      glDisable(GL_CULL_FACE);
    }
    glLineWidth(lineWidth_);
    glPointSize(pointSize_);
  }

  Scene::Scene(){
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
  Scene::~Scene(){};
  int Scene::init(int argc, char** argv, const std::string& mVSFileName,const std::string& mFSFileName){
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
  std::shared_ptr<SceneObject> Scene::addSceneObject(){
    std::shared_ptr<SceneObject> sp(new SceneObject());
    mSceneObjects_.push_back(sp);
    return sp;
  }
  void Scene::makeTestScene(){
    std::shared_ptr<SceneObject> mpSceneObject;
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
  void Scene::RenderSceneCB()
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
  void Scene::setView(const Eigen::Vector3f& pos, const Eigen::Vector3f& target){
    W_r_WC_ = pos;
    q_CW_.setFromVectors(Eigen::Vector3f(target-pos),Eigen::Vector3f(0.0f,0.0f,-1.0f));
  }
  void Scene::setYDown(float step){
    float min = 1.0f;
    float minAng = 0.0;
    float angle = 0.0;
    kindr::RotationQuaternionPF q;
    while(angle<=2*M_PI){
      q = kindr::EulerAnglesYprPF(angle,0,0);
      if((q*q_CW_).inverseRotate(Eigen::Vector3f(0,-1,0))(2) < min){
        minAng = angle;
        min = (q*q_CW_).inverseRotate(Eigen::Vector3f(0,-1,0))(2);
      }
      angle += step;
    }
    q_CW_ = kindr::EulerAnglesYprPF(minAng,0,0)*q_CW_;
  }
  void Scene::SpecialKeyboardCB(int Key, int x, int y)
  {
    if (specialKeyboardCallbacks_.count(Key)>0){
      specialKeyboardCallbacks_[Key]();
    }
  }
  void Scene::addSpecialKeyboardCB(int Key, std::function<void()> f){
    specialKeyboardCallbacks_[Key] = f;
  }
  void Scene::KeyboardCB(unsigned char Key, int x, int y)
  {
    if (keyboardCallbacks_.count(Key)>0){
      keyboardCallbacks_[Key]();
    }
  }
  void Scene::addKeyboardCB(unsigned char Key, std::function<void()> f){
    keyboardCallbacks_[Key] = f;
  }
  void Scene::MotionCB(int x, int y)
  {
    if(enableMouseMotion_){
      const int DeltaX = x - mMousePos_(0);
      const int DeltaY = y - mMousePos_(1);

      mMousePos_(0) = x;
      mMousePos_(1) = y;

      Eigen::Vector3f vec;
      vec.setZero();
      vec(0) = (float)DeltaY / 50.0f;
      vec(1) = (float)DeltaX / 50.0f;

      q_CW_ = q_CW_.boxPlus(vec);
      q_CW_.fix();
    }
  }
  void Scene::MouseCB(int button, int state, int x, int y)
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
  void Scene::AddShader(GLuint ShaderProgram, const char* pShaderText, GLenum ShaderType)
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
  void Scene::CompileShaders(const std::string& mVSFileName,const std::string& mFSFileName){
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
  void Scene::setIdleFunction(void (*idleFunc)()){
    mIdleFunc = idleFunc;
  }

}
