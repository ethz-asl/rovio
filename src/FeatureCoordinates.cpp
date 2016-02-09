#include "rovio/FeatureCoordinates.hpp"

namespace rovio{

  FeatureCoordinates::FeatureCoordinates(){
    mpCamera_ = nullptr;
    trackWarping_ = false;
    resetCoordinates();
  }

  FeatureCoordinates::FeatureCoordinates(const cv::Point2f& pixel){
    mpCamera_ = nullptr;
    trackWarping_ = false;
    resetCoordinates();
    c_ = pixel;
    valid_c_ = true;
  }

  FeatureCoordinates::FeatureCoordinates(const LWF::NormalVectorElement& nor){
    mpCamera_ = nullptr;
    trackWarping_ = false;
    resetCoordinates();
    nor_ = nor;
    valid_nor_ = true;
  }

  FeatureCoordinates::FeatureCoordinates(const Camera* mpCamera): mpCamera_(mpCamera){
    trackWarping_ = false;
    resetCoordinates();
  }

  FeatureCoordinates::~FeatureCoordinates(){};

  void FeatureCoordinates::resetCoordinates(){
    valid_c_ = false;
    valid_nor_ = false;
    set_warp_identity();
    camID_ = -1;
  }

  bool FeatureCoordinates::com_c() const{
    if(!valid_c_){
      assert(mpCamera_ != nullptr);
      if(valid_nor_ && mpCamera_->bearingToPixel(nor_,c_)){
        valid_c_ = true;
      }
    }
    return valid_c_;
  }

  const cv::Point2f& FeatureCoordinates::get_c() const{
    if(!com_c()){
      std::cout << "    \033[31mERROR: No valid coordinate data!\033[0m" << std::endl;
    }
    return c_;
  }

  bool FeatureCoordinates::com_nor() const{
    if(!valid_nor_){
      assert(mpCamera_ != nullptr);
      if(valid_c_ && mpCamera_->pixelToBearing(c_,nor_)){
        valid_nor_ = true;
      }
    }
    return valid_nor_;
  }

  const LWF::NormalVectorElement& FeatureCoordinates::get_nor() const{
    if(!com_nor()){
      std::cout << "    \033[31mERROR: No valid coordinate data!\033[0m" << std::endl;
    }
    return nor_;
  }

  Eigen::Matrix<double,2,2> FeatureCoordinates::get_J() const{
    assert(mpCamera_ != nullptr);
    if(!mpCamera_->bearingToPixel(get_nor(),c_,matrix2dTemp_)){
      matrix2dTemp_.setZero();
      std::cout << "    \033[31mERROR: No valid coordinate data!\033[0m" << std::endl;
    }
    return matrix2dTemp_;
  }

  void FeatureCoordinates::set_c(const cv::Point2f& c, const bool resetWarp){
    c_ = c;
    valid_c_ = true;
    valid_nor_ = false;
    if(trackWarping_ && resetWarp){
      valid_warp_c_ = false;
      valid_warp_nor_ = false;
    }
  }

  void FeatureCoordinates::set_nor(const LWF::NormalVectorElement& nor, const bool resetWarp){
    nor_ = nor;
    valid_nor_ = true;
    valid_c_ = false;
    if(trackWarping_ && resetWarp){
      valid_warp_c_ = false;
      valid_warp_nor_ = false;
    }
  }

  bool FeatureCoordinates::com_warp_c() const{
    if(!valid_warp_c_){
      if(valid_warp_nor_ && com_c() && com_nor()){
        matrix2dTemp_ = get_J();
        warp_c_ = (matrix2dTemp_*warp_nor_).cast<float>();
        valid_warp_c_ = true;
      }
    }
    return valid_warp_c_;
  }

  Eigen::Matrix2f& FeatureCoordinates::get_warp_c() const{
    if(!com_warp_c()){
      std::cout << "    \033[31mERROR: No valid warping data in get_warp_c!\033[0m" << std::endl;
    }
    return warp_c_;
  }

  bool FeatureCoordinates::com_warp_nor() const{
    if(!valid_warp_nor_){
      if(valid_warp_c_ && com_c() && com_nor()){
        matrix2dTemp_ = get_J();
        fullPivLU2d_.compute(matrix2dTemp_);
        if(fullPivLU2d_.rank() == 2){
          warp_nor_ = fullPivLU2d_.inverse()*warp_c_.cast<double>();
          valid_warp_nor_ = true;
        }
      }
    }
    return valid_warp_nor_;
  }

  Eigen::Matrix2d& FeatureCoordinates::get_warp_nor() const{
    if(!com_warp_nor()){
      std::cout << "    \033[31mERROR: No valid warping data in get_warp_nor!\033[0m" << std::endl;
    }
    return warp_nor_;
  }

  FeatureCoordinates FeatureCoordinates::get_patchCorner(const double x, const double y) const{
    FeatureCoordinates temp; // TODO: avoid temp
    get_nor().boxPlus(get_warp_nor()*Eigen::Vector2d(x,y),norTemp_);
    temp.set_nor(norTemp_);
    temp.mpCamera_ = mpCamera_;
    temp.camID_ = camID_;
    return temp;
  }

  void FeatureCoordinates::set_warp_c(const Eigen::Matrix2f& warp_c){
    warp_c_ = warp_c;
    valid_warp_c_ = true;
    valid_warp_nor_ = false;
    isWarpIdentity_ = false;
  }

  void FeatureCoordinates::set_warp_nor(const Eigen::Matrix2d& warp_nor){
    warp_nor_ = warp_nor;
    valid_warp_nor_ = true;
    valid_warp_c_ = false;
    isWarpIdentity_ = false;
  }

  void FeatureCoordinates::set_warp_identity(){
    warp_c_.setIdentity();
    valid_warp_c_ = true;
    valid_warp_nor_ = false;
    isWarpIdentity_ = true;
  }

  bool FeatureCoordinates::isInFront() const{
    return valid_c_ || (valid_nor_ && nor_.getVec()[2] > 0);
  }

  bool FeatureCoordinates::isNearIdentityWarping() const{
    return isWarpIdentity_ || (com_warp_c() && (get_warp_c()-Eigen::Matrix2f::Identity()).norm() < 1e-6);
  }

  void FeatureCoordinates::setPixelCov(const Eigen::Matrix2d& cov){
    pixelCov_ = cov;
    es_.compute(cov);
    sigmaAngle_ = std::atan2(es_.eigenvectors()(1,0).real(),es_.eigenvectors()(0,0).real());
    sigma1_ = sqrt(es_.eigenvalues()(0).real());
    sigma2_ = sqrt(es_.eigenvalues()(1).real());
    if(sigma1_<sigma2_){ // Get larger axis on index 1
      const double temp = sigma1_;
      sigma1_ = sigma2_;
      sigma2_ = temp;
      sigmaAngle_ += 0.5*M_PI;
      eigenVector1_ = es_.eigenvectors().col(1).real();
      eigenVector2_ = es_.eigenvectors().col(0).real();
    } else {
      eigenVector1_ = es_.eigenvectors().col(0).real();
      eigenVector2_ = es_.eigenvectors().col(1).real();
    }
  }

  void FeatureCoordinates::drawPoint(cv::Mat& drawImg, const cv::Scalar& color, const float s) const{
    cv::Size size(s,s);
    cv::ellipse(drawImg,get_c(),size,0,0,360,color,-1,8,0);
  }

  void FeatureCoordinates::drawEllipse(cv::Mat& drawImg, const cv::Scalar& color, double scaleFactor, const bool withCenterPoint) const{
    if(withCenterPoint) drawPoint(drawImg,color);
    cv::ellipse(drawImg,get_c(),cv::Size(std::max(static_cast<int>(scaleFactor*sigma1_+0.5),1),std::max(static_cast<int>(scaleFactor*sigma2_+0.5),1)),sigmaAngle_*180/M_PI,0,360,color,1,8,0);
  }

  void FeatureCoordinates::drawLine(cv::Mat& drawImg, const FeatureCoordinates& other, const cv::Scalar& color, int thickness) const{
    cv::line(drawImg,get_c(),other.get_c(),color,thickness);
  }

  void FeatureCoordinates::drawText(cv::Mat& drawImg, const std::string& s, const cv::Scalar& color) const{
    cv::putText(drawImg,s,get_c(),cv::FONT_HERSHEY_SIMPLEX, 0.4, color);
  }

  bool FeatureCoordinates::getDepthFromTriangulation(const FeatureCoordinates& other, const V3D& C2rC2C1, const QPD& qC2C1, FeatureDistance& d, const double minDistance){
    const V3D C2v1 = qC2C1.rotate(get_nor().getVec());
    const V3D C2v2 = other.get_nor().getVec();
    const double a = 1.0-pow(C2v1.dot(C2v2),2.0);
    if(a < 1e-6){
      return false;
    }

    const double distance = -C2v1.dot((M3D::Identity()-C2v2*C2v2.transpose())*C2rC2C1) / a;
    if(distance < minDistance){
      return false;
    }
    d.setParameter(distance);
    return true;

//    Possible alternative -> investigate
//    Eigen::Matrix<double,3,2> V;
//    V.col(0) = -C2v1;
//    V.col(1) = C2v2;
//    Eigen::JacobiSVD<Eigen::Matrix<double,3,2>> svd(V, Eigen::ComputeThinU | Eigen::ComputeThinV);
//    Eigen::Vector2d distances = svd.solve(C2rC2C1);
  }

  float FeatureCoordinates::getDepthUncertaintyTau(const V3D& C1rC1C2, const float d, const float px_error_angle){
    const V3D C1fP = get_nor().getVec();
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
