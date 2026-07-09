#include "rovio/FeatureDistance.hpp"
#include <complex>
#include <iostream>

namespace rovio {

  FeatureDistance::FeatureDistance(const Type& type){
    setType(type);
    p_ = 1.0;
  }

  FeatureDistance::~FeatureDistance(){};

  void FeatureDistance::setType(const Type& type){
    type_ = type;
  }

  FeatureDistance::Type FeatureDistance::getType(){
    return type_;
  }

  void FeatureDistance::setType(const int& type){
    switch(type){
      case 0:
        type_ = REGULAR;
        break;
      case 1:
        type_ = INVERSE;
        break;
      case 2:
        type_ = LOG;
        break;
      case 3:
        type_ = HYPERBOLIC;
        break;
      default:
        std::cout << "Invalid type for distance parameterization: " << type << std::endl;
        type_ = REGULAR;
        break;
    }
  }

  void FeatureDistance::setParameter(const double& d){
    switch(type_){
      case REGULAR:
        setParameterRegular(d);
        break;
      case INVERSE:
        setParameterInverse(d);
        break;
      case LOG:
        setParameterLog(d);
        break;
      case HYPERBOLIC:
        setParameterHyperbolic(d);
        break;
      default:
        setParameterRegular(d);
        break;
    }
  }

  double FeatureDistance::getDistance() const{
    switch(type_){
      case REGULAR:
        return getDistanceRegular();
      case INVERSE:
        return getDistanceInverse();
      case LOG:
        return getDistanceLog();
      case HYPERBOLIC:
        return getDistanceHyperbolic();
      default:
        return getDistanceRegular();
    }
  }

  double FeatureDistance::getDistanceDerivative() const{
    switch(type_){
      case REGULAR:
        return getDistanceDerivativeRegular();
      case INVERSE:
        return getDistanceDerivativeInverse();
      case LOG:
        return getDistanceDerivativeLog();
      case HYPERBOLIC:
        return getDistanceDerivativeHyperbolic();
      default:
        return getDistanceDerivativeRegular();
    }
  }

  double FeatureDistance::getParameterDerivative() const{
    switch(type_){
      case REGULAR:
        return getParameterDerivativeRegular();
      case INVERSE:
        return getParameterDerivativeInverse();
      case LOG:
        return getParameterDerivativeLog();
      case HYPERBOLIC:
        return getParameterDerivativeHyperbolic();
      default:
        return getParameterDerivativeRegular();
    }
  }

  double FeatureDistance::getParameterDerivativeCombined() const{
    switch(type_){
      case REGULAR:
        return getParameterDerivativeCombinedRegular();
      case INVERSE:
        return getParameterDerivativeCombinedInverse();
      case LOG:
        return getParameterDerivativeCombinedLog();
      case HYPERBOLIC:
        return getParameterDerivativeCombinedHyperbolic();
      default:
        return getParameterDerivativeCombinedRegular();
    }
  }

  void FeatureDistance::getParameterDerivativeCombined(FeatureDistance other){
    setParameter(other.getDistance());
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  void FeatureDistance::setParameterRegular(const double& d){
    p_ = d;
  }

  double FeatureDistance::getDistanceRegular() const{
    return p_;
  }

  double FeatureDistance::getDistanceDerivativeRegular() const{
    return 1.0;
  }

  double FeatureDistance::getParameterDerivativeRegular() const{
    return 1.0;
  }

  double FeatureDistance::getParameterDerivativeCombinedRegular() const{
    return 0.0;
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  double FeatureDistance::makeNonZero(const double& p) const{
    if(p < 1e-6){
      if(p >= 0){
        return 1e-6;
      } else if (p > -1e-6){
        return -1e-6;
      }
    }
    return p;
  }

  void FeatureDistance::setParameterInverse(const double& d){
    p_ = 1/makeNonZero(d);
  }

  double FeatureDistance::getDistanceInverse() const{
    const double p_temp = makeNonZero(p_);
    return 1/p_temp;
  }

  double FeatureDistance::getDistanceDerivativeInverse() const{
    const double p_temp = makeNonZero(p_);
    return -1.0/(p_temp*p_temp);
  }

  double FeatureDistance::getParameterDerivativeInverse() const{
    const double p_temp = makeNonZero(p_);
    return -p_temp*p_temp;
  }

  double FeatureDistance::getParameterDerivativeCombinedInverse() const{
    const double p_temp = makeNonZero(p_);
    return -2*p_temp;
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  void FeatureDistance::setParameterLog(const double& d){
    p_ = std::log(d);
  }

  double FeatureDistance::getDistanceLog() const{
    return std::exp(p_);
  }

  double FeatureDistance::getDistanceDerivativeLog() const{
    return std::exp(p_);
  }

  double FeatureDistance::getParameterDerivativeLog() const{
    return std::exp(-p_);
  }

  double FeatureDistance::getParameterDerivativeCombinedLog() const{
    return -std::exp(-p_);
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  void FeatureDistance::setParameterHyperbolic(const double& d){
    p_ = std::asinh(d);
  }

  double FeatureDistance::getDistanceHyperbolic() const{
    return std::sinh(p_);
  }

  double FeatureDistance::getDistanceDerivativeHyperbolic() const{
    return std::cosh(p_);
  }

  double FeatureDistance::getParameterDerivativeHyperbolic() const{
    return 1/std::sqrt(std::pow(std::sinh(p_),2)+1); // p = asinh(d)
  }

  double FeatureDistance::getParameterDerivativeCombinedHyperbolic() const{
    return -std::sinh(p_)/std::pow(std::pow(std::sinh(p_),2)+1,1.5)*std::cosh(p_);
  }
}
