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

#ifndef FEATUREDISTANCE_HPP_
#define FEATUREDISTANCE_HPP_

namespace rovio {

/** \brief Class allowing the computation of some distance. Different parametrizations are implemented.
 */
class FeatureDistance{
 public:
  double p_; /**<Distance parameter*/

  /** \brief Specifies the parametrization type
   */
  enum Type{
    REGULAR,    /**<Regular distance p = d*/
    INVERSE,    /**<Inverse distance p = 1/d*/
    LOG,        /**<Logarithmic distance p = ln(d)*/
    HYPERBOLIC  /**<Hyperbolic distance p = asinh(d)*/
  } type_;

  /** \brief Constructor. Create a FeatureDistance object with a desired \ref Type.
   *
   *  @param type - enum \ref Type.
   */
  FeatureDistance(const Type& type = REGULAR){
    reset(type);
  }

  /** \brief Resets the distance parameter
   *
   *  @param type - enum \ref Type.
   */
  void reset(const Type& type = REGULAR){
    setType(type);
    p_ = 1.0;
  }

  /** \brief Set the \ref Type type_ using the enum \ref Type.
   *
   *  @param type - Enum \ref Type.
   */
  void setType(const Type& type){
    type_ = type;
  }

  /** \brief Get the set \ref Type.
   *
   *  @return the set \ref Type.
   */
  Type getType() {
    return type_;
  }

  /** \brief Set the \ref Type type_ using the integer value of the enum \ref Type.
   *
   *  @param type - Integer value of the enum \ref Type.
   */
  void setType(const int& type){
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

  /** \brief Returns the distance based on the set \ref Type type_.
   *
   *  @return Distance value.
   */
  void setParameter(const double& d){
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

  /** \brief Returns the distance based on the set \ref Type type_.
   *
   *  @return Distance value.
   */
  double getDistance() const{
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

  /** \brief Returns the derivative of the distance w.r.t. the parameter.
   *
   *  @return d derived w.r.t. p
   */
  double getDistanceDerivative() const{
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

  /** \brief Returns the derivative of the parameter w.r.t. the distance.
   *
   *  @return p derived w.r.t. d
   */
  double getParameterDerivative() const{
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

  /** \brief Returns the derivative of the parameter w.r.t. the distance w.r.t. the parameter.
   *
   *  @return p derived w.r.t. d and then derived w.r.t. p
   */
  double getParameterDerivativeCombined() const{
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

  /** \brief Converts from an other feature distance parametrization
   *
   *  @param other "other" feature distance, the type is beeing considered during conversion
   */
  void getParameterDerivativeCombined(FeatureDistance other){
    setParameter(other.getDistance());
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  /** \brief Set the distance parameter for a desired distance for a regular parametrization.
   *
   *  @param Distance value.
   */
  void setParameterRegular(const double& d){
    p_ = d;
  }

  /** \brief Returns the distance based for a regular parametrization.
   *
   *  @return Distance value.
   */
  double getDistanceRegular() const{
    return p_;
  }

  /** \brief Returns the derivative of the distance w.r.t. the parameter based for a regular parametrization.
   *
   *  @return d derived w.r.t. p
   */
  double getDistanceDerivativeRegular() const{
    return 1.0;
  }

  /** \brief Returns the derivative of the parameter w.r.t. the distance based for a regular parametrization.
   *
   *  @return p derived w.r.t. d
   */
  double getParameterDerivativeRegular() const{
    return 1.0;
  }

  /** \brief Returns the derivative of the parameter w.r.t. the distance w.r.t. the parameter based for a regular parametrization.
   *
   *  @return p derived w.r.t. d and then derived w.r.t. p
   */
  double getParameterDerivativeCombinedRegular() const{
    return 0.0;
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  /** \brief Ensures that a value is non-zero by setting it to a small value
   *
   *  @param p - Input
   *  @return Garanteed non-zero output
   */
  double makeNonZero(const double& p) const{
    if(p < 1e-6){
      if(p >= 0){
        return 1e-6;
      } else if (p > -1e-6){
        return -1e-6;
      }
    }
    return p;
  }

  /** \brief Set the distance parameter for a desired distance for a inverse parametrization.
   *
   *  @param Distance value.
   */
  void setParameterInverse(const double& d){
    p_ = 1/makeNonZero(d);
  }

  /** \brief Returns the distance based for a Inverse parametrization.
   *
   *  @return Distance value.
   */
  double getDistanceInverse() const{
    const double p_temp = makeNonZero(p_);
    return 1/p_temp;
  }

  /** \brief Returns the derivative of the distance w.r.t. the parameter based for a Inverse parametrization.
   *
   *  @return d derived w.r.t. p
   */
  double getDistanceDerivativeInverse() const{
    const double p_temp = makeNonZero(p_);
    return -1.0/(p_temp*p_temp);
  }

  /** \brief Returns the derivative of the parameter w.r.t. the distance based for a Inverse parametrization.
   *
   *  @return p derived w.r.t. d
   */
  double getParameterDerivativeInverse() const{
    const double p_temp = makeNonZero(p_);
    return -p_temp*p_temp;
  }

  /** \brief Returns the derivative of the parameter w.r.t. the distance w.r.t. the parameter based for a Inverse parametrization.
   *
   *  @return p derived w.r.t. d and then derived w.r.t. p
   */
  double getParameterDerivativeCombinedInverse() const{
    const double p_temp = makeNonZero(p_);
    return -2*p_temp;
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  /** \brief Set the distance parameter for a desired distance for a log parametrization.
   *
   *  @param Distance value.
   */
  void setParameterLog(const double& d){
    p_ = std::log(d);
  }

  /** \brief Returns the distance based for a Log parametrization.
   *
   *  @return Distance value.
   */
  double getDistanceLog() const{
    return std::exp(p_);
  }

  /** \brief Returns the derivative of the distance w.r.t. the parameter based for a Log parametrization.
   *
   *  @return d derived w.r.t. p
   */
  double getDistanceDerivativeLog() const{
    return std::exp(p_);
  }

  /** \brief Returns the derivative of the parameter w.r.t. the distance based for a Log parametrization.
   *
   *  @return p derived w.r.t. d
   */
  double getParameterDerivativeLog() const{
    return std::exp(-p_);
  }

  /** \brief Returns the derivative of the parameter w.r.t. the distance w.r.t. the parameter based for a Log parametrization.
   *
   *  @return p derived w.r.t. d and then derived w.r.t. p
   */
  double getParameterDerivativeCombinedLog() const{
    return -std::exp(-p_);
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  /** \brief Set the distance parameter for a desired distance for a hyperbolic parametrization.
   *
   *  @param Distance value.
   */
  void setParameterHyperbolic(const double& d){
    p_ = std::asinh(d);
  }

  /** \brief Returns the distance based for a Hyperbolic parametrization.
   *
   *  @return Distance value.
   */
  double getDistanceHyperbolic() const{
    return std::sinh(p_);
  }

  /** \brief Returns the derivative of the distance w.r.t. the parameter based for a Hyperbolic parametrization.
   *
   *  @return d derived w.r.t. p
   */
  double getDistanceDerivativeHyperbolic() const{
    return std::cosh(p_);
  }

  /** \brief Returns the derivative of the parameter w.r.t. the distance based for a Hyperbolic parametrization.
   *
   *  @return p derived w.r.t. d
   */
  double getParameterDerivativeHyperbolic() const{
    return 1/std::sqrt(std::pow(std::sinh(p_),2)+1); // p = asinh(d)
  }

  /** \brief Returns the derivative of the parameter w.r.t. the distance w.r.t. the parameter based for a Hyperbolic parametrization.
   *
   *  @return p derived w.r.t. d and then derived w.r.t. p
   */
  double getParameterDerivativeCombinedHyperbolic() const{
    return -std::sinh(p_)/std::pow(std::pow(std::sinh(p_),2)+1,1.5)*std::cosh(p_);
  }
};

}


#endif /* FEATUREDISTANCE_HPP_ */
