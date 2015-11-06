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
  FeatureDistance(const Type& type = REGULAR);

  /** \brief Destructor
   */
  virtual ~FeatureDistance();

  /** \brief Set the \ref Type type_ using the enum \ref Type.
   *
   *  @param type - Enum \ref Type.
   */
  void setType(const Type& type);

  /** \brief Get the set \ref Type.
   *
   *  @return the set \ref Type.
   */
  Type getType();

  /** \brief Set the \ref Type type_ using the integer value of the enum \ref Type.
   *
   *  @param type - Integer value of the enum \ref Type.
   */
  void setType(const int& type);

  /** \brief Returns the distance based on the set \ref Type type_.
   *
   *  @return Distance value.
   */
  void setParameter(const double& d);

  /** \brief Returns the distance based on the set \ref Type type_.
   *
   *  @return Distance value.
   */
  double getDistance() const;

  /** \brief Returns the derivative of the distance w.r.t. the parameter.
   *
   *  @return d derived w.r.t. p
   */
  double getDistanceDerivative() const;

  /** \brief Returns the derivative of the parameter w.r.t. the distance.
   *
   *  @return p derived w.r.t. d
   */
  double getParameterDerivative() const;

  /** \brief Returns the derivative of the parameter w.r.t. the distance w.r.t. the parameter.
   *
   *  @return p derived w.r.t. d and then derived w.r.t. p
   */
  double getParameterDerivativeCombined() const;

  /** \brief Converts from an other feature distance parametrization
   *
   *  @param other "other" feature distance, the type is beeing considered during conversion
   */
  void getParameterDerivativeCombined(FeatureDistance other);

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  /** \brief Set the distance parameter for a desired distance for a regular parametrization.
   *
   *  @param Distance value.
   */
  void setParameterRegular(const double& d);

  /** \brief Returns the distance based for a regular parametrization.
   *
   *  @return Distance value.
   */
  double getDistanceRegular() const;

  /** \brief Returns the derivative of the distance w.r.t. the parameter based for a regular parametrization.
   *
   *  @return d derived w.r.t. p
   */
  double getDistanceDerivativeRegular() const;

  /** \brief Returns the derivative of the parameter w.r.t. the distance based for a regular parametrization.
   *
   *  @return p derived w.r.t. d
   */
  double getParameterDerivativeRegular() const;

  /** \brief Returns the derivative of the parameter w.r.t. the distance w.r.t. the parameter based for a regular parametrization.
   *
   *  @return p derived w.r.t. d and then derived w.r.t. p
   */
  double getParameterDerivativeCombinedRegular() const;

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  /** \brief Ensures that a value is non-zero by setting it to a small value
   *
   *  @param p - Input
   *  @return Garanteed non-zero output
   */
  double makeNonZero(const double& p) const;

  /** \brief Set the distance parameter for a desired distance for a inverse parametrization.
   *
   *  @param Distance value.
   */
  void setParameterInverse(const double& d);

  /** \brief Returns the distance based for a Inverse parametrization.
   *
   *  @return Distance value.
   */
  double getDistanceInverse() const;

  /** \brief Returns the derivative of the distance w.r.t. the parameter based for a Inverse parametrization.
   *
   *  @return d derived w.r.t. p
   */
  double getDistanceDerivativeInverse() const;

  /** \brief Returns the derivative of the parameter w.r.t. the distance based for a Inverse parametrization.
   *
   *  @return p derived w.r.t. d
   */
  double getParameterDerivativeInverse() const;

  /** \brief Returns the derivative of the parameter w.r.t. the distance w.r.t. the parameter based for a Inverse parametrization.
   *
   *  @return p derived w.r.t. d and then derived w.r.t. p
   */
  double getParameterDerivativeCombinedInverse() const;

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  /** \brief Set the distance parameter for a desired distance for a log parametrization.
   *
   *  @param Distance value.
   */
  void setParameterLog(const double& d);

  /** \brief Returns the distance based for a Log parametrization.
   *
   *  @return Distance value.
   */
  double getDistanceLog() const;

  /** \brief Returns the derivative of the distance w.r.t. the parameter based for a Log parametrization.
   *
   *  @return d derived w.r.t. p
   */
  double getDistanceDerivativeLog() const;

  /** \brief Returns the derivative of the parameter w.r.t. the distance based for a Log parametrization.
   *
   *  @return p derived w.r.t. d
   */
  double getParameterDerivativeLog() const;

  /** \brief Returns the derivative of the parameter w.r.t. the distance w.r.t. the parameter based for a Log parametrization.
   *
   *  @return p derived w.r.t. d and then derived w.r.t. p
   */
  double getParameterDerivativeCombinedLog() const;

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  /** \brief Set the distance parameter for a desired distance for a hyperbolic parametrization.
   *
   *  @param Distance value.
   */
  void setParameterHyperbolic(const double& d);

  /** \brief Returns the distance based for a Hyperbolic parametrization.
   *
   *  @return Distance value.
   */
  double getDistanceHyperbolic() const;

  /** \brief Returns the derivative of the distance w.r.t. the parameter based for a Hyperbolic parametrization.
   *
   *  @return d derived w.r.t. p
   */
  double getDistanceDerivativeHyperbolic() const;

  /** \brief Returns the derivative of the parameter w.r.t. the distance based for a Hyperbolic parametrization.
   *
   *  @return p derived w.r.t. d
   */
  double getParameterDerivativeHyperbolic() const;

  /** \brief Returns the derivative of the parameter w.r.t. the distance w.r.t. the parameter based for a Hyperbolic parametrization.
   *
   *  @return p derived w.r.t. d and then derived w.r.t. p
   */
  double getParameterDerivativeCombinedHyperbolic() const;
};

}


#endif /* FEATUREDISTANCE_HPP_ */
