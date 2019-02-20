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
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND
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

#ifndef ROVIO_IMAGE_MASK_HPP_
#define ROVIO_IMAGE_MASK_HPP_

#include <algorithm>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "rovio/FeatureCoordinates.hpp"

namespace rovio {

class ImageMask {
 public:
  ImageMask() {}

  // Loads an image mask. White = ok, black = don't detect features.
  bool loadImage(const std::string& image_path) {
    mask_ = cv::imread(image_path, CV_LOAD_IMAGE_GRAYSCALE);
    if (!mask_.data) {
      std::cout << "Couldn't load image mask from path: " << image_path
                << std::endl;
      return false;
    }
    std::cout << "Successfully loaded image mask from path: " << image_path
              << std::endl;
    return true;
  }

  // Prunes feature vector, removing any features detected in the *black* region
  // of the mask.
  void pruneFeatureVector(FeatureCoordinatesVec* feature_vec) const {
    if (!mask_.data) {
      return;
    }
    // Ugly or nice -- up to you. Write a lambda that looks up each feature in
    // the mask to decide if it should be erased.
    feature_vec->erase(
        std::remove_if(feature_vec->begin(), feature_vec->end(),
                       [](const FeatureCoordinates& feature_coord) {
                         return false;
                       }),
        feature_vec->end());
  }

 private:
  cv::Mat mask_;
};

}  // namespace rovio

#endif  // ROVIO_IMAGE_MASK_HPP_
