// Copyright (c) 2011 libmv authors.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to
// deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
// sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
#include <list>
#include "libmv/correspondence/matches.h"
#include "libmv/correspondence/feature.h"
#include "libmv/correspondence/klt.h"
#include "libmv/image/image_pyramid.h"
#include "ui/tracker/klt.h"

namespace libmv {

class ConcreteKLT : public KLT {
 public:

  ConcreteKLT(int pyramid_levels, float sigma)
    : pyramid_levels_(pyramid_levels), sigma_(sigma) {
    klt_.min_trackness_=0.5;
  }

  ImagePyramid* MakeImagePyramid(const ubyte* data, int width, int height) {
    Array3Df image(width, height);
    for (int y = 0; y < height; ++y) for (int x = 0; x < width; ++x) {
      image(x, y, 0) = (0.2126*data[(y*width+x)*4+2]+
                        0.7152*data[(y*width+x)*4+1]+
                        0.0722*data[(y*width+x)*4+0]) / 255;
    }
    return libmv::MakeImagePyramid(image, pyramid_levels_, sigma_);
  }

  Features Detect(ImagePyramid* first) {
    features_.resize(1);
    klt_.DetectGoodFeatures(first->Level(0), &features_[0]);
    return features_[0];
  }

  Features Track(int frame, ImagePyramid* previous, ImagePyramid* next) {
    features_.resize(frame);
    for(size_t i = 0 ; i < features_[frame-1].size() ; i++ ) {
      const KLTPointFeature& position = features_[frame-1][i];
      KLTPointFeature next_position;
      if (klt_.TrackFeature(previous, position, next, &next_position)) {
        features_[frame].push_back(next_position);
      }
    }
    return features_[frame];
  }

 private:
  int pyramid_levels_;
  int sigma_;
  KLTContext klt_;
  std::vector<Features> features_;
};

KLT* KLT::CreateContext(int pyramid_levels, float sigma) {
  return new ConcreteKLT(pyramid_levels, sigma);
}
}
