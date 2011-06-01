// Copyright 2011 libmv authors
#include <list>
#include "libmv/correspondence/matches.h"
#include "libmv/correspondence/feature.h"
#include "libmv/correspondence/klt.h"
#include "libmv/image/image_pyramid.h"
#include "ui/tracker/klt.h"

#define foreach(t,c) for ( t::iterator it = c.begin() ; it != c.end() ; ++it )

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
    klt_.DetectGoodFeatures(first->Level(0), features_[0]);
    return features_[0];
  }

  Features Track(int i, ImagePyramid* previous, ImagePyramid* next) {
    features_.resize(i);
    foreach (Features, features_[i-1] ) {
      KLTPointFeature next_position;
      if (klt_.TrackFeature(previous, *it, next, next_position)) {
        features_[i].push_back(next_position);
      }
    }
    return features_[i];
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
