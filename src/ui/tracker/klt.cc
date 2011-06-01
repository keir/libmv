// Copyright 2011 libmv authors
#include <list>
#include "libmv/correspondence/matches.h"
#include "libmv/correspondence/feature.h"
#include "libmv/correspondence/klt.h"
#include "libmv/image/image_pyramid.h"
#include "ui/tracker/klt.h"

namespace libmv {

class ConcreteKLT : public KLT {
 public:
  typedef KLTContext::FeatureList FeatureList;

  ConcreteKLT(int pyramid_levels, float sigma)
    : pyramid_levels_(pyramid_levels), sigma_(sigma) {}

  ImagePyramid* MakeImagePyramid(const ubyte* data, int width, int height) {
    Array3Df image(width, height);
    for (int y = 0; y < height; ++y) for (int x = 0; x < width; ++x) {
      image(x, y, 0) = (0.2126*data[(y*width+x)*4+2]+
                        0.7152*data[(y*width+x)*4+1]+
                        0.0722*data[(y*width+x)*4+0]) / 255;
    }
    return libmv::MakeImagePyramid(image, pyramid_levels_, sigma_);
  }

  void Detect(ImagePyramid* first) {
    klt_.DetectGoodFeatures(first->Level(0), &features_);
    int i = 0;
    for (FeatureList::iterator it = features_.begin();
         it != features_.end(); ++it, ++i) {
      matches_.Insert(0, i, *it);
    }
  }

  void Track(int i, ImagePyramid* previous, ImagePyramid* next) {
    for (Matches::Features<KLTPointFeature> r =
         matches_.InImage<KLTPointFeature>(i-1); r; ++r) {
      // FIXME: allocating many small structures
      KLTPointFeature *next_position = new KLTPointFeature;
      if (klt_.TrackFeature(previous, *r.feature(), next, next_position)) {
        matches_.Insert(i, r.track(), next_position);
      } else {
        delete next_position;  // interface should be fixed to avoid this
      }
    }
  }

 private:
  int pyramid_levels_;
  int sigma_;
  KLTContext klt_;
  FeatureList features_;
  Matches matches_;
};

KLT* KLT::CreateContext(int pyramid_levels, float sigma) {
  return new ConcreteKLT(pyramid_levels, sigma);
}
}
