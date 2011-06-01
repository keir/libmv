// Copyright 2011 libmv authors
#ifndef SRC_UI_TRACKER_KLT_H_
#define SRC_UI_TRACKER_KLT_H_
#include <vector>
#include <string>

typedef unsigned char ubyte;

namespace libmv {

class ImagePyramid;

class KLT {
 public:
  static KLT* CreateContext(int pyramid_levels = 4, float sigma = 0.9);
  virtual ~KLT() {}
  virtual ImagePyramid* MakeImagePyramid(
    const ubyte* data, int width, int height) = 0;
  virtual void Detect(ImagePyramid* first) = 0;
  virtual void Track(int frame, ImagePyramid* previous, ImagePyramid* next) = 0;
  // TODO(MatthiasF): return std::vector<Feature>;
};
}

#endif  // SRC_UI_TRACKER_KLT_H_
