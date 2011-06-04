// Copyright 2011 libmv authors
#ifndef SRC_UI_TRACKER_KLT_H_
#define SRC_UI_TRACKER_KLT_H_
#include <vector>
typedef unsigned char ubyte;

namespace libmv {

class ImagePyramid;

#ifndef LIBMV_CORRESPONDENCE_KLT_H_
struct KLTPointFeature {
  virtual ~KLTPointFeature() {}
  float x,y,scale,orientation;
  int half_window_size;
  float trackness;
};
#endif
typedef std::vector<KLTPointFeature> Features;

class KLT {
 public:
  static KLT* CreateContext(int pyramid_levels = 4, float sigma = 0.9);
  virtual ~KLT() {}
  virtual ImagePyramid* MakeImagePyramid(
    const ubyte* data, int width, int height) = 0;
  virtual Features Detect(ImagePyramid* first) = 0;
  virtual Features Track(int frame, ImagePyramid* previous,
                            ImagePyramid* next) = 0;
};
}

#endif  // SRC_UI_TRACKER_KLT_H_
