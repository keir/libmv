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

#ifndef LIBMV_SIMPLE_PIPELINE_RECONSTRUCTION_H_
#define LIBMV_SIMPLE_PIPELINE_RECONSTRUCTION_H_

#include <vector>

#include "libmv/numeric/numeric.h"

namespace libmv {

class Reconstruction {
 public:
  struct Camera {
    Camera() : image(-1) {}
    Camera(const Camera &c) : image(c.image), R(c.R), t(c.t) {}

    int image;
    Mat3 R;
    Vec3 t;
  };

  struct Point {
    int track;
    Vec3 X;
  };

  Reconstruction();

  void InsertCamera(int image, const Mat3 &R, const Vec3 &t);
  void InsertPoint(int track, const Vec3 &X);

  Camera *CameraForImage(int image);
  Point *PointForTrack(int track);

 private:
  std::vector<Camera> cameras_;
  std::vector<Point> points_;
};

}  // namespace libmv

#endif  // LIBMV_SIMPLE_PIPELINE_RECONSTRUCTION_H_
