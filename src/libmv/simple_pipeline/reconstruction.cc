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

#include "libmv/simple_pipeline/reconstruction.h"

#include <vector>

#include "libmv/numeric/numeric.h"

namespace libmv {

// Start out with some space.
Reconstruction::Reconstruction() : cameras_(1000), points_(1000) {}

void Reconstruction::InsertCamera(int image, const Mat3 &R, const Vec3 &t) {
  if (image >= cameras_.size()) {
    cameras_.resize(image + 1);
  }
  cameras_[image].image = image;
  cameras_[image].R = R;
  cameras_[image].t = t;
}

void Reconstruction::InsertPoint(int track, const Vec3 &X) {
  if (track >= points_.size()) {
    points_.resize(track + 1);
  }
  points_[track].track = track;
  points_[track].X = X;
}

Camera *Reconstruction::CameraForImage(int image) {
  return (cameras_[image].image == -1) ? NULL : &cameras_[image];
}

std::vector<Camera> Reconstruction::AllCameras() {
  std::vector<Camera> cameras;
  for (int i = 0; i < cameras_.size(); ++i) {
    if (cameras_[i].image != -1) {
      cameras.push_back(cameras_[i]);
    }
  }
  return cameras;
}

Point *Reconstruction::PointForTrack(int track) {
  return (points_[track].track == -1) ? NULL : &points_[track];
}

std::vector<Point> Reconstruction::AllPoints() {
  std::vector<Point> points;
  for (int i = 0; i < points_.size(); ++i) {
    if (points_[i].track != -1) {
      points.push_back(points_[i]);
    }
  }
  return points;
}

}  // namespace libmv
