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

#include <vector>

#include "libmv/numeric/numeric.h"
#include "libmv/multiview/fundamental.h"
#include "libmv/simple_pipeline/tracks.h"
#include "libmv/simple_pipeline/reconstruction.h"

using std::vector;

namespace libmv {

void CoordinatesForMarkersInImage(const vector<Marker> &markers,
                                  int image,
                                  Mat *coordinates) {
  (void) markers;
  (void) image;
  (void) coordinates;
  // XXX
}

void GetImagesInMarkers(const vector<Marker> &markers,
                        int *image1, int *image2) {
  if (markers.empty()) {
    return;
  }
  *image1 = markers[0].image;
  for (int i = 1; i < markers.size(); ++i) {
    if (markers[i].image != *image1) {
      *image2 = markers[i].image;
      return;
    }
  }
}

bool ReconstructTwoFrames(const vector<Marker> &markers,
                          Reconstruction *reconstruction) {
  if (markers.size() < 16) {
    return false;
  }

  int image1, image2;
  GetImagesInMarkers(markers, &image1, &image2);

  Mat x1, x2;
  CoordinatesForMarkersInImage(markers, image1, &x1);
  CoordinatesForMarkersInImage(markers, image2, &x2);

  Mat3 F, E;
  NormalizedEightPointSolver(x1, x2, &F);

  // XXX Broken!
  Mat3 K1, K2;
  EssentialFromFundamental(F, K1, K2, &E);

  // Recover motion between the two images
  Mat3 dR;
  Vec3 dt;
  Mat3 K = Mat3::Identity();
  if (!MotionFromEssentialAndCorrespondence(
          E, K, x1.col(0), K, x2.col(0), &dR, &dt)) {
    return false;
  }

  reconstruction->CameraForImage(image1)->image = image1;
  reconstruction->CameraForImage(image1)->R = Mat3::Identity();
  reconstruction->CameraForImage(image1)->t = Vec3::Zero();

  reconstruction->CameraForImage(image2)->image = image2;
  reconstruction->CameraForImage(image2)->R = dR;
  reconstruction->CameraForImage(image2)->t = dt;

  return true;
}

bool Resect(const vector<Marker> &markers, Reconstruction */*reconstruction*/) {
  if (markers.size() < 8) {
    return false;
  }
  // XXX
  return true;
}

bool Intersect(const vector<Marker> &markers, Reconstruction */*reconstruction*/) {
  if (markers.size() < 4) {
    return false;
  }
  // XXX
  return true;
}

void Bundle(const Tracks &/*tracks*/, Reconstruction */*reconstruction*/) {
  // XXX
}

}  // namespace libmv
