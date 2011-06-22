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

#include "libmv/base/vector.h"
#include "libmv/numeric/numeric.h"
#include "libmv/multiview/fundamental.h"
#include "libmv/simple_pipeline/tracks.h"
#include "libmv/simple_pipeline/reconstruction.h"
#include "libmv/multiview/nviewtriangulation.h"

namespace libmv {

void CoordinatesForMarkersInImage(const vector<Marker> &markers,
                                  int image,
                                  Mat *coordinates) {
  vector<Vec2> coords;
  for (int i = 0; i < markers.size(); ++i) {
    const Marker &marker = markers[i];
    if (markers[i].image == image) {
      coords.push_back(Vec2(marker.x, marker.y));
    }
  }
  // FIXME: avoid pointless conversions between abstractions
  coordinates->resize(2, coords.size());
  for (int i = 0; i < coords.size(); i++) {
    coordinates->col(i) = coords[i];
  }
}

void GetImagesInMarkers(const vector<Marker> &markers,
                        int *image1, int *image2) {
  if (markers.size() < 2) {
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

  Mat3 F;
  NormalizedEightPointSolver(x1, x2, &F);

  // The F matrix should be an E matrix, but squash it just to be sure.
  Eigen::JacobiSVD<Mat3> svd(F, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Vec3 diag;
  diag << 1, 1, 0;
  Mat3 E = svd.matrixU() * diag.asDiagonal() * svd.matrixV().transpose();

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
  return true;
}

bool Intersect(const vector<Marker> &markers, Reconstruction *reconstruction) {
  if (markers.size() < 4) {
    return false;
  }
  int image1, image2;
  GetImagesInMarkers(markers, &image1, &image2);

  vector<Matrix<double, 3, 4> > Ps;
  {
    Camera camera = *reconstruction->CameraForImage(image1);
    Mat3 K;
    // FIXME: we need calibration data
    /*K << calibration_.focal_length, 0, (w-1)/2,
      0, calibration_.focal_length, (h-1)/2,
      0, 0,                         1;*/
    K << 1, 0, 0,
        0, 1, 0,
        0, 0, 1;
    Mat34 P;
    P_From_KRt(K, camera.R, camera.t, &P);
    Ps.push_back(P);
  }
  {
    Camera camera = *reconstruction->CameraForImage(image2);
    Mat3 K;
    // FIXME: we need calibration data
    /*K << calibration_.focal_length, 0, (w-1)/2,
      0, calibration_.focal_length, (h-1)/2,
      0, 0,                         1;*/
    K << 1, 0, 0,
        0, 1, 0,
        0, 0, 1;
    Mat34 P;
    P_From_KRt(K, camera.R, camera.t, &P);
    Ps.push_back(P);
  }

  // FIXME: the API makes retrieving matching markers quite inconvenient
  for(int i = 0; i < markers.size(); i++) {
    Marker a = markers[i];
    for(int j = 0; j < i; j++) {
      Marker b = markers[j];
      if(a.track == b.track) {
        Matrix<double, 2, Dynamic>  x(2, 2);
        if(a.image == image1 && b.image == image2) {
          x.col(0) = Vec2(a.x, a.y);
          x.col(1) = Vec2(b.x, b.y);
        } else {
          x.col(0) = Vec2(b.x, b.y);
          x.col(1) = Vec2(a.x, a.y);
        }
        Matrix<double, 4, 1> X;
        NViewTriangulate(x, Ps, &X);
        reconstruction->InsertPoint(a.track, X.row(0).head<3>());
      }
    }
  }
  return true;
}

void Bundle(const Tracks &/*tracks*/, Reconstruction */*reconstruction*/) {
  // XXX
}

}  // namespace libmv
