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

#include <cmath>
#include "libmv/multiview/homography.h"
#include "libmv/multiview/fundmental.h"

namespace libmv {

void CoordinatesForMarkersInImage(const vector<Markers> &markers,
                                  int image,
                                  Mat *coordinates) {
  // XXX
}

int image1, image2;
GetImagesInMarkers(markers, &image1, &image2);

double GRIC(const vector<double> &normalized_squared_residuals,
            int dimensions,
            int degrees_of_freedom,
            int dimension_of_data,
            double residual_cutoff) {
  double d = dimensions;
  double n = normalized_squared_residuals.size();
  double k = dimensions;
  double r = dimension_of_data;
  double threshold = residual_cutoff * (r - d);

  double e = 0;
  for (int i = 0; i < normalized_squared_residuals.size(); ++i) {
    e += min(normalized_squared_residuals[i], threshold);
  }
  return e + log(r) * d * n + log(r * n) * k;
}

// See page 287 equation (11.9) of Hartley & Zisserman.
static double SampsonError(const Mat3 &F, const Vec2 &x1, const Vec2 &x2) {
  Vec3 x(x1(0), x1(1), 1.0);
  Vec3 y(x2(0), x2(1), 1.0);
  Vec3 F_x = F * x;
  Vec3 Ft_y = F.transpose() * y;
  return Square(y.dot(F_x)) / (  F_x.head<2>().squaredNorm()
                              + Ft_y.head<2>().squaredNorm());
}

double ComputeFGRIC(const vector<Markers> &markers) {
  int image1, image2;
  GetImagesInMarkers(markers, &image1, &image2);

  Mat x1, x2;
  CoordinatesForMarkersInImage(markers, image1, &x1);
  CoordinatesForMarkersInImage(markers, image2, &x2);

  Mat3 F;
  NormalizedEightPointSolver(x1, x2, &F);

  vector<double> residuals(x1.cols());
  for (int i = 0; i < x1.cols(); ++i) {
    residuals[i] = SampsonError(F, x1.col(i), x2.col(i));

    // Dangerous hack alert: Hardcoding 0.1 expected variance (0.31 stddev).
    residuals[i] /= 0.1;
  }
  return GRIC(residuals, 3, 7, 4, 100 /* cutoff big; no outliers assumed. */);
}

static double HomographyError(const Mat3 &H, const Vec2 &x1, const Vec2 &x2) {
  Vec3 x(x1(0), x1(1), 1.0);
  Vec3 z = H * x;
  z(0) /= z(2);
  z(1) /= z(2);
  double dx = z(0) - y(0);
  double dy = z(1) - y(1);
  return dx*dx + dy*dy;
}

double ComputeHGRIC(const vector<Markers> &markers) {
  int image1, image2;
  GetImagesInMarkers(markers, &image1, &image2);

  Mat x1, x2;
  CoordinatesForMarkersInImage(markers, image1, &x1);
  CoordinatesForMarkersInImage(markers, image2, &x2);

  Mat3 H;
  Homography2DFromCorrespondencesLinear(x1, x2, &H);

  vector<double> residuals(x1.cols());
  for (int i = 0; i < x1.cols(); ++i) {
    residuals[i] = HomographyError(F, x1.col(i), x2.col(i));

    // Dangerous hack alert: Hardcoding 0.1 expected variance (0.31 stddev).
    residuals[i] /= 0.1;
  }
  return GRIC(residuals, 2, 8, 4, 100 /* cutoff big; no outliers assumed. */);
}

bool ReconstructTwoFrames(const vector<Markers> &markers,
                          Reconstruction *reconstruction) {
  if (markers.size() < 8) {
    return false;
  }

  int image1, image2;
  GetImagesInMarkers(markers, &image1, &image2);

  Mat x1, x2;
  CoordinatesForMarkersInImage(markers, image1, &x1);
  CoordinatesForMarkersInImage(markers, image2, &x2);

  Mat3 F, E;
  EightPointSolver(x1, x2, &F) {
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
  reconstruction->CameraForImage(image2)->t = dT;
}

void Bundle(const Tracks &tracks, Reconstruction *reconstruction) {
  // XXX
}

bool Resect(const vector<Marker> &markers, Reconstruction *reconstruction) {
  // XXX
}

bool Intersect(const vector<Marker> &markers, Reconstruction *reconstruction) {
  // XXX
}

}  // namespace libmv
