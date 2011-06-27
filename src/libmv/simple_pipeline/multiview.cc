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
#include "libmv/logging/logging.h"
#include "libmv/multiview/euclidean_resection.h"
#include "libmv/multiview/resection.h"
#include "libmv/multiview/projection.h"
#include "libmv/multiview/fundamental.h"
#include "libmv/multiview/nviewtriangulation.h"
#include "libmv/numeric/numeric.h"
#include "libmv/simple_pipeline/reconstruction.h"
#include "libmv/simple_pipeline/tracks.h"

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

  // Recover motion between the two images. Since this function assumes a
  // calibrated camera, use the identity for K.
  Mat3 R;
  Vec3 t;
  Mat3 K = Mat3::Identity();
  if (!MotionFromEssentialAndCorrespondence(
        E, K, x1.col(0), K, x2.col(0), &R, &t)) {
    return false;
  }

  reconstruction->InsertCamera(image1, Mat3::Identity(), Vec3::Zero());
  reconstruction->InsertCamera(image2, R, t);

  LG << "From two frame reconstruction got:\nR:\n" << R
     << "\nt:\n" << t;
  return true;
}

Mat2X PointMatrixFromMarkers(const vector<Marker> &markers) {
  Mat2X points(2, markers.size());
  for (int i = 0; i < markers.size(); ++i) {
    points(0, i) = markers[i].x;
    points(1, i) = markers[i].y;
  }
  return points;
}

bool Resect(const vector<Marker> &markers, Reconstruction *reconstruction) {
  if (markers.size() < 5) {
    return false;
  }
  Mat2X points_2d = PointMatrixFromMarkers(markers);
  //Mat4X points_3d(4, markers.size());
  //for (int i = 0; i < markers.size(); i++) {
    //points_3d.col(i).head<3>() =
      //reconstruction->PointForTrack(markers[i].track)->X;
    //points_3d(3, i) = 1.0;
  //}
  Mat3X points_3d(3, markers.size());
  for (int i = 0; i < markers.size(); i++) {
    points_3d.col(i) = reconstruction->PointForTrack(markers[i].track)->X;
  }
  LG << "Points for intersect:\n" << points_2d;

  Mat3 R, K;
  Vec3 t;

  //Mat34 P;
  //Mat3 K;
  //resection::Resection(points_2d, points_3d, &P);
  //KRt_From_P(P, &K, &R, &t);
  //LG << "Resection for image " << markers[0].image << " got:\n"
     //<< "K:\n" << K << "\nR:\n" << R << "\nt:\n" << t;

  // TODO(keir): Figure out why euclidean resection is not working.
  euclidean_resection::EuclideanResection(points_2d, points_3d, &R, &t);
  LG << "Resection for image " << markers[0].image << " got:\n"
     << "R:\n" << R << "\nt:\n" << t;

  reconstruction->InsertCamera(markers[0].image, R, t);
  return true;
}

bool Intersect(const vector<Marker> &markers, Reconstruction *reconstruction) {
  if (markers.size() < 2) {
    return false;
  }

  // Compute projective camera matrices for the cameras the intersection is
  // going to use.
  Mat3 K = Mat3::Identity();
  vector<Mat34> cameras;
  Mat34 P;
  for (int i = 0; i < markers.size(); ++i) {
    Camera camera = *reconstruction->CameraForImage(markers[i].image);
    P_From_KRt(K, camera.R, camera.t, &P);
    cameras.push_back(P);
  }

  // Stack the 2D coordinates together as required by NViewTriangulate.
  Mat2X points(2, markers.size());
  for (int i = 0; i < markers.size(); ++i) {
    points(0, i) = markers[i].x;
    points(1, i) = markers[i].y;
  }

  Vec4 X;
  NViewTriangulate(points, cameras, &X);
  X /= X(3);

  Vec3 point = X.head<3>();
  reconstruction->InsertPoint(markers[0].track, point);

  // TODO(keir): Add proper error checking.
  return true;
}

void Bundle(const Tracks &/*tracks*/, Reconstruction */*reconstruction*/) {
  // XXX
}

}  // namespace libmv
