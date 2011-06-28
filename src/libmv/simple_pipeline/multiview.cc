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
#include "libmv/multiview/triangulation.h"
#include "libmv/multiview/resection.h"
#include "libmv/multiview/projection.h"
#include "libmv/multiview/fundamental.h"
#include "libmv/multiview/nviewtriangulation.h"
#include "libmv/numeric/numeric.h"
#include "libmv/numeric/levenberg_marquardt.h"
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

  // See Hartley & Zisserman page 294, result 11.1, which shows how to get the
  // closest essential matrix to a matrix that is "almost" an essential matrix.
  double a = svd.singularValues()(0);
  double b = svd.singularValues()(1);
  double s = (a + b) / 2.0;
  LG << "Initial reconstruction's rotation is non-euclidean by "
     << (((a - b) / max(a, b)) * 100) << "%; singular values:"
     << svd.singularValues().transpose();

  Vec3 diag;
  diag << s, s, 0;
  Mat3 E = svd.matrixU() * diag.asDiagonal() * svd.matrixV().transpose();

  // Recover motion between the two images. Since this function assumes a
  // calibrated camera, use the identity for K.
  Mat3 R;
  Vec3 t;
  Mat3 K = Mat3::Identity();
  if (!MotionFromEssentialAndCorrespondence(E,
                                            K, x1.col(0),
                                            K, x2.col(0),
                                            &R, &t)) {
    return false;
  }

  // Image 1 gets the reference frame, image 2 gets the relative motion.
  reconstruction->InsertCamera(image1, Mat3::Identity(), Vec3::Zero());
  reconstruction->InsertCamera(image2, R, t);

  LG << "From two frame reconstruction got:\nR:\n" << R
     << "\nt:" << t.transpose();
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

Mat3 RotationFromEulerVector(Vec3 euler_vector) {
  double theta = euler_vector.norm();
  if (theta == 0.0) {
    return Mat3::Identity();
  }
  Vec3 w = euler_vector / theta;
  Mat3 w_hat = CrossProductMatrix(w);
  return Mat3::Identity() + w_hat*sin(theta) + w_hat*w_hat*(1 - cos(theta));
}

// Uses an incremental rotation:
//
//   x = R' * R * X + t;
//
// to avoid issues with the rotation representation. R' is derived from a
// euler vector encoding the rotation in 3 parameters; the direction is the
// axis to rotate around and the magnitude is the amount of the rotation.
struct ResectCostFunction {
 public:
  typedef Vec  FMatrixType;
  typedef Vec6 XMatrixType;

  ResectCostFunction(const vector<Marker> &markers,
                     const Reconstruction &reconstruction,
                     const Mat3 initial_R)
    : markers(markers),
      reconstruction(reconstruction),
      initial_R(initial_R) {}

  // dRt has dR (delta R) encoded as a euler vector in the first 3 parameters,
  // followed by t in the next 3 parameters.
  Vec2 operator()(const Vec6 &dRt) const {
    // Unpack R, t from dRt.
    Mat3 R = RotationFromEulerVector(dRt.head<3>()) * initial_R;
    Vec3 t = dRt.tail<3>();

    // Compute the reprojection error for each coordinate.
    Vec residuals(2 * markers.size());
    for (int i = 0; i < markers.size(); ++i) {
      const Point &point = *reconstruction.PointForTrack(markers[i].track);
      Vec3 projected = R * point.X + t;
      projected /= projected(2);
      residuals[2*i + 0] = projected(0) - markers[i].x;
      residuals[2*i + 1] = projected(1) - markers[i].y;
    }
    return residuals;
  }

  const vector<Marker> &markers;
  const Reconstruction &reconstruction;
  const Mat3 &initial_R;
};

bool Resect(const vector<Marker> &markers, Reconstruction *reconstruction) {
  if (markers.size() < 5) {
    return false;
  }
  Mat2X points_2d = PointMatrixFromMarkers(markers);
  Mat3X points_3d(3, markers.size());
  for (int i = 0; i < markers.size(); i++) {
    points_3d.col(i) = reconstruction->PointForTrack(markers[i].track)->X;
  }
  LG << "Points for resect:\n" << points_2d;

  Mat3 R;
  Vec3 t;
  if (!euclidean_resection::EuclideanResection(points_2d, points_3d, &R, &t)) {
    LG << "Resection for image " << markers[0].image << " failed.";
    return false;
  }

  // Refine the result.
  typedef LevenbergMarquardt<ResectCostFunction> Solver;

  // Give the cost our initial guess for R.
  ResectCostFunction resect_cost(markers, *reconstruction, R);

  // Encode the initial parameters: start with zero delta rotation, and the
  // guess for t obtained from resection.
  Vec6 dRt = Vec6::Zero();
  dRt.tail<3>() = t;

  Solver solver(resect_cost);

  Solver::SolverParameters params;
  Solver::Results results = solver.minimize(params, &dRt);
  LG << "LM found incremental rotation: " << dRt.head<3>().transpose();
  // TODO(keir): Check results to ensure clean termination.

  // Unpack the rotation and translation.
  R = RotationFromEulerVector(dRt.head<3>()) * R;
  t = dRt.tail<3>();

  LG << "Resection for image " << markers[0].image << " got:\n"
     << "R:\n" << R << "\nt:\n" << t;
  reconstruction->InsertCamera(markers[0].image, R, t);
  return true;
}

struct TriangulateCostFunction {
 public:
  typedef Vec  FMatrixType;
  typedef Vec3 XMatrixType;

  TriangulateCostFunction(const vector<Marker> &markers,
                          const Reconstruction &reconstruction)
    : markers(markers), reconstruction(reconstruction) {}

  Vec2 operator()(const Vec3 &X) const {
    Vec residuals(2 * markers.size());
    for (int i = 0; i < markers.size(); ++i) {
      const Camera &camera = *reconstruction.CameraForImage(markers[i].image);
      Vec3 projected = camera.R * X + camera.t;
      projected /= projected(2);
      residuals[2*i + 0] = projected(0) - markers[i].x;
      residuals[2*i + 1] = projected(1) - markers[i].y;
    }
    return residuals;
  }
  const vector<Marker> &markers;
  const Reconstruction &reconstruction;
};

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
    Camera *camera = reconstruction->CameraForImage(markers[i].image);
    P_From_KRt(K, camera->R, camera->t, &P);
    cameras.push_back(P);
  }

  // Stack the 2D coordinates together as required by NViewTriangulate.
  Mat2X points(2, markers.size());
  for (int i = 0; i < markers.size(); ++i) {
    points(0, i) = markers[i].x;
    points(1, i) = markers[i].y;
  }

  Vec4 Xp;
  LG << "Triangulating with " << markers.size() << " markers.";
  NViewTriangulateAlgebraic(points, cameras, &Xp);
  Xp /= Xp(3);

  // Get euclidean version.
  Vec3 X = Xp.head<3>();

  typedef LevenbergMarquardt<TriangulateCostFunction> Solver;

  TriangulateCostFunction triangulate_cost(markers, *reconstruction);
  Solver::SolverParameters params;
  Solver solver(triangulate_cost);

  Solver::Results results = solver.minimize(params, &X);

  // Try projecting the point; make sure it's in front of everyone.
  for (int i = 0; i < cameras.size(); ++i) {
    const Camera &camera = *reconstruction->CameraForImage(markers[i].image);
    Vec3 x = camera.R * X + camera.t;
    if (x(2) < 0) {
      LOG(ERROR) << "Point behind camera " << markers[i].image
                 << ": " << x.transpose();
    }
  }

  Vec3 point = X.head<3>();
  reconstruction->InsertPoint(markers[0].track, point);

  // TODO(keir): Add proper error checking.
  return true;
}

void Bundle(const Tracks &/*tracks*/, Reconstruction */*reconstruction*/) {
  // XXX
}

}  // namespace libmv
