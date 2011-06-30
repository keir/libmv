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

#define V3DLIB_ENABLE_SUITESPARSE 1

#include <map>

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
#include "third_party/ssba/Math/v3d_linear.h"
#include "third_party/ssba/Math/v3d_linear_utils.h"
#include "third_party/ssba/Geometry/v3d_metricbundle.h"
#include "third_party/ssba/Geometry/v3d_cameramatrix.h"

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
     << (((a - b) / std::max(a, b)) * 100) << "%; singular values:"
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
  Vec operator()(const Vec6 &dRt) const {
    // Unpack R, t from dRt.
    Mat3 R = RotationFromEulerVector(dRt.head<3>()) * initial_R;
    Vec3 t = dRt.tail<3>();

    // Compute the reprojection error for each coordinate.
    Vec residuals(2 * markers.size());
    residuals.setZero();
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
  if (1 || !euclidean_resection::EuclideanResection(points_2d, points_3d, &R, &t)) {
    LG << "Resection for image " << markers[0].image << " failed;"
       << " trying fallback projective resection.";
    // Euclidean resection failed. Fall back to projective resection, which is
    // less reliable but better conditioned when there are many points.
    Mat34 P;
    Mat4X points_3d_homogeneous(4, markers.size());
    for (int i = 0; i < markers.size(); i++) {
      points_3d_homogeneous.col(i).head<3>() = points_3d.col(i);
      points_3d_homogeneous(3, i) = 1.0;
    }
    resection::Resection(points_2d, points_3d_homogeneous, &P);
    if ((P * points_3d_homogeneous.col(0))(2) < 0) {
      LG << "Point behind camera; switch sign.";
      P = -P;
    }

    Mat3 ignored;
    KRt_From_P(P, &ignored, &R, &t);

    // The R matrix should be a rotation, but don't rely on it.
    Eigen::JacobiSVD<Mat3> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);

    LG << "Resection rotation is: " << svd.singularValues().transpose();
    LG << "Determinant is: " << R.determinant();

    // Correct to make R a rotation.
    R = svd.matrixU() * svd.matrixV().transpose();

    Vec3 xx = R * points_3d + t;
    if (xx(2) < 0.0) {
      LG << "Final point is still behind camera...";
    }
    // XXX Need to check if error is horrible and fail here too in that case.
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

  Vec operator()(const Vec3 &X) const {
    Vec residuals(2 * markers.size());
    residuals.setZero();
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

void Bundle(const Tracks &tracks, Reconstruction *reconstruction) {
  vector<Marker> markers = tracks.AllMarkers();

  // "index" in this context is the index that V3D's optimizer will see. The
  // V3D index must be dense in that the cameras are numbered 0...n-1, which is
  // not the case for the "image" numbering that arises from the tracks
  // structure. The complicated mapping is necessary to convert between the two
  // representations.
  std::map<Camera *, int> camera_to_index;
  std::map<Point *, int> point_to_index;
  vector<Camera *> index_to_camera;
  vector<Point *> index_to_point;
  int num_cameras = 0;
  int num_points = 0;
  for (int i = 0; i < markers.size(); ++i) {
    const Marker &marker = markers[i];
    Camera *camera = reconstruction->CameraForImage(marker.image);
    Point *point = reconstruction->PointForTrack(marker.track);
    if (camera && point) {
      if (camera_to_index.find(camera) == camera_to_index.end()) {
        camera_to_index[camera] = num_cameras;
        index_to_camera.push_back(camera);
        num_cameras++;
      }
      if (point_to_index.find(point) == point_to_index.end()) {
        point_to_index[point] = num_points;
        index_to_point.push_back(point);
        num_points++;
      }
    }
  }

  // Make a V3D identity matrix, needed in a few places for K, since this
  // assumes a calibrated setup.
  V3D::Matrix3x3d identity3x3;
  identity3x3[0][0] = 1.0;
  identity3x3[0][1] = 0.0;
  identity3x3[0][2] = 0.0;
  identity3x3[1][0] = 0.0;
  identity3x3[1][1] = 1.0;
  identity3x3[1][2] = 0.0;
  identity3x3[2][0] = 0.0;
  identity3x3[2][1] = 0.0;
  identity3x3[2][2] = 1.0;

  // Convert libmv's cameras to V3D's cameras.
  std::vector<V3D::CameraMatrix> v3d_cameras(index_to_camera.size());
  for (int k = 0; k < index_to_camera.size(); ++k) {
    V3D::Matrix3x3d R;
    V3D::Vector3d t;

    // Libmv's rotation matrix type.
    const Mat3 &R_libmv = index_to_camera[k]->R;
    const Vec3 &t_libmv = index_to_camera[k]->t;

    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        R[i][j] = R_libmv(i, j);
      }
      t[i] = t_libmv(i);
    }
    v3d_cameras[k].setIntrinsic(identity3x3);
    v3d_cameras[k].setRotation(R);
    v3d_cameras[k].setTranslation(t);
  }
  LG << "Number of cameras: " << index_to_camera.size();

  // Convert libmv's points to V3D's points.
  std::vector<V3D::Vector3d> v3d_points(index_to_point.size());
  for (int i = 0; i < index_to_point.size(); i++) {
    v3d_points[i][0] = index_to_point[i]->X(0);
    v3d_points[i][1] = index_to_point[i]->X(1);
    v3d_points[i][2] = index_to_point[i]->X(2);
  }
  LG << "Number of points: " << index_to_point.size();

  // Convert libmv's measurements to v3d measurements.
  int num_residuals = 0;
  std::vector<V3D::Vector2d> v3d_measurements;
  std::vector<int> v3d_camera_for_measurement;
  std::vector<int> v3d_point_for_measurement;
  for (int i = 0; i < markers.size(); ++i) {
    Camera *camera = reconstruction->CameraForImage(markers[i].image);
    Point *point = reconstruction->PointForTrack(markers[i].track);
    if (!camera || !point) {
      continue;
    }
    V3D::Vector2d v3d_point;
    v3d_point[0] = markers[i].x;
    v3d_point[1] = markers[i].y;
    v3d_measurements.push_back(v3d_point);
    v3d_camera_for_measurement.push_back(camera_to_index[camera]);
    v3d_point_for_measurement.push_back(point_to_index[point]);
    num_residuals++;
  }
  LG << "Number of residuals: " << num_residuals;
  
  V3D::StdDistortionFunction v3d_distortion;
  v3d_distortion.k1 = 0;
  v3d_distortion.k2 = 0;
  v3d_distortion.p1 = 0;
  v3d_distortion.p2 = 0;

  // Finally, run the bundle adjustment.
  V3D::optimizerVerbosenessLevel = 100;
  double const inlierThreshold = 50000.0;  // XXX big!
  V3D::CommonInternalsMetricBundleOptimizer opt(V3D::FULL_BUNDLE_METRIC,
                                                inlierThreshold,
                                                identity3x3,
                                                v3d_distortion,
                                                v3d_cameras,
                                                v3d_points,
                                                v3d_measurements,
                                                v3d_camera_for_measurement,
                                                v3d_point_for_measurement);
  opt.maxIterations = 50;
  opt.minimize();
  LG << "Bundle status: " << opt.status;

  // Convert V3D's cameras back to libmv's cameras.
  for (int k = 0; k < num_cameras; k++) {
    V3D::Matrix3x4d const Rt = v3d_cameras[k].getOrientation();
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        index_to_camera[k]->R(i, j) = Rt[i][j];
      }
      index_to_camera[k]->t(i) = Rt[i][3];
    }
  }

  // Convert V3D's cameras back to libmv's cameras.
  for (int k = 0; k < num_points; k++) {
    for (int i = 0; i < 3; ++i) {
      index_to_point[k]->X(i) = v3d_points[k][i];
    }
  }
}

}  // namespace libmv
