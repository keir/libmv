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
#include <limits>

#include "libmv/logging/logging.h"
#include "libmv/multiview/projection.h"
#include "libmv/simple_pipeline/reconstruction.h"
#include "libmv/simple_pipeline/camera_intrinsics.h"

namespace libmv {

// Finds a transformation H such that P * H = [I|0].
Mat4 NormalizingTransform(const Mat34 &P) {
  // Assuming that
  // 
  //   P1 = [Q | q] ,
  //
  // then the transformation H, shown below,
  //
  //   H = [  Q^-1  | -Q^-1 * q ] ,
  //       [  0 0 0 |         1 ]
  //
  // brings the camera P1 to the identity since
  //
  //   [Q | q] * [  Q^-1  | -Q^-1 * q ] = [I | 0]
  //             [  0 0 0 |         1 ]
  //
  Mat3 Q_inverse = P.block<3, 3>(0, 0).inverse();
  Vec3 q = P.col(3);

  Mat4 H;
  H << Q_inverse,  q,
       0., 0., 0., 1.;
  return H;
}

// This is the worlds dumbest algorithm which finds R (a rotation) such that
//
//   R * x == [ ||x|| 0 0 ].
//
// If x is the vector, we can make a 3x3 matix [x * *], then take the QR
// decomposition giving
//
//   M = [x * *] = QR
//
// where, in this case, Q is a rotation and R is (confusingly) upper
// triangular. Then, multiplying both sides with Q^-1 ( == Q^T)
//
//   Q^T M = R
//
// Since the first column of R is of the form [y 0 0], and since rotations are
// norm-preserving, y = ||x|| and therefore Q^T is the required matrix.
Mat3 FindZeroingRotation(Vec3 x) {
  Mat3 M;

  // First column is x; this will transform to (||x||, 0, 0).
  M.col(0) = x;

  // An arbitrary vector perpendicular to x. Note that this vector is not
  // required to be perpendicular, but choose it so to keep the QR
  // decomposition stable. This may not be necessary, oh well.
  M.col(1) = Vec3(x(2), x(2), - x(0) - x(1));

  // Pick another arbitrary vector; this one perpendicular to the prevous two.
  M.col(2) = x.cross(M.col(1));

  // The inverse of Q is the answer.
  Mat3 Q_inv = M.householderQr().householderQ().transpose();
  return Q_inv;
}

// Assumes P1 = [I|0].
Vec3 ComputePlaneAtInfinityGivenFocalLength(double f,
                                            const Mat34 &P2) {
  // In the paper's equation 3, they use P2' = [Q2|q2], but instead take
  //
  //   P2' = [Q|q]
  //
  // for simplicity.
  Mat3 Q = P2.block<3, 3>(0, 0);
  Vec3 q = P2.col(3);

  // Since the cheriality of the reconstruction is already accounted for, take
  // lambda as unity.
  double lambda = 1.;

  // This assumes K is the same for both cameras, and centering is already
  // corrected for, so create K as such.
  Mat3 K;
  K << f,  0., 0.,
       0., f,  0.,
       0., 0., 1.;
  Mat3 K_inverse = K.inverse();

  // With K, q, and lambda, compute t2 for the second camera; note that the
  // transform by H made t1 = 0.
  Vec3 t2 = K_inverse * lambda * q;

  // Now find a rotation R such that R * t2 = [||t2|| 0 0].
  Mat3 R_star = FindZeroingRotation(t2);

  // Compute the W matrix based on equation (7), and extract w1..w3.
  Mat3 W = R_star * K_inverse * Q * K;
  Vec3 w1 = W.row(0);
  Vec3 w2 = W.row(1);
  Vec3 w3 = W.row(2);

  // Compute v based on equation (9).
  Vec3 v = (w2.cross(w3) / w3.norm() - w1) / t2.norm();

  return v;
}

Mat4 ComputeUpgradeHomographyFromCamerasGivenFocalLength(double f,
                                                         const Mat34 &P1,
                                                         const Mat34 &P2) {
  // Transform both cameras such that P1 = [I|0].
  Mat4 H_normalizer = NormalizingTransform(P1);
  Mat34 P2H = P2 * H_normalizer;

  // Find the plane at infinity.
  Vec3 v = ComputePlaneAtInfinityGivenFocalLength(f, P2H);
  VLOG(1) << "For v got:\n" << v;

  // Form the upgrading homography H.
  Mat4 H;
  H << f,  0., 0.,    0.,
       0., f,  0.,    0.,
       0., 0., 1.,    0.,

       v.transpose(), 1.;

  // The full upgrade homography is the concatenation of H_normalizer and H.
  return H_normalizer * H;
}

// This is similar to MATLAB's logspace.
vector<double> LogSpace(double start, double end, int steps) {
  double log_start = log(start);
  double log_end = log(end);
  double log_step = (log_end - log_start) / steps;
  vector<double> log_space;
  for (int i = 0; i < steps; ++i) {
    log_space.push_back(exp(log_start + log_step * i));
  }
  return log_space;
}

// Returns the relative difference as a fraction between x and y. For example,
// if x = 0.95 * y, then this returns 0.05 (== 1 - 0.95). Note: This makes no
// sense for numbers with differing signs.
double RelativeDifference(double x, double y) {
  double max_value = std::max(x, y);
  double min_value = std::min(x, y);
  if (max_value == 0.0) {
    return 0.0;
  }
  return 1.0 - (max_value - min_value) / max_value;
}

// Compute the squared cost for a projective matrix. This uses a slightly
// different cost formulation for the focal lengths than the one in the paper,
// since it is easier to understand what it means to constrain the aspect
// ratio, rather than the absolute difference of the focal lengths.
// Furthermore, since this assumes equal focal lengths, the recovered focal
// length is compared to the expected focal length.
double CostForProjectiveMatrix(double expected_focal_length, const Mat34 &P) {
  Mat3 K;
  Mat3 R;
  Vec3 t;
  KRt_From_P(P, &K, &R, &t);

  double fx = fabs(K(0, 0));
  double fy = fabs(K(1, 1));
  double average_focal_length = 0.5 * (fx + fy);
  double skew = fabs(K(0, 1));
  double cx = fabs(K(0, 2));
  double cy = fabs(K(1, 2));

  // Penalize deviation from the guessed focal length.
  double focal_length_error_cost = RelativeDifference(expected_focal_length,
                                                      average_focal_length);

  // Penalize deviation from equal aspect ratio.
  double fraction_off_equal_aspect_ratio_cost = RelativeDifference(fx, fy);

  // Penalize non-zero skew.
  double skew_cost = skew;

  // Penalize non-zero center of projection.
  double principal_point_cost = cx + cy;

  // Combine costs, with approximate weights.
  double weighted_cost =   1. / 0.05   * focal_length_error_cost +
                           1. / 0.001  * fraction_off_equal_aspect_ratio_cost +
                           1. / 0.0005 * skew_cost +
                           1. / 0.01   * principal_point_cost;

  // Rescale for small errors; doesn't matter since it's relative.
  weighted_cost /= 100;

  return weighted_cost * weighted_cost;
}

void UpgradeProjectiveReconstructionToEuclidean(
    int keyframe1,
    int keyframe2,
    const ProjectiveReconstruction &projective_reconstruction,
    EuclideanReconstruction *euclidean_reconstruction,
    CameraIntrinsics *camera_intrinsics) {
  const int kNumFocalsToTry = 100;

  Mat34 P1 = projective_reconstruction.CameraForImage(keyframe1)->P;
  Mat34 P2 = projective_reconstruction.CameraForImage(keyframe2)->P;

  vector<ProjectiveCamera> all_cameras =
    projective_reconstruction.AllCameras();

  // Compute the costs for each choice of focal length.
  vector<double> costs(kNumFocalsToTry, 0.0);
  vector<double> focal_lengths_to_try = LogSpace(0.1, 3.0, kNumFocalsToTry);
  vector<Mat4> upgrading_homographies(kNumFocalsToTry);
  for (int i = 0; i < focal_lengths_to_try.size(); ++i) {
    double f = focal_lengths_to_try[i];
    Mat4 H = ComputeUpgradeHomographyFromCamerasGivenFocalLength(f, P1, P2);
    upgrading_homographies[i] = H;
    VLOG(1) << "Upgrading homography:\n" << H;
    double cost = 0;
    for (int j = 0; j < all_cameras.size(); ++j) {
      if (all_cameras[i].image == keyframe1 ||
          all_cameras[i].image == keyframe2) {
        continue;
      }
      Mat34 euclidean_P = all_cameras[i].P * H;
      cost += CostForProjectiveMatrix(f, euclidean_P);
    }
    costs[i] = cost;
    VLOG(1) << "For focal length " << f << " got cost " << cost;
  }

  // Find the best calibration.
  double best_cost = std::numeric_limits<double>::max();
  double best_focal_length = std::numeric_limits<double>::max();
  Mat4 best_H;

  for (int i = 0; i < focal_lengths_to_try.size(); ++i) {
    if (costs[i] < best_cost) {
      best_focal_length = focal_lengths_to_try[i];
      best_H = upgrading_homographies[i];
      best_cost = costs[i];
    }
  }
  LG << "Best focal length was " << best_focal_length
     << " with cost " << best_cost;

  // Set the intrinsics to the best one. Since the reconstruction is
  // quasicalibrated, only the focal length needs setting.
  camera_intrinsics->SetK(Mat3::Identity());
  camera_intrinsics->SetFocalLength(best_focal_length);

  // Make the Euclidean cameras.
  for (int i = 0; i < all_cameras.size(); i++) {
    Mat34 P = all_cameras[i].P * best_H;
    Mat3 ignored;
    Mat3 R;
    Vec3 t;
    KRt_From_P(P, &ignored, &R, &t);
    euclidean_reconstruction->InsertCamera(all_cameras[i].image, R, t);
  }

  // Make the Euclidean points.
  vector<ProjectivePoint> all_points = projective_reconstruction.AllPoints();
  Mat4 Hinv = best_H.inverse();
  for (int i = 0; i < all_points.size(); ++i) {
    Vec4 X = Hinv * all_points[i].X;
    X /= X(3);
    euclidean_reconstruction->InsertPoint(all_points[i].track, X.head<3>());
  }
}

}  // namespace libmv
