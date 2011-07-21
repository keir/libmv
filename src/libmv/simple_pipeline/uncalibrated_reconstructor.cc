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

#include "libmv/simple_pipeline/uncalibrated_reconstructor.h"

#include <cmath>
#include <cstdio>

#include "libmv/numeric/numeric.h"
#include "libmv/logging/logging.h"
#include "libmv/simple_pipeline/autocalibrate.h"
#include "libmv/simple_pipeline/camera_intrinsics.h"
#include "libmv/simple_pipeline/bundle.h"
#include "libmv/simple_pipeline/initialize_reconstruction.h"
#include "libmv/simple_pipeline/pipeline.h"
#include "libmv/simple_pipeline/reconstruction.h"

namespace libmv {

// Make a matrix to normalize image coordinates for "approximate" calibration.
// This takes the heursitic that the width and height of an image in pixels are
// approximately related to the focal length, and that the center of an image
// is the center of projection.
Mat3 NormalizerMatrix(int width, int height) {
  Mat3 M;
  double s = hypot(width, height);
  M << s, 0, width,
       0, s, height,
       0, 0, 2.0;
  M /= 2.0;
  return M.inverse();
}

// Normalizes a set of tracks according to the normalizer matrix.
void TransformTracks(Mat3 transform, const Tracks &tracks,
                     Tracks *normalized_tracks) {
  vector<Marker> markers = tracks.AllMarkers();
  for (int i = 0; i < markers.size(); ++i) {
    Vec3 a = Vec3(markers[i].x, markers[i].y, 1.0);
    Vec3 b = transform * a;
    markers[i].x = a(0) / a(2);
    markers[i].y = a(1) / a(2);
  }
  *normalized_tracks = Tracks(markers);
}

UncalibratedReconstructor::UncalibratedReconstructor(int width,
                                                     int height,
                                                     int keyframe1,
                                                     int keyframe2,
                                                     const Tracks &tracks) {
  // Save the raw tracks for later use.
  raw_tracks_ = tracks;

  LG << "Normalize all tracks with an approximate calibration.";
  getchar();
  Mat3 normalizer_matrix = NormalizerMatrix(width, height);
  TransformTracks(normalizer_matrix, tracks, &normalized_tracks_);

  LG << "Initialize the reconstruction; bundle (not implemented yet).";
  getchar();
  vector<Marker> initial_markers =
      normalized_tracks_.MarkersForTracksInBothImages(keyframe1, keyframe2);
  ProjectiveReconstructTwoFrames(initial_markers, &projective_reconstruction_);
  CameraIntrinsics default_intrinsics;
  ProjectiveReprojectionError(normalized_tracks_,
                              projective_reconstruction_,
                              default_intrinsics);

  LG << "Complete the projective reconstruction.";
  getchar();
  ProjectiveCompleteReconstruction(normalized_tracks_, &projective_reconstruction_);
  ProjectiveReprojectionError(normalized_tracks_,
                              projective_reconstruction_,
                              default_intrinsics);

  LG << "Upgrade the reconstruction to Euclidean.";
  getchar();
  UpgradeProjectiveReconstructionToEuclidean(keyframe1,
                                             keyframe2,
                                             projective_reconstruction_,
                                             &euclidean_reconstruction_,
                                             &camera_intrinsics_);

  LG << "Multiply the normalizer matrix into the calibration matrix and "
     << "update the intrinsic parameters.";
  getchar();
  Mat3 K = camera_intrinsics_.K();
  camera_intrinsics_.SetK(K * NormalizerMatrix(width, height).inverse());
  camera_intrinsics_.set_image_size(width, height);
  LG << "Got K matrix: " << camera_intrinsics_.K();

  LG << "Invert the intrinsics to get calibrated tracks.";
  getchar();
  InvertIntrinsicsForTracks(raw_tracks_,
                            camera_intrinsics_,
                            &calibrated_tracks_);

  LG << "Bundle adjust the final result.";
  getchar();
  EuclideanBundle(calibrated_tracks_, &euclidean_reconstruction_);
}

}  // namespace libmv
