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

#ifndef LIBMV_SIMPLE_PIPELINE_UNCALIBRATED_RECONSTRUCTOR_H_
#define LIBMV_SIMPLE_PIPELINE_UNCALIBRATED_RECONSTRUCTOR_H_

#include "libmv/simple_pipeline/reconstruction.h"
#include "libmv/simple_pipeline/tracks.h"
#include "libmv/simple_pipeline/camera_intrinsics.h"

namespace libmv {

class UncalibratedReconstructor {
 public:
  UncalibratedReconstructor(int width,
                            int height,
                            int keyframe1,
                            int keyframe2,
                            const Tracks &tracks);

  // Algorithm:
  // 1. normalize all tracks
  // 2. initialize the reconstruction; bundle (not implemented yet)
  // 3. complete the pipeline
  // 4. <bundle or not, grr>
  // 5. run autocalibration
  // 5. convert projectiveReconstruction to EuclideanReconstruction

  /// The raw tracks in pixel coordinates, unmodified.
  const Tracks &raw_tracks()        const { return raw_tracks_;        }

  /// Shifted and scaled raw tracks, to improve numerical conditioning.
  const Tracks &normalized_tracks() const { return normalized_tracks_; }

  /// The final calibrated tracks, after inverting camera intrinsics.
  const Tracks &calibrated_tracks() const { return calibrated_tracks_; }

  /// The estimated camera intrinsics.
  const CameraIntrinsics &camera_intrinsics() const {
    return camera_intrinsics_;
  }

  /// The projective reconstruction.
  const ProjectiveReconstruction projective_reconstruction() const {
    return projective_reconstruction_;
  }

  /// The full Euclidean reconstruction.
  const EuclideanReconstruction euclidean_reconstruction() const {
    return euclidean_reconstruction_;
  }

  Tracks raw_tracks_;
  Tracks normalized_tracks_;
  Tracks calibrated_tracks_;

  ProjectiveReconstruction projective_reconstruction_;

  EuclideanReconstruction euclidean_reconstruction_;

  CameraIntrinsics camera_intrinsics_;

  Mat3 normalizer_matrix_;
};

}  // namespace libmv

#endif  // LIBMV_SIMPLE_PIPELINE_UNCALIBRATED_RECONSTRUCTOR_H_
