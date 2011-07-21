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

#ifndef LIBMV_SIMPLE_PIPELINE_AUTOCALIBRATE_H_
#define LIBMV_SIMPLE_PIPELINE_AUTOCALIBRATE_H_

namespace libmv {

class CameraIntrinsics;
class EuclideanReconstruction;
class ProjectiveReconstruction;

/*!
    Upgrade a projective reconstruction to Euclidean.

    The code assumes the projective reconstruction was done with normalized
    tracks; in other words, an approximate calibration has already been
    inverted.

    The algorthim is based on the one described in "Practical Autocalibration"
    by Riccardo Gherardi and Andrea Fusiello that appeared at ECCV 2010.

    \a keyframe1 and \a keyframe2 should have a large baseline.
*/
void UpgradeProjectiveReconstructionToEuclidean(
    int keyframe1,
    int keyframe2,
    const ProjectiveReconstruction &projective_reconstruction,
    EuclideanReconstruction *euclidean_reconstruction,
    CameraIntrinsics *camera_intrinsics);

}  // namespace libmv

#endif  // LIBMV_SIMPLE_PIPELINE_AUTOCALIBRATE_H_
