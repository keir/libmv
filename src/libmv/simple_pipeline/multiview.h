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
#include "libmv/simple_pipeline/tracks.h"
#include "libmv/simple_pipeline/reconstruction.h"

namespace libmv {

/*!
    Initialize the \link Reconstruction reconstruction \endlink using two frames.

    \a markers should contain all \l Marker markers \endlink belonging to
    tracks visible in both keyframes. The pose estimation of the camera for
    these keyframes will be inserted into \a *reconstruction.

    \note The two keyframes need to have both enough parallax and enough common tracks
          for accurate reconstruction. At least 8 tracks are suggested.
    \note The origin of the coordinate system is defined to be the camera of
          the first keyframe.
    \note This assumes a calibrated reconstruction, e.g. the markers are
          already corrected for camera intrinsics and radial distortion.
    \note This assumes an outlier-free set of markers.

    \sa Resect
*/
bool ReconstructTwoFrames(const std::vector<Marker> &markers,
                          Reconstruction *reconstruction);

/*!
    Estimate the pose of a camera from 2D to 3D correspondences.

    This takes a set of markers visible in one frame (which is the one to
    resection), such that the markers are also reconstructed in 3D in the
    reconstruction object, and solves for the pose and orientation of the
    camera for that frame.

    \a markers should contain all \l Marker markers \endlink belonging to
    tracks visible in both keyframes. Each of the tracks associated with the
    markers must be reconstructed in the \a *reconstruction object.

    \a *reconstruction should contain the 3D points associated with the tracks
    for the markers present in \a markers.

    \note The two keyframes need to have both enough parallax and enough common tracks
          for accurate reconstruction.
    \note This assumes a calibrated reconstruction, e.g. the markers are
          already corrected for camera intrinsics and radial distortion.
    \note This assumes an outlier-free set of markers.

    \sa ReconstructTwoFrames
*/
bool Resect(const std::vector<Marker> &markers,
            Reconstruction *reconstruction);

/*!
    Estimate the 3D coordinates of a track by intersecting rays from images.

    This takes a set of markers, where each marker is for the same track but
    different images, and reconstructs the 3D position of that track. Each of
    the frames for which there is a marker for that track must have a
    corresponding reconstructed camera in \a *reconstruction.

    \a markers should contain all \l Marker markers \endlink belonging to
       tracks visible in both keyframes.
    \a reconstruction should contain the cameras for both keyframes.
       The new \l Point points \endlink will be inserted in \a reconstruction.

    \note This assumes a calibrated reconstruction, e.g. the markers are
          already corrected for camera intrinsics and radial distortion.
    \note This assumes an outlier-free set of markers.

    \sa Resect
*/
bool Intersect(const std::vector<Marker> &markers,
               Reconstruction *reconstruction);

/*!
    Refine camera poses and 3D coordinates using bundle adjustment.

    This routine adjusts all cameras and points in \a *reconstruction. This
    assumes a full observation for reconstructed tracks; this implies that if
    there is a reconstructed 3D point (a bundle) for a track, then all markers
    for that track will be included in the minimization. \a tracks should
    contain markers used in the initial reconstruction.

    The cameras and bundles (3D points) are refined in-place.

    \note This assumes an outlier-free set of markers.
    \note This assumes a calibrated reconstruction, e.g. the markers are
          already corrected for camera intrinsics and radial distortion.

    \sa Resect, Intersect, ReconstructTwoFrames
*/
void Bundle(const Tracks &tracks, Reconstruction *reconstruction);

}  // namespace libmv
