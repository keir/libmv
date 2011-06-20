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
    Initialize the \link Reconstruction reconstruction \endlink using the first two keyframes.

    \a markers should contain all \l Marker markers \endlink belonging to
       tracks visible in both keyframes.
    The pose estimation of the camera for these keyframes will be inserted in \a reconstruction.

    \note The two keyframes need to have both enough parallax and enough common tracks
          for accurate reconstruction.
    \note the origin of the coordinate system is defined to be the camera of the first keyframe.

    \sa Resect
*/
bool ReconstructTwoFrames(const std::vector<Marker> &markers,
                          Reconstruction *reconstruction);

/*!
    Estimate the pose of a new keyframe from a previous keyframe.

    \a markers should contain all \l Marker markers \endlink belonging to
       tracks visible in both keyframes.
    \a reconstruction should contain the \l Camera camera \endlink for one keyframe.
       The camera for the other keyframe will be inserted in \a reconstruction.

    \note The two keyframes need to have both enough parallax and enough common tracks
          for accurate reconstruction.

    \sa ReconstructTwoFrames
*/
bool Resect(const std::vector<Marker> &markers,
            Reconstruction *reconstruction);

/*!
    Estimate the 3D coordinates by intersecting markers between two keyframes.

    \a markers should contain all \l Marker markers \endlink belonging to
       tracks visible in both keyframes.
    \a reconstruction should contain the cameras for both keyframes.
       The new \l Point points \endlink will be inserted in \a reconstruction.

    \note The two keyframes need to have both enough parallax and enough common tracks
          for accurate reconstruction.

    \sa Resect
*/
bool Intersect(const std::vector<Marker> &markers,
               Reconstruction *reconstruction);

/*!
    Optimize the camera pose estimation and scene 3D coordinates using bundle adjustment.

    \a \l Tracks structure containing all markers used in the reconstruction.
    \a reconstruction should contain the cameras for all processed keyframes.
    \a reconstruction should contain the points for all processed keyframes.
       The cameras and the points will be refined in place in \a reconstruction.

    \sa Resect, Intersect
*/
void Bundle(const Tracks &tracks, Reconstruction *reconstruction);

}  // namespace libmv
