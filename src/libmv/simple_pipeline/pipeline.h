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

#include "libmv/simple_pipeline/tracks.h"
#include "libmv/simple_pipeline/reconstruction.h"

namespace libmv {

/*!
    Estimate non-key camera poses and reconstruct scene 3D coordinates using all frames.

    This method should be used once all keyframes have been processed to compute
    cameras for all frames between the keyframes and to refine 3D reconstruction.

    \a tracks should contain all markers used in the reconstruction.
    \a reconstruction should contain the cameras for all processed keyframes.
    \a reconstruction should contain the points for all processed keyframes.
       Cameras for non-key frames will be added to \a reconstruction.

    \sa Resect, Intersect, Bundle
*/
void CompleteReconstruction(const Tracks &tracks,
                            Reconstruction *reconstruction);

}  // namespace libmv

