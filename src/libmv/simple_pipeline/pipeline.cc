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

#include "libmv/simple_pipeline/multiview.h"
#include "libmv/simple_pipeline/reconstruction.h"
#include "libmv/simple_pipeline/tracks.h"

namespace libmv {

void CompleteReconstruction(const Tracks &tracks,
                            Reconstruction *reconstruction) {
  int max_track = tracks.MaxTrack();
  int max_image = tracks.MaxImage();
  int num_resects = -1;
  int num_intersects = -1;
  while (num_resects != 0 || num_intersects != 0) {
    // Do all possible intersections.
    num_intersects = 0;
    for (int track = 0; track < max_track; ++track) {
      if (reconstruction->PointForTrack(track)) {
        continue;
      }
      vector<Marker> all_markers = tracks.MarkersForTrack(track);

      vector<Marker> reconstructed_markers;
      for (int i = 0; i < all_markers.size(); ++i) {
        if (reconstruction->CameraForImage(all_markers[i].image)) {
          reconstructed_markers.push_back(all_markers[i]);
        }
      }
      if (reconstructed_markers.size() >= 2) {
        Intersect(reconstructed_markers, reconstruction);
        num_intersects++;
      }
    }
    if (num_intersects) {
      Bundle(tracks, reconstruction);
    }

    // Do all possible resections.
    num_resects = 0;
    for (int image = 0; image < max_image; ++image) {
      if (reconstruction->CameraForImage(image)) {
        continue;
      }
      vector<Marker> all_markers = tracks.MarkersInImage(image);

      vector<Marker> reconstructed_markers;
      for (int i = 0; i < all_markers.size(); ++i) {
        if (reconstruction->PointForTrack(all_markers[i].track)) {
          reconstructed_markers.push_back(all_markers[i]);
        }
      }
      if (reconstructed_markers.size() >= 4) {
        Resect(reconstructed_markers, reconstruction);
        num_resects++;
      }
    }
    if (num_resects) {
      Bundle(tracks, reconstruction);
    }
  }
}

}  // namespace libmv
