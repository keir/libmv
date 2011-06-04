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

#ifndef LIBMV_CORRESPONDENCE_MATCHES_H_
#define LIBMV_CORRESPONDENCE_MATCHES_H_

#include <algorithm>
#include <vector>

#include "libmv/simple_pipeline/pipeline.h"

namespace libmv {

void FindKeyframes(const Tracks &tracks, vector<int> *keyframes) {
  keyframes->push_back(0);

  int last_keyframe = 0;
  int max_image = tracks.MaxImage();
  for (int i = 1; i < max_image; ++i) {
    vector<Marker> markers;
    tracks.TracksInBothImages(last_keyframe, i, &markers);
    double fgric = ComputeFGRIC(markers);
    double hgric = ComputeHGRIC(markers);
    if (fgric < hgric) {
      keyframes->push_back(i);
      last_keyframe = i;
    }
  }
}

// For the given set of images, try all possible pairings to find the one with
// the biggest difference in GRIC scores between the H and F models. A big
// difference in GRIC score suggests a good baseline from which to start.
bool FindBestPairToInitializeFrom(const Tracks &tracks,
                                  const vector<int> images,
                                  int *image1, int *image2) {
  double best_diff = -1.;
  for (int i = 0; i < images.size(); ++i) {
    for (int j = i; j < images.size(); ++j) {
      vector<Marker> markers;
      tracks.TracksInBothImages(last_keyframe, i, &markers);
      double fgric = ComputeFGRIC(markers);
      double hgric = ComputeHGRIC(markers);
      if (fgric < 0 || hgric < 0 || fgric > hgric) {
        continue;
      }
      double diff = hgric - fgric;
      if (diff > best_diff) {
        *image1 = i;
        *image2 = j;
        best_diff = diff;
      }
    }
  }
  return best_diff != -1.;
}


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
      vector<Marker> all_markers;
      tracks.ImagesWithTrack(track, &all_markers);

      vector<Marker> reconstructed_markers;
      for (int i = 0; i < all_markers.size(); ++i) {
        if (reconstruction->CameraForImage(all_markers[i].image)) {
          reconstructed_markers.push_back(all_markers[i]);
        }
      }
      if (reconstructed_markers.size() >= 2) {
        Intersect(tracks, reconstructed_markers, reconstruction);
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
      vector<Markers> all_markers;
      tracks.TracksInImage(image, &all_markers);

      vector<Marker> reconstructed_markers;
      for (int i = 0; i < all_markers.size(); ++i) {
        if (reconstruction->PointForTrack(all_markers[i].track)) {
          reconstructed_markers.push_back(all_markers[i]);
        }
      }
      if (reconstructed_markers.size() >= 4) {
        Resect(tracks, reconstructed_markers, reconstruction);
        num_resects++;
      }
    }
    if (num_resects) {
      Bundle(tracks, reconstruction);
    }
  }
}

bool Reconstruct(const Tracks &tracks, Reconstruction *reconstruction) {
  vector<int> keyframes;
  FindKeyframes(tracks, &keyframes);
  if (keyframes.size() < 2) {
    return false;
  }

  int image1, image2;
  if (!FindBestPairToInitializeFrom(tracks, keyframes, &image1, &image2)) {
    return false;
  }

  vector<Marker> markers;
  tracks.TracksInBothImages(image1, image2, &markers);
  if (!ReconstructTwoFrames(markers, reconstruction)) {
    return false;
  }
  CompleteReconstruction(tracks, reconstruction);
  return true;
}

}  // namespace libmv
