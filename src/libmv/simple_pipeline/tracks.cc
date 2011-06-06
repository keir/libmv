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

#include <algorithm>
#include <vector>

#include "libmv/numeric/numeric.h"
#include "libmv/simple_pipeline/tracks.h"

using std::vector;

namespace libmv {

Marker *Tracks::Insert(int image, int track, double x, double y) {
  // TODO(keir): Wow, this is quadratic for repeated insertions. Fix this by
  // adding a smarter data structure like a set<>.
  for (int i = 0; i < markers_.size(); ++i) {
    markers_[i].image;
    if (markers_[i].image == image &&
        markers_[i].track == track) {
      markers_[i].x = x;
      markers_[i].y = y;
      return &markers_[i];
    }
  }
  Marker marker = { image, track, x, y };
  markers_.push_back(marker);
  return &markers_.back();
}

void Tracks::TracksInBothImages(int image1,
                                int image2,
                                vector<Marker> *markers) {
  vector<int> image1_tracks;
  vector<int> image2_tracks;

  for (int i = 0; i < markers_.size(); ++i) {
    int image = markers_[i].image;
    if (image == image1) {
      image1_tracks.push_back(image);
    } else if (image == image2) {
      image2_tracks.push_back(image);
    }
  }

  std::sort(image1_tracks.begin(), image1_tracks.end());
  std::sort(image2_tracks.begin(), image2_tracks.end());

  vector<int> intersection;
  std::set_intersection(image1_tracks.begin(), image1_tracks.end(),
                        image2_tracks.begin(), image2_tracks.end(),
                        std::back_inserter(intersection));

  markers->clear();
  for (int i = 0; i < markers_.size(); ++i) {
    if (std::binary_search(intersection.begin(),
                           intersection.end(),
                           markers_[i].image)) {
      markers->push_back(markers_[i]);
    }
  }
}

void Tracks::TracksInImage(int image, vector<Marker> *markers) {
  markers->clear();
  for (int i = 0; i < markers_.size(); ++i) {
    if (image == markers_[i].image) {
      markers->push_back(markers_[i]);
    }
  }
}

void Tracks::ImagesWithTrack(int track, vector<Marker> *markers) {
  markers->clear();
  for (int i = 0; i < markers_.size(); ++i) {
    if (track == markers_[i].track) {
      markers->push_back(markers_[i]);
    }
  }
}
  
int Tracks::MaxImage() const {
  int max_image = -1;
  for (int i = 0; i < markers_.size(); ++i) {
    max_image = std::max(markers_[i].image, max_image);
  }
  return max_image;
}

int Tracks::MaxTrack() const {
  int max_track = -1;
  for (int i = 0; i < markers_.size(); ++i) {
    max_track = std::max(markers_[i].track, max_track);
  }
  return max_track;
}

}  // namespace libmv
