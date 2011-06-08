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

#ifndef LIBMV_SIMPLE_PIPELINE_TRACKS_H_
#define LIBMV_SIMPLE_PIPELINE_TRACKS_H_

#include <vector>

namespace libmv {

/*!
    The Marker structure represents a 2D reconstruction marker.

    x, y represents the position of the marker (measured in pixels, from the
    top left corner) in the image identified by \l image.
    All markers which corresponds to the same target form a track identified by
    a common \l track number.

    \note Markers are typically aggregated with the help of the \l Tracks class.

    \sa Tracks
*/
struct Marker {
  double x, y;
  int image;
  int track;
};

/*!
    The Tracks class provides an abstraction for the storage of
    \l{Marker}{reconstruction markers}.

    It is a container to be used for both matchmoving and multiview purposes.
    It will provide fast lookup of the markers from image and track keys.

    To insert or edit a marker, use \l Insert().
    You can retrieve markers using \l MarkersInImage() and \l MarkersInTrack().
    \l MarkersForTracksInBothImages() is provided to conveniently retrieve
    matching tracks.
    All markers belonging to a track can be removed using
    \l RemoveMarkersForTrack().

    \sa Marker
*/
class Tracks {
 public:
/*!
    Inserts a marker item into the set.

    \a image and \a track are the keys used to retrieve the markers.
    \note You can also use this method to edit existing markers.
    \note You can generate a new track identifier using \l MaxTrack() + 1
*/
  void Insert(int image, int track, double x, double y);
/*!
    Returns an std::vector containing all markers for tracks visible in both images.

    \a image1 and \a image2 are the identifiers used when the markers were inserted.
*/
  void MarkersForTracksInBothImages(int image1, int image2,
                                    std::vector<Marker> *markers);
/*!
    Returns an std::vector containing all markers visible in an image.
*/
  void MarkersInImage(int image, std::vector<Marker> *markers);
/*!
    Returns an std::vector containing all markers belonging to a track.
*/
  void MarkersInTrack(int track, std::vector<Marker> *markers);
/*!
    Removes all markers belonging to \a track.
*/
  void RemoveMarkersForTrack(int track);
/*!
    Returns the maximum image identifier used.
*/
  int MaxImage() const;
/*!
    Returns the maximum track identifier used.
*/
  int MaxTrack() const;

 private:
  std::vector<Marker> markers_;
};

}  // namespace libmv

#endif  // LIBMV_SIMPLE_PIPELINE_MARKERS_H_
