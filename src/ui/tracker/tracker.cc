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

#include "ui/tracker/tracker.h"
#include "ui/tracker/gl.h"

#include "libmv/image/image.h"
#include "libmv/simple_pipeline/tracks.h"
#include "libmv/tracking/klt_region_tracker.h"
#include "libmv/tracking/trklt_region_tracker.h"
#include "libmv/tracking/pyramid_region_tracker.h"
#include "libmv/tracking/retrack_region_tracker.h"

#include <QMouseEvent>

using std::vector;
using libmv::Marker;

// Copy the region starting at *x0, *y0 with width w, h into region. If the
// region asked for is outside the image border, clipping is done and the
// returned region is smaller than requested. At return, *x0 and *y0 contain
// the top left pixel of *region in the original image (which may not match
// what was passed in). Returns true if any values were copied.
bool CopyRegionFromQImage(QImage image,
                          int w, int h,
                          int *x0, int *y0,
                          libmv::FloatImage *region) {
  const unsigned char *data = image.bits();
  int width = image.width();
  int height = image.height();

  // Clip the region on the upper right.
  if (*x0 < 0) {
    w += *x0;
    *x0 = 0;
  }

  if (*y0 < 0) {
    h += *y0;
    *y0 = 0;
  }

  // Clip the region on the lower left.
  w = std::min(w, std::max(width - *x0, 0));
  h = std::min(h, std::max(height - *y0, 0));

  // The region is entirely outside the given image.
  if (w <= 0 || h <= 0) {
    return false;
  }

  // Now that clipping is done, do the blit.
  region->resize(h, w);
  for (int y = *y0; y < *y0 + h; ++y) {
    for (int x = *x0; x < *x0 + w; ++x) {
      // This assumes BGR row-major ordering for the QImage's raw bytes.
      const unsigned char *pixel = data + (y * width + x) * 4;
      (*region)(y - *y0, x - *x0, 0) = (0.2126 * pixel[2] +
                                        0.7152 * pixel[1] +
                                        0.0722 * pixel[0]) / 255;
    }
  }
  return true;
}

// TODO(MatthiasF) shouldn't this be moved to RegionTracker API ?
libmv::RegionTracker *CreateRegionTracker() {
  libmv::TrkltRegionTracker *trklt_region_tracker = new libmv::TrkltRegionTracker;
  trklt_region_tracker->half_window_size = 5;
  trklt_region_tracker->max_iterations = 200;

  libmv::PyramidRegionTracker *pyramid_region_tracker =
      new libmv::PyramidRegionTracker(trklt_region_tracker, 3);
  return new libmv::RetrackRegionTracker(pyramid_region_tracker, 0.2);
}

Tracker::Tracker(QWidget *parent, QGLWidget *shareWidget)
  : QGLWidget(QGLFormat(QGL::SampleBuffers),parent,shareWidget),
    tracks_(new libmv::Tracks()),
    region_tracker_(CreateRegionTracker()),
    current_image_(-1), active_track_(-1), dragged_(false) {}

Tracker::~Tracker() {}

void Tracker::Load(QByteArray data) {
  const Marker *markers = reinterpret_cast<const Marker *>(data.constData());
  for (size_t i = 0; i < data.size() / sizeof(Marker); ++i) {
    Marker marker = markers[i];
    tracks_->Insert(marker.image, marker.track, marker.x, marker.y);
  }
}

QByteArray Tracker::Save() {
  std::vector<Marker> markers = tracks_->AllMarkers();
  return QByteArray(reinterpret_cast<char *>(markers.data()),
                    markers.size() * sizeof(Marker));
}

void Tracker::SetImage(int image, QImage new_image, bool track) {
  int previous_image = current_image_;
  current_image_ = image;

  // Track active trackers from the previous image into this one.
  if (track) {
    vector<Marker> previous_markers = tracks_->MarkersInImage(previous_image);
    foreach (const Marker &marker, previous_markers) {
      if (!selected_tracks_.contains(marker.track)) {
        continue;
      }
      // TODO(keir): For now this uses a fixed size region. What's needed is
      // an extension to use custom sized boxes around the tracked point.
      int size = 64;
      int half_size = size / 2;

      // [xy][01] is the upper right box corner.
      int x0 = marker.x - half_size;
      int y0 = marker.y - half_size;
      libmv::FloatImage old_patch;
      if (!CopyRegionFromQImage(previous_image_, size, size,
                                &x0, &y0,
                                &old_patch)) {
        selected_tracks_.remove(selected_tracks_.indexOf(marker.track));
        continue;
      }

      int x1 = marker.x - half_size;
      int y1 = marker.y - half_size;
      libmv::FloatImage new_patch;
      if (!CopyRegionFromQImage(new_image, size, size,
                                &x1, &y1,
                                &new_patch)) {
        selected_tracks_.remove(selected_tracks_.indexOf(marker.track));
        continue;
      }

      double xx0 = marker.x - x0;
      double yy0 = marker.y - y0;
      double xx1 = marker.x - x1;
      double yy1 = marker.y - y1;
      if (!region_tracker_->Track(old_patch, new_patch,
                                 xx0, yy0,
                                 &xx1, &yy1)) {
        selected_tracks_.remove(selected_tracks_.indexOf(marker.track));
        continue;
      }
      tracks_->Insert(current_image_, marker.track, x1 + xx1, y1 + yy1);
    }
  }
  previous_image_ = new_image;

  makeCurrent();
  image_.upload(new_image);
  upload();
  emit trackChanged(selected_tracks_);
}

void Tracker::select(QVector<int> tracks) {
  selected_tracks_ = tracks;
  upload();
}

void Tracker::deleteSelectedMarkers() {
  foreach(int track, selected_tracks_) {
    tracks_->RemoveMarker(current_image_,track);
  }
  selected_tracks_.clear();
  upload();
  emit trackChanged(selected_tracks_);
}

void Tracker::deleteSelectedTracks() {
  foreach(int track, selected_tracks_) {
    tracks_->RemoveMarkersForTrack(track);
  }
  selected_tracks_.clear();
  upload();
  emit trackChanged(selected_tracks_);
}

// TODO(MatthiasF): custom pattern/search size
static const float kSearchWindowSize = 32;
static const float kPatternWindowSize = 5.5;

void Tracker::DrawMarker(libmv::Marker marker, QVector<vec2> &lines) {
  vec2 center = vec2(marker.x,marker.y);
  vec2 quad[] = { vec2(-1,-1), vec2(1,-1), vec2(1,1), vec2(-1,1) };
  for (int i=0; i<4; i++) {
    lines << center+kSearchWindowSize*quad[i];
    lines << center+kSearchWindowSize*quad[(i+1)%4];
  }
  for (int i=0; i<4; i++) {
    lines << center+kPatternWindowSize*quad[i];
    lines << center+kPatternWindowSize*quad[(i+1)%4];
  }
}

void Tracker::upload() {
  vector<Marker> markers = tracks_->MarkersInImage(current_image_);
  QVector<vec2> lines; lines.reserve(markers.size()*8);
  foreach(Marker marker, markers) {
    DrawMarker(marker,lines);
  }
  foreach(int track, selected_tracks_) {
    Marker marker = tracks_->MarkerInImageForTrack(current_image_,track);
    DrawMarker(marker,lines);
    DrawMarker(marker,lines);
    DrawMarker(marker,lines);
  }
  markers_.primitiveType=2;
  markers_.upload(lines.constData(), lines.count(), sizeof(vec2));
  update();
}

void Tracker::Render(int w, int h, int image, int track) {
  glBindWindow(w,h);
  static GLShader image_shader;
  if (!image_shader.id) {
    image_shader.compile(glsl("vertex image"),glsl("fragment image"));
  }
  image_shader.bind();
  image_shader["image"] = 0;
  image_.bind(0);
  int W=image_.width;
  int H=image_.height;
  float width,height;
  if (image >= 0 && track >= 0) {
    Marker marker = tracks_->MarkerInImageForTrack(image,track);
    vec2 center(marker.x,marker.y);
    vec2 min = (center-kSearchWindowSize)/vec2(W,H);
    vec2 max = (center+kSearchWindowSize)/vec2(W,H);
    glQuad(vec4(-1,1,min.x,min.y),vec4(1,-1,max.x,max.y));
  } else {
    if (W*h > H*w) { width=1; height=(float)(H*w)/(W*h); }
    else { height=1; width=(float)(W*h)/(H*w); }
    glQuad(vec4(-width,-height,0,1),vec4(width,height,1,0));
  }

  static GLShader marker_shader;
  if(!marker_shader.id) {
    marker_shader.compile(glsl("vertex transform marker"),glsl("fragment transform marker"));
  }
  marker_shader.bind();
  mat4 transform;
  if (image >= 0 && track >= 0) {
    Marker marker = tracks_->MarkerInImageForTrack(image,track);
    vec2 center(marker.x,marker.y);
    vec2 min = center-kSearchWindowSize;
    vec2 max = center+kSearchWindowSize;
    transform.translate(vec3(-1,-1,0));
    transform.scale(vec3(2.0/(max-min).x,2.0/(max-min).y,1));
    transform.translate(vec3(-min.x,-min.y,0));
  } else {
    transform.scale(vec3(2*width/W,-2*height/H,1));
    transform.translate(vec3(-W/2,-H/2,0));
    transform_=transform;
  }
  marker_shader["transform"] = transform;
  markers_.bind();
  markers_.bindAttribute(&marker_shader,"position",2);
  glAdditiveBlendMode();
  markers_.draw();
}

void Tracker::paintGL() {
  Render(width(),height());
}

void Tracker::mousePressEvent(QMouseEvent* e) {
  vec2 pos = transform_.inverse()*vec2(2.0*e->x()/width()-1,1-2.0*e->y()/height());
  last_position_=pos;
  vector<Marker> markers = tracks_->MarkersInImage(current_image_);
  foreach(Marker marker, markers) {
    vec2 center = vec2(marker.x,marker.y);
    if( pos > center-kSearchWindowSize && pos < center+kSearchWindowSize ) {
      active_track_ = marker.track;
      return;
    }
  }
  int new_track = tracks_->MaxTrack() + 1;
  tracks_->Insert(current_image_, new_track, pos.x, pos.y);
  selected_tracks_ << new_track;
  active_track_ = new_track;
  emit trackChanged(selected_tracks_);
  upload();
}

void Tracker::mouseMoveEvent(QMouseEvent* e) {
  vec2 pos = transform_.inverse()*vec2(2.0*e->x()/width()-1,1-2.0*e->y()/height());
  vec2 delta = pos-last_position_;
  // FIXME: a reference would avoid duplicate lookup
  Marker marker = tracks_->MarkerInImageForTrack(current_image_,active_track_);
  marker.x += delta.x;
  marker.y += delta.y;
  tracks_->Insert(current_image_, active_track_, marker.x, marker.y);
  upload();
  last_position_=pos;
  dragged_=true;
  emit trackChanged(selected_tracks_);
}

void Tracker::mouseReleaseEvent(QMouseEvent *) {
  if(!dragged_ && active_track_>=0) {
    if(selected_tracks_.contains(active_track_)) {
      selected_tracks_.remove(selected_tracks_.indexOf(active_track_));
    } else {
      selected_tracks_ << active_track_;
    }
    emit trackChanged(selected_tracks_);
  }
  active_track_ = -1;
  dragged_=false;
  upload();
}

Zoom::Zoom(QWidget *parent, Tracker *tracker)
  : QGLWidget(QGLFormat(QGL::SampleBuffers),parent,tracker),
    tracker_(tracker) {
  setMinimumSize(4*kSearchWindowSize,4*kSearchWindowSize);
  setSizePolicy(QSizePolicy::Fixed,QSizePolicy::Fixed);
}

void Zoom::SetMarker(int image, int track) {
  image_ = image;
  track_ = track;
  update();
}

void Zoom::paintGL() {
  tracker_->Render(width(),height(),image_,track_);
}
