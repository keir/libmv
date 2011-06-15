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

#include <QGraphicsSceneMouseEvent>
#include <QPainter>

#include "libmv/simple_pipeline/tracks.h"
#include "libmv/logging/logging.h"
#include "libmv/image/image.h"
#include "libmv/tracking/klt_region_tracker.h"
#include "libmv/tracking/trklt_region_tracker.h"
#include "libmv/tracking/pyramid_region_tracker.h"
#include "libmv/tracking/retrack_region_tracker.h"

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

/// TrackItem

TrackItem::TrackItem(int track) : track_(track) {
  setFlags(QGraphicsItem::ItemIsSelectable|QGraphicsItem::ItemIsMovable);
}

QRectF TrackItem::boundingRect() const {
  return QRectF(-kSearchWindowSize/2, -kSearchWindowSize/2,
                kSearchWindowSize, kSearchWindowSize);
}
void TrackItem::paint(QPainter *painter, const QStyleOptionGraphicsItem*, QWidget*) {
  painter->setPen(QPen(QBrush( isSelected() ? Qt::green : Qt::darkGreen), 2));
  painter->drawRect(-kSearchWindowSize/2, -kSearchWindowSize/2,
                    kSearchWindowSize, kSearchWindowSize);
  if(isSelected()) {
    painter->setPen(QPen(QBrush(Qt::green), 1));
    painter->drawRect(-kPatternWindowSize/2, -kPatternWindowSize/2,
                      kPatternWindowSize, kPatternWindowSize);
  }
}

/// Tracker

Tracker::Tracker(QObject *parent)
  : QGraphicsScene(parent),
    tracks_(new libmv::Tracks()),
    region_tracker_(CreateRegionTracker()),
    current_item_(0) {}

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

void Tracker::SetFrame(int frame, QImage image, bool track) {
  if (frame == current_frame_) {
    LG << "Ignoring request to set frame to current frame.";
    return;
  }
  LG << "Setting frame to " << frame;

  int previous_frame = current_frame_;
  current_frame_ = frame;

  // Track active trackers from the previous frame into this one.
  if (track) {
    vector<Marker> previous_markers = tracks_->MarkersInImage(previous_frame);
    foreach (const Marker &marker, previous_markers) {
      if (!track_items_[marker.track]->isSelected()) {
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
        continue;
      }

      int x1 = marker.x - half_size;
      int y1 = marker.y - half_size;
      libmv::FloatImage new_patch;
      if (!CopyRegionFromQImage(image, size, size,
                                &x1, &y1,
                                &new_patch)) {
        continue;
      }

      double xx0 = marker.x - x0;
      double yy0 = marker.y - y0;
      double xx1 = marker.x - x1;
      double yy1 = marker.y - y1;
      if (region_tracker_->Track(old_patch, new_patch,
                                 xx0, yy0,
                                 &xx1, &yy1)) {
        tracks_->Insert(current_frame_, marker.track, x1 + xx1, y1 + yy1);
        LG << "Tracked (" << xx0 << ", " << yy0 << ") to ("
           << xx1 << ", " << yy1 << ").";
      }
    }
  }
  previous_image_ = image;

  vector<Marker> markers = tracks_->MarkersInImage(current_frame_);
  LG << "Got " << markers.size() << " markers.";

  // Set the position of the tracks for this frame.
  QSet<int> tracks_in_new_frame;
  foreach (const Marker &marker, markers) {
    // Create a visible set to find the tracks that disappeared from the
    // previous frame.
    tracks_in_new_frame << marker.track;

    // Set the position of the marker in the new frame.
    TrackItem*& track_item = track_items_[marker.track];
    if (!track_item) {
      track_item = new TrackItem(marker.track);
      addItem(track_item);
    }
    track_item->setPos(marker.x, marker.y);
    track_item->show();
  }

  // Hide any tracks that were visible in the last frame but not this one.
  foreach (int track, tracks_in_previous_frame_) {
    if (!tracks_in_new_frame.contains(track)) {
      track_items_[track]->hide();
    }
  }
  tracks_in_previous_frame_ = tracks_in_new_frame;

  if(current_item_) {
    current_item_->setSelected(true);
    emit trackChanged(current_item_);
  }
}

void Tracker::deleteCurrentMarker() {
  if(current_item_) {
    tracks_->RemoveMarker(current_frame_, current_item_->Track());
    current_item_->hide();
  }
}

void Tracker::deleteCurrentTrack() {
  if(current_item_) {
    tracks_->RemoveMarkersForTrack(current_item_->Track());
    current_item_->hide();
    current_item_ = 0;
  }
}

void Tracker::mousePressEvent(QGraphicsSceneMouseEvent *mouseEvent) {
  QGraphicsScene::mousePressEvent(mouseEvent);
  if(selectedItems().isEmpty()) {
    current_item_ = 0;
  } else {
    current_item_ = static_cast<TrackItem*>(selectedItems().first());
    emit trackChanged(current_item_);
  }
}

void Tracker::mouseMoveEvent(QGraphicsSceneMouseEvent *mouseEvent) {
  QGraphicsScene::mouseMoveEvent(mouseEvent);
  if(current_item_) {
    emit trackChanged(current_item_);
  }
}

void Tracker::mouseReleaseEvent(QGraphicsSceneMouseEvent *mouseEvent) {
  QGraphicsScene::mouseReleaseEvent(mouseEvent);
  if(current_item_) {
    tracks_->Insert(current_frame_,
                    current_item_->Track(),
                    current_item_->pos().x(),
                    current_item_->pos().y());
  }
}

void Tracker::mouseDoubleClickEvent(QGraphicsSceneMouseEvent *mouseEvent) {
  int x = mouseEvent->scenePos().x();
  int y = mouseEvent->scenePos().y();

  int new_track = tracks_->MaxTrack() + 1;

  TrackItem* item = current_item_ = new TrackItem(new_track);
  track_items_[new_track] = item;
  addItem(item);
  item->setPos(x, y);
  item->setSelected(true);

  tracks_->Insert(current_frame_, new_track, x, y);
  emit trackChanged(current_item_);
}
