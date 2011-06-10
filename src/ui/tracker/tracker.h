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

#ifndef UI_TRACKER_TRACKER_H_
#define UI_TRACKER_TRACKER_H_

#include <QGraphicsScene>
#include <QGraphicsItem>
#include <QSet>

namespace libmv {
class Tracks;
class RegionTracker;
} // namespace libmv

class TrackItem : public QGraphicsItem {
public:
  // TODO(MatthiasF): per track editable window sizes
  static const int kSearchWindowSize = 64;
  static const int kPatternWindowSize = 11;
  TrackItem(int track);
  inline int Track() { return track_; }

protected:
  QRectF boundingRect() const;
  void paint(QPainter *painter, const QStyleOptionGraphicsItem*, QWidget*);

private:
  int track_;
};

class Tracker : public QGraphicsScene {
  Q_OBJECT
 public:
  Tracker(QObject* parent=0);
  ~Tracker();
  void Load(QByteArray data);
  QByteArray Save();
  void SetFrame(int frame, QImage image, bool track);

 public slots:
  void deleteCurrentMarker();
  void deleteCurrentTrack();

 signals:
  void trackChanged(QGraphicsItem*);

 protected:
  void mousePressEvent(QGraphicsSceneMouseEvent*);
  void mouseMoveEvent(QGraphicsSceneMouseEvent*);
  void mouseReleaseEvent(QGraphicsSceneMouseEvent*);
  void mouseDoubleClickEvent(QGraphicsSceneMouseEvent*);

 private:
  QScopedPointer<libmv::Tracks> tracks_;
  QScopedPointer<libmv::RegionTracker> region_tracker_;
  QMap<int, TrackItem *> track_items_;
  QSet<int> tracks_in_previous_frame_;
  QImage previous_image_;
  int current_frame_;
  TrackItem* current_item_;
};


#endif
