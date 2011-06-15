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

#include <QGLWidget>
#include "ui/tracker/gl.h"

namespace libmv {
class Tracks;
class RegionTracker;
class Marker;
} // namespace libmv

class Tracker : public QGLWidget {
  Q_OBJECT
 public:
  Tracker(QWidget *parent = 0,QGLWidget *shareWidget = 0);
  ~Tracker();
  void Load(QByteArray data);
  QByteArray Save();
  void SetImage(int image, QImage new_image, bool track);
  void Render(int w, int h, int image=-1, int track=-1);

 public slots:
  void select(QVector<int>);
  void deleteSelectedMarkers();
  void deleteSelectedTracks();
  void upload();

 signals:
  void trackChanged(QVector<int>);

 protected:
  void paintGL();
  void mousePressEvent(QMouseEvent*);
  void mouseMoveEvent(QMouseEvent*);
  void mouseReleaseEvent(QMouseEvent *);

 private:
  void DrawMarker(libmv::Marker marker, QVector<vec2>& lines);

  QScopedPointer<libmv::Tracks> tracks_;
  QScopedPointer<libmv::RegionTracker> region_tracker_;
  QImage previous_image_;
  GLTexture image_;

  mat4 transform_;
  GLBuffer markers_;

  int current_image_;
  QVector<int> selected_tracks_;
  vec2 last_position_;
  int active_track_;
  bool dragged_;
};

class Zoom : public QGLWidget {
  Q_OBJECT
 public:
  Zoom(QWidget *parent = 0,Tracker *tracker = 0);
  void SetMarker(int image, int track);

 protected:
  void paintGL();
 private:
  Tracker* tracker_;
  int image_;
  int track_;
};

#endif
