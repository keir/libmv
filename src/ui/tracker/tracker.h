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

#ifndef SRC_UI_TRACKER_TRACKER_H_
#define SRC_UI_TRACKER_TRACKER_H_

#include <QApplication>
#include <QMainWindow>
#include <QToolBar>
#include <QSpinBox>
#include <QAction>
#include <QSlider>
#include <QGraphicsView>
#include <QTimer>
#include <QCache>

class Clip;
class TrackerScene;
class View;
class TrackItem;
namespace libmv {
class Tracks;
class RegionTracker;
}  // namespace libmv

class Tracker : public QMainWindow {
  Q_OBJECT
 public:
  Tracker();
  ~Tracker();

 public slots:
  void open();
  void seek(int);
  void first();
  void previous();
  void next();
  void last();
  void togglePlay(bool);
  void start();
  void stop();

  void selectMarker();
  void moveMarker();

 protected:
  void resizeEvent(QResizeEvent *);

 private:
  void open(QStringList);

// TODO(MatthiasF): separate concerns
// Data
  QScopedPointer<Clip> clip_;
  QScopedPointer<libmv::Tracks> tracks_;
  QScopedPointer<libmv::RegionTracker> region_tracker_;

// Model
  QScopedPointer<TrackerScene> scene_;
  QGraphicsPixmapItem *pixmap_;

// View
  QSpinBox frame_number_;
  QAction* play_action_;
  QSlider slider_;
  View* view_;
  View* zoom_view_;
  QTimer play_timer_;
  int current_frame_;
  TrackItem* current_item_;
};
#endif
