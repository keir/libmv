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

#ifndef SRC_UI_TRACKER_MAIN_H_
#define SRC_UI_TRACKER_MAIN_H_

#include <QApplication>
#include <QMainWindow>
#include <QToolBar>
#include <QSpinBox>
#include <QAction>
#include <QSlider>
#include <QTimer>

class Clip;
class Tracker;
class View;
class QGraphicsItem;
class QGraphicsPixmapItem;

class MainWindow : public QMainWindow {
  Q_OBJECT
 public:
  MainWindow();
  ~MainWindow();

 public slots:
  void open();
  void seek(int);
  void toggleTracking(bool);
  void toggleBackward(bool);
  void toggleForward(bool);
  void first();
  void previous();
  void next();
  void last();
  void stop();
  void viewTrack(QGraphicsItem*);

 protected:
  void resizeEvent(QResizeEvent *);

 private:
  void open(QString);

  QString path_;
  Clip *clip_;
  Tracker *tracker_;
  QGraphicsPixmapItem *pixmap_;

  QAction *track_action_;
  QAction *backward_action_;
  QAction *forward_action_;
  QSpinBox spinbox_;
  QSlider slider_;
  View *view_;
  View *zoom_view_;
  QTimer previous_timer_;
  QTimer next_timer_;
  int current_frame_;
};
#endif

