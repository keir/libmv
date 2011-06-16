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

#ifndef UI_TRACKER_MAIN_H_
#define UI_TRACKER_MAIN_H_

#include <QMainWindow>
#include <QSpinBox>
#include <QSlider>
#include <QCache>
#include <QTimer>

class Tracker;
class Zoom;
class Scene;

class Clip : public QObject, public QList<QString>   {
  Q_OBJECT
 public:
  Clip(QObject* parent=0) : QObject(parent) {}
  void Open(QString path);
  QImage Image(int frame);
 private:
  QCache<int, QImage> cache_;
};

class MainWindow : public QMainWindow {
  Q_OBJECT
 public:
  MainWindow();
  ~MainWindow();

 public slots:
  void open();
  void open(QString);
  void seek(int);
  void stop();
  void first();
  void previous();
  void next();
  void last();
  void toggleTracking(bool);
  void toggleBackward(bool);
  void toggleForward(bool);
  void toggleZoom(bool);
  void updateZooms(QVector<int>);

 private:
  QByteArray Load(QString name);
  void Save(QString name,QByteArray data);

  QString path_;
  Clip *clip_;
  Scene *scene_;
  Tracker *tracker_;
  QVector<Zoom*> zooms_;
  QVector<QDockWidget*> zooms_docks_;
  int current_frame_;
  QAction *track_action_;
  QAction *backward_action_;
  QAction *forward_action_;
  QSpinBox spinbox_;
  QSlider slider_;
  QTimer previous_timer_;
  QTimer next_timer_;
};
#endif

