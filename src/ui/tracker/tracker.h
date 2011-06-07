// Copyright 2011 libmv authors
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
  QScopedPointer<TrackerScene> scene;
  QGraphicsPixmapItem* pixmap;
  QVector<QGraphicsPathItem*> tracks;

// View
  QSpinBox frameNumber;
  QAction* playAction;
  QSlider slider;
  QDockWidget* detailDock;
  View* view;
  View* zoomView;
  QTimer playTimer;
  int current_;
};
#endif
