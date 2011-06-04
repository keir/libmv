// Copyright 2011 libmv authors
// Initial revision by Matthias Fauconneau.
#include "tracker.h"
#include "klt.h"
#include <QFileDialog>
#include <QDesktopWidget>
#include <QGraphicsPixmapItem>
#include <QTime>
#include <QDebug>

/// minimal scope profiling tool
struct Profile {
  QTime time;
  Profile(const char* msg,const char* file,const int line) {
    fprintf(stderr,"%s:%d: %s... ",file,line,msg); fflush(stderr);
    time.start();
  }
  ~Profile() { fprintf(stderr,"%d ms\n",time.elapsed()); fflush(stderr); }
#define profile(msg) Profile p(msg,__FILE__, __LINE__)
};

int main(int argc, char *argv[]) {
  QApplication app(argc, argv);
  Tracker window;
  window.show();
  return app.exec();
}

Tracker::Tracker() : current(-2), klt(libmv::KLT::CreateContext()) {
  setWindowTitle("Qt Tracker");
  setMaximumSize(qApp->desktop()->availableGeometry().size());
  QToolBar* toolBar = addToolBar("Main Toolbar");
  toolBar->setObjectName("mainToolbar");
  toolBar->addAction(QIcon::fromTheme("document-open"), "Open...",
                     this, SLOT(open()));
  toolBar->addAction(QIcon::fromTheme("media-skip-backward"), "First Frame",
                     this, SLOT(first()));
  toolBar->addAction(QIcon::fromTheme("media-seek-backward"),"Previous Frame",
                     this, SLOT(previous()))->setShortcut(QKeySequence("Left"));
  toolBar->addWidget(&frameNumber);
  connect(&frameNumber, SIGNAL(valueChanged(int)), SLOT(seek(int)));
  playAction = toolBar->addAction(QIcon::fromTheme("media-playback-start"),
                                  QKeySequence("Play"));
  playAction->setCheckable(true);
  playAction->setShortcut(QKeySequence("Space"));
  connect(playAction, SIGNAL(triggered(bool)), SLOT(togglePlay(bool)));
  toolBar->addWidget(&slider);
  slider.setOrientation(Qt::Horizontal);
  connect(&slider, SIGNAL(sliderMoved(int)), SLOT(seek(int)));
  toolBar->addAction(QIcon::fromTheme("media-seek-forward"), "Next Frame",
                     this, SLOT(next()))
      ->setShortcut(QKeySequence("Right"));
  toolBar->addAction(QIcon::fromTheme("media-skip-forward"), "Last Frame",
                     this, SLOT(last()));
  view.setScene(&scene);
  view.setRenderHints(QPainter::Antialiasing|QPainter::SmoothPixmapTransform);
  view.setFrameShape(QFrame::NoFrame);
  view.setDragMode(QGraphicsView::ScrollHandDrag);
  view.setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  view.setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  setCentralWidget(&view);
  connect(&playTimer, SIGNAL(timeout()), SLOT(next()));
  QStringList args = qApp->arguments();
  args.removeFirst();
  if(args.isEmpty()) open(); else open(args);
}

void Tracker::open() {
  open(QFileDialog::getOpenFileNames(this, tr("Select Images"), "",
                                     "Images (*.png *.jpg);;All Files (*.*)"));
}
void Tracker::open(QStringList paths) {
  if(paths.isEmpty()) return;
  images.clear();
  pyramids.clear();
  pyramids.setMaxCost(3);
  scene.clear();
  pixmap=0;
  foreach(QString path,paths) {
    if(QFileInfo(path).isFile()) {
      images << path;
    } else {
      foreach(QString file,QDir(path).entryList(QStringList("*.jpg")<<"*.png",
                                                QDir::Files,QDir::Name)) {
        images << QDir(path).filePath(file);
      }
    }
  }
  frameNumber.setMaximum(images.count()-1);
  slider.setMaximum(images.count()-1);
  first();
}

void Tracker::seek(int frame) {
  if(frame==current) return;
  if(frame<0 || frame>=images.count()) {
    stop();
    return;
  }
  QImage image(images[frame]);
  if(!pixmap) {
    pixmap = scene.addPixmap(QPixmap::fromImage(image));
    view.fitInView(pixmap);
  } else pixmap->setPixmap(QPixmap::fromImage(image));
  if(!pyramids[frame]) {
    profile("Mipmap");
    pyramids.insert(frame, klt->MakeImagePyramid(
          image.constBits(),image.width(),image.height()));
  }
  libmv::Features features;
  if(frame==current+1) {
    profile("Track");
    features = klt->Track(pyramids[frame-1],pyramids[frame]);
    int i=0; foreach(libmv::KLTPointFeature feature,features) {
      QGraphicsPathItem* track = tracks[i++];
      if(feature.trackness==0) continue;
      QPainterPath path = track->path();
      path.lineTo(QPointF(feature.x,feature.y));
      track->setPath(path);
    }
  } else {
    profile("Detect");
    features = klt->Detect(pyramids[frame]);
    foreach(QGraphicsPathItem* track, tracks) delete track;
    tracks.resize(features.size());
    int i=0; foreach(libmv::KLTPointFeature feature,features) {
      QPainterPath path;
      path.addEllipse(QPointF(feature.x,feature.y),4,4);
      path.moveTo(QPointF(feature.x,feature.y));
      tracks[i++]=scene.addPath(path);
    }
  }
  qDebug()<<features.size()<<"features";
  current=frame;
  slider.setValue(frame);
  frameNumber.setValue(frame);
}

void Tracker::first() {
  seek(0);
}
void Tracker::previous() {
  seek(current-1);
}
void Tracker::next() {
  seek(current+1);
}
void Tracker::last() {
  seek(images.count()-1);
}

void Tracker::start() {
  playAction->setChecked(true);
  playTimer.start(100);
}
void Tracker::stop() {
  playAction->setChecked(false);
  playTimer.stop();
}
void Tracker::togglePlay(bool play) {
  if(play) {
    start();
  } else {
    stop();
  }
}

void Tracker::resizeEvent(QResizeEvent *) { view.fitInView(pixmap); }
