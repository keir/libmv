// Copyright 2011 libmv authors
// Initial revision by Matthias Fauconneau.
#include "tracker.h"
#include "klt.h"
#include <QFileDialog>
#include <QDesktopWidget>
#include <QDebug>

int main(int argc, char *argv[]) {
  QApplication app(argc, argv);
  Tracker window;
  window.show();
  return app.exec();
}

Tracker::Tracker() : current(-2), klt(libmv::KLT::CreateContext()) {
  setWindowTitle("Qt Tracker");
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
  view.setScaledContents(true);
  view.setMaximumSize(qApp->desktop()->availableGeometry().size());
  setCentralWidget(&view);
  connect(&playTimer, SIGNAL(timeout()), SLOT(next()));
  QStringList args = qApp->arguments();
  args.removeFirst();
  if(args.isEmpty()) {
    view.setText("No image sequence selected");
  } else {
    open(args);
  }
}

void Tracker::open() {
  open(QFileDialog::getOpenFileNames(this, tr("Select Images"), "",
                                     "Images (*.png *.jpg);;All Files (*.*)"));
}
void Tracker::open(QStringList paths) {
  if(paths.isEmpty()) return;
  images.clear();
  pyramids.clear();
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
  pyramids.resize(images.count());
  first();
}

void Tracker::seek(int frame) {
  if(frame==current) return;
  if(frame<0 || frame>=images.count()) {
    stop();
    return;
  }
  slider.setValue(frame);
  frameNumber.setValue(frame);
  QImage image(images[frame]);
  view.setPixmap(QPixmap::fromImage(image));
  if(!pyramids[frame]) {
    pyramids[frame] = klt->MakeImagePyramid(
          image.constBits(),image.width(),image.height());
  }
  if(frame==current+1) {
    klt->Track(frame,pyramids[frame-1],pyramids[frame]);
  } else {
    klt->Detect(pyramids[frame]);
  }
  current=frame;
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
