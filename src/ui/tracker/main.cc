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

#include "ui/tracker/main.h"
#include "ui/tracker/tracker.h"
#include "ui/tracker/view.h"

#include <QDesktopWidget>
#include <QGraphicsPixmapItem>
#include <QGraphicsView>
#include <QFileDialog>
#include <QDockWidget>
#include <QToolButton>
#include <QSettings>
#include <QCache>
#include <QDebug>
#include <QMenu>
#include <QTime>

#include "libmv/tools/tool.h"

void Clip::Open(QString path) {
  cache_.setMaxCost(64 * 1024 * 1024);
  if (path.endsWith(".avi")) {
    // TODO(MatthiasF): load videos using ffmpeg
    return;
  }

  clear();
  foreach (QString file, QDir(path).entryList(QStringList("*.jpg") << "*.png",
                                              QDir::Files, QDir::Name)) {
    append( QDir(path).filePath(file) );
  }
}

QImage Clip::Frame(int frame) {
  QImage* image = cache_[frame];
  if (!image) {
    image = new QImage(value(frame));
    cache_.insert(frame, image, image->byteCount());
  }
  return *image;
}

TrackerView::TrackerView(QGraphicsScene *scene,QWidget *parent)
  : QGraphicsView(scene,parent) {
  setRenderHints(QPainter::Antialiasing|QPainter::SmoothPixmapTransform);
  setDragMode(QGraphicsView::ScrollHandDrag);
  setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
}

MainWindow::MainWindow()
  : clip_(new Clip(this)),
    tracker_(new Tracker(this)),
    pixmap_(0), overlay_(0), current_frame_(-1),
    image_view_(new TrackerView(tracker_)),
    zoom_view_(new TrackerView(tracker_)),
    scene_view_(new View()) {
  setWindowTitle("Tracker");
  setMaximumSize(qApp->desktop()->availableGeometry().size());

  QToolBar* toolbar = addToolBar("Main Toolbar");
  toolbar->setObjectName("mainToolbar");

  toolbar->addAction(QIcon(":/open"), "Open a new sequence...",
                     this, SLOT(open()));

  setCentralWidget(image_view_);
  //image_view_->setVisible(QSettings().value("showImage").toBool());
  QAction* image_action_ = toolbar->addAction(QIcon(":/view-image"),"Image View");
  image_action_->setCheckable(true);
  image_action_->setChecked(image_view_->isVisible());
  connect(image_action_,SIGNAL(triggered(bool)),image_view_,SLOT(setVisible(bool)));
  connect(image_view_, SIGNAL(geometryChanged()), SLOT(fitImage()));

  QDockWidget* zoom_dock = new QDockWidget("Zoom View");
  zoom_dock->setObjectName("zoomDock");
  addDockWidget(Qt::TopDockWidgetArea, zoom_dock);
  const int kZoomFactor = 2;
  zoom_view_->setMinimumSize(kZoomFactor * TrackItem::kSearchWindowSize,
                             kZoomFactor * TrackItem::kSearchWindowSize);
  zoom_view_->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Fixed);
  zoom_dock->setWidget(zoom_view_);
  zoom_dock->toggleViewAction()->setIcon(QIcon(":/view-zoom"));
  toolbar->addAction(zoom_dock->toggleViewAction());
  connect(tracker_, SIGNAL(trackChanged(QGraphicsItem*)), SLOT(fitZoom(QGraphicsItem*)));

  QDockWidget* scene_dock = new QDockWidget("Scene View");
  scene_dock->setObjectName("sceneDock");
  addDockWidget(Qt::BottomDockWidgetArea, scene_dock);
  scene_dock->setWidget(scene_view_);
  scene_dock->toggleViewAction()->setIcon(QIcon(":/view-scene"));
  toolbar->addAction(scene_dock->toggleViewAction());
  connect(scene_view_, SIGNAL(imageChanged(int)), SLOT(seek(int)));
  connect(scene_view_, SIGNAL(objectChanged()), SLOT(updateOverlay()));

  toolbar->addSeparator();

  QToolButton* delete_button = new QToolButton();
  toolbar->addWidget(delete_button);
  QMenu* delete_popup = new QMenu(this);
  delete_popup->addAction(QIcon(":/delete"),
                          "Delete current marker",
                          tracker_, SLOT(deleteCurrentMarker()));
  QAction* delete_track = delete_popup->addAction(QIcon(":/delete-row"),
                                                "Delete current track",
                                                tracker_,
                                                SLOT(deleteCurrentTrack()));
  delete_button->setMenu(delete_popup);
  delete_button->setDefaultAction(delete_track);
  delete_button->setPopupMode(QToolButton::MenuButtonPopup);
  connect(delete_popup,SIGNAL(triggered(QAction*)),
          delete_button,SLOT(setDefaultAction(QAction*)));

  track_action_ = toolbar->addAction(QIcon(":/record"), "Track selected markers");
  track_action_->setCheckable(true);
  connect(track_action_, SIGNAL(triggered(bool)), SLOT(toggleTracking(bool)));
  connect(image_action_, SIGNAL(triggered(bool)), track_action_, SLOT(setVisible(bool)));
  track_action_->setVisible(image_view_->isVisible());

  QAction* add_action = toolbar->addAction(QIcon(":/add"), "Add object", scene_view_, SLOT(add()));
  connect(scene_dock->toggleViewAction(), SIGNAL(triggered(bool)),
          add_action, SLOT(setVisible(bool)));

  QAction* link_action = toolbar->addAction(QIcon(":/link"), "Link active object to selected bundles",
                                            scene_view_, SLOT(link()));
  connect(scene_dock->toggleViewAction(), SIGNAL(triggered(bool)),
          link_action, SLOT(setVisible(bool)));

  toolbar->addSeparator();

  toolbar->addAction(QIcon(":/skip-backward"), "Seek to first frame",
                     this, SLOT(first()));
  toolbar->addAction(QIcon(":/step-backward"),"Step to previous frame",
                     this, SLOT(previous()))->setShortcut(QKeySequence("Left"));
  backward_action_ = toolbar->addAction(QIcon(":/play-backward"),
                                        "Play sequence backwards");
  backward_action_->setCheckable(true);
  connect(backward_action_, SIGNAL(triggered(bool)), SLOT(toggleBackward(bool)));
  connect(&previous_timer_, SIGNAL(timeout()), SLOT(previous()));

  toolbar->addWidget(&spinbox_);
  connect(&spinbox_, SIGNAL(valueChanged(int)), SLOT(seek(int)));
  toolbar->addWidget(&slider_);
  slider_.setOrientation(Qt::Horizontal);
  connect(&slider_, SIGNAL(valueChanged(int)), SLOT(seek(int)));

  forward_action_ = toolbar->addAction(QIcon(":/play-forward"),
                                       "Play sequence forwards");
  forward_action_->setCheckable(true);
  connect(forward_action_, SIGNAL(triggered(bool)), SLOT(toggleForward(bool)));
  connect(&next_timer_, SIGNAL(timeout()), SLOT(next()));
  toolbar->addAction(QIcon(":/step-forward"), "Next Frame",this, SLOT(next()))
      ->setShortcut(QKeySequence("Right"));
  toolbar->addAction(QIcon(":/skip-forward"), "Last Frame",this, SLOT(last()));

  restoreGeometry(QSettings().value("geometry").toByteArray());
  restoreState(QSettings().value("windowState").toByteArray());
}
void MainWindow::Save(QString name,QByteArray data) {
  QFile file(QDir(path_).filePath(name));
  if (file.open(QFile::WriteOnly | QIODevice::Truncate)) {
    file.write(data);
  }
}
MainWindow::~MainWindow() {
  //QSettings().setValue("showImage", image_view_->isVisible());
  QSettings().setValue("geometry", saveGeometry());
  QSettings().setValue("windowState", saveState());
  if(clip_->isEmpty()) return;
  Save("tracks",tracker_->Save());
  Save("cameras",scene_view_->SaveCameras());
  Save("bundles",scene_view_->SaveBundles());
  Save("objects",scene_view_->SaveObjects());
}

QByteArray MainWindow::Load(QString name) {
  QFile file(QDir(path_).filePath(name));
  return file.open(QFile::ReadOnly) ? file.readAll() : QByteArray();
}

void MainWindow::open() {
  open(QFileDialog::getExistingDirectory(this, "Select sequence folder"));
}

void MainWindow::open(QString path) {
  if (path.isEmpty() || !QDir(path).exists()) return;
  clip_->Open(path);
  if(clip_->isEmpty()) return;
  pixmap_ = 0;
  overlay_ = 0;
  tracker_->clear();
  path_ = path;
  setWindowTitle(QString("Tracker - %1").arg(QDir(path).dirName()));
  tracker_->Load(Load("tracks"));
  scene_view_->LoadCameras(Load("cameras"));
  scene_view_->LoadBundles(Load("bundles"));
  scene_view_->LoadObjects(Load("objects"));
  scene_view_->upload();
  spinbox_.setMaximum(clip_->size() - 1);
  slider_.setMaximum(clip_->size() - 1);
  first();
}

void MainWindow::seek(int frame) {
  // Bail out if there's nothing to do.
  if (frame == current_frame_) {
    return;
  }
  if (frame < 0 || frame >= clip_->size()) {
    stop();
    return;
  }
  // Track only if the shift is between consecutive frames.
  if ( frame > current_frame_ + 1 || frame < current_frame_ - 1 ) {
    track_action_->setChecked(false);
  }
  current_frame_ = frame;

  // Get and display the image.
  QImage image = clip_->Frame(current_frame_);
  if (!pixmap_) {
    pixmap_ = tracker_->addPixmap(QPixmap::fromImage(image));
    overlay_ = tracker_->addPixmap(QPixmap());
    overlay_->setZValue(1);
    image_view_->setMinimumSize(image.size()/2);
    image_view_->setMaximumSize(image.size());
    image_view_->fitInView(pixmap_, Qt::KeepAspectRatio);
  } else {
    pixmap_->setPixmap(QPixmap::fromImage(image));
  }

  zoom_view_->clearFocus();
  slider_.setValue(current_frame_);
  spinbox_.setValue(current_frame_);
  tracker_->SetFrame(current_frame_, image, track_action_->isChecked());
  updateOverlay();
}

void MainWindow::toggleTracking(bool track) {
  stop();
  if (track) {
    backward_action_->setText("Track sequence backwards");
    forward_action_->setText("Track sequence forwards");
  } else {
    backward_action_->setText("Play sequence backwards");
    forward_action_->setText("Play sequence forwards");
  }
}

void MainWindow::toggleBackward(bool play) {
  if (play) {
    forward_action_->setChecked(false);
    next_timer_.stop();
    previous_timer_.start(0);
  } else {
    stop();
  }
}

void MainWindow::toggleForward(bool play) {
  if (play) {
    backward_action_->setChecked(false);
    previous_timer_.stop();
    next_timer_.start(0);
  } else {
    stop();
  }
}

void MainWindow::stop() {
  backward_action_->setChecked(false);
  previous_timer_.stop();
  forward_action_->setChecked(false);
  next_timer_.stop();
}

void MainWindow::first() {
  seek(0);
}

void MainWindow::previous() {
  seek(current_frame_ - 1);
}

void MainWindow::next() {
  seek(current_frame_ + 1);
}

void MainWindow::last() {
  seek(clip_->size() - 1);
}

void MainWindow::fitImage() {
  if(pixmap_) image_view_->fitInView(pixmap_, Qt::KeepAspectRatio);
  updateOverlay();
}

void MainWindow::fitZoom(QGraphicsItem* item) {
  if (!zoom_view_->hasFocus()) {
    zoom_view_->fitInView(item, Qt::KeepAspectRatio);
  }
}

void MainWindow::updateOverlay() {
  if(!image_view_->isVisible()) return;
  if(overlay_) {
    overlay_->setPixmap(
          scene_view_->renderCamera(pixmap_->pixmap().width(),pixmap_->pixmap().height(),
                                    current_frame_));
  }
}

int main(int argc, char *argv[]) {
  libmv::Init("", &argc, &argv);
  QApplication app(argc, argv);
  app.setOrganizationName("libmv");
  app.setApplicationName("tracker");
  MainWindow window;
  window.show();
  window.open(app.arguments().value(1));
  return app.exec();
}

