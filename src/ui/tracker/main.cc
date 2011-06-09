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

#include "main.h"
#include "tracker.h"

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

class Clip : public QObject {
 public:
  Clip(QObject* parent=0) : QObject(parent) {}
  void Open(QString path) {
    cache_.setMaxCost(128 * 1024 * 1024);
    if (path.endsWith(".avi")) {
      // TODO(MatthiasF): load videos using ffmpeg
      return;
    }

    image_filenames_.clear();
    foreach (QString file, QDir(path).entryList(QStringList("*.jpg") << "*.png",
                                                QDir::Files, QDir::Name)) {
      image_filenames_ << QDir(path).filePath(file);
    }
  }

  QImage Frame(int frame) {
    QImage* image = cache_[frame];
    if (!image) {
      image = new QImage(image_filenames_[frame]);
      cache_.insert(frame, image, image->byteCount());
    }
    return *image;
  }

  int size() const { return image_filenames_.size(); }

 private:
  QList<QString> image_filenames_;
  QCache<int, QImage> cache_;
};

class View : public QGraphicsView {
public:
  View(QGraphicsScene *scene) {
    setScene(scene);
    setRenderHints(QPainter::Antialiasing|QPainter::SmoothPixmapTransform);
    setFrameShape(QFrame::NoFrame);
    setDragMode(QGraphicsView::ScrollHandDrag);
    setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  }
};

MainWindow::MainWindow()
  : clip_(new Clip(this)),
    tracker_(new Tracker(this)),
    view_(new View(tracker_)),
    zoom_view_(new View(tracker_)),
    current_frame_(-1) {
  connect(tracker_, SIGNAL(trackChanged(QGraphicsItem*)), SLOT(viewTrack(QGraphicsItem*)));

  setWindowTitle("LibMV Simple Tracker");
  setMaximumSize(qApp->desktop()->availableGeometry().size());

  QDockWidget* detail_dock = new QDockWidget("Selected Track Details");
  detail_dock->setObjectName("detailDock");
  addDockWidget(Qt::TopDockWidgetArea, detail_dock);
  const int kZoomFactor = 2;
  zoom_view_->setMinimumSize(kZoomFactor * TrackItem::kSearchWindowSize,
                             kZoomFactor * TrackItem::kSearchWindowSize);
  detail_dock->setWidget(zoom_view_);

  // Create the toolbar.
  QToolBar* toolbar = addToolBar("Main Toolbar");
  toolbar->setObjectName("mainToolbar");

  toolbar->addAction(QIcon(":/open"), "Open a new sequence...",
                     this, SLOT(open()));
  QToolButton* delete_button = new QToolButton;
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
  toolbar->addWidget(delete_button);
  track_action_ = toolbar->addAction(QIcon(":/record"), "Track selected markers");
  track_action_->setCheckable(true);
  connect(track_action_, SIGNAL(triggered(bool)), SLOT(toggleTracking(bool)));

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

  setCentralWidget(view_);

  restoreGeometry(QSettings().value("geometry").toByteArray());
  restoreState(QSettings().value("windowState").toByteArray());

  QStringList args = qApp->arguments();
  args.removeFirst();
  if (args.isEmpty()) {
    open();
  } else {
    open(args.first());
  }
}
MainWindow::~MainWindow() {
  QSettings().setValue("geometry", saveGeometry());
  QSettings().setValue("windowState", saveState());
  QFile tracks(path_+"/tracks");
  if (tracks.open(QFile::WriteOnly | QIODevice::Truncate)) {
    tracks.write(tracker_->Save());
  }
}

void MainWindow::open() {
  open(QFileDialog::getExistingDirectory(this, "Select sequence folder"));
}

void MainWindow::open(QString path) {
  if (path.isNull() || !QDir(path).exists()) {
    open();
    return;
  }
  tracker_->clear();
  pixmap_ = 0;
  path_ = path;
  clip_->Open(path);
  if (clip_->size() == 0) {
    open();
    return;
  }
  QFile tracks(path + "/tracks");
  if (tracks.open(QFile::ReadOnly)) {
    tracker_->Load(tracks.readAll());
  }
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
    view_->setMinimumSize(image.size()/2);
    view_->fitInView(pixmap_, Qt::KeepAspectRatio);
  } else {
    pixmap_->setPixmap(QPixmap::fromImage(image));
  }

  zoom_view_->clearFocus();
  slider_.setValue(current_frame_);
  spinbox_.setValue(current_frame_);
  tracker_->SetFrame(current_frame_, image, track_action_->isChecked());
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

void MainWindow::viewTrack(QGraphicsItem* item) {
  if (!zoom_view_->hasFocus()) {
    zoom_view_->fitInView(item, Qt::KeepAspectRatio);
  }
}

void MainWindow::resizeEvent(QResizeEvent *) {
  view_->fitInView(pixmap_, Qt::KeepAspectRatio);
}

int main(int argc, char *argv[]) {
  libmv::Init("", &argc, &argv);
  QApplication app(argc, argv);
  MainWindow window;
  window.show();
  return app.exec();
}

