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
#include <QSettings>
#include <QCache>
#include <QDebug>
#include <QTime>

#include "libmv/tools/tool.h"

class Clip {
 public:
  void Open(QStringList paths) {
    cache_.setMaxCost(128 * 1024 * 1024);
    if (paths.isEmpty()) {
      return;
    }
    if (paths.size() == 1 && paths[0].endsWith(".avi")) {
      // TODO(MatthiasF): load videos using ffmpeg"
      return;
    }

    image_filenames_.clear();
    foreach (QString path, paths) {
      if (QFileInfo(path).isFile()) {
        image_filenames_ << path;
      } else {
        foreach (QString file,
                 QDir(path).entryList(QStringList("*.jpg") << "*.png",
                                      QDir::Files,
                                      QDir::Name)) {
          image_filenames_ << QDir(path).filePath(file);
        }
      }
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
  : clip_(new Clip),
    tracker_(new Tracker()),
    view_(new View(tracker_.data())),
    zoom_view_(new View(tracker_.data())),
    current_frame_(-1) {
  connect(tracker_.data(), SIGNAL(trackChanged(TrackItem*)), SLOT(viewTrack(TrackItem*)));

  setWindowTitle("LibMV Simple Tracker");
  setMaximumSize(qApp->desktop()->availableGeometry().size());
  connect(&play_timer_, SIGNAL(timeout()), SLOT(next()));

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
  toolbar->addAction(QIcon::fromTheme("document-open"), "Open...",
                     this, SLOT(open()));
  toolbar->addAction(QIcon::fromTheme("media-skip-backward"), "First Frame",
                     this, SLOT(first()));
  toolbar->addAction(QIcon::fromTheme("media-seek-backward"),"Previous Frame",
                     this, SLOT(previous()))->setShortcut(QKeySequence("Left"));
  toolbar->addWidget(&frame_number_);
  connect(&frame_number_, SIGNAL(valueChanged(int)), SLOT(seek(int)));
  play_action_ = toolbar->addAction(QIcon::fromTheme("media-playback-start"),
                                  QKeySequence("Play"));
  play_action_->setCheckable(true);
  play_action_->setShortcut(QKeySequence("Space"));
  connect(play_action_, SIGNAL(triggered(bool)), SLOT(togglePlay(bool)));
  toolbar->addWidget(&slider_);
  slider_.setOrientation(Qt::Horizontal);
  connect(&slider_, SIGNAL(valueChanged(int)), SLOT(seek(int)));
  toolbar->addAction(QIcon::fromTheme("media-seek-forward"), "Next Frame",
                     this, SLOT(next()))
      ->setShortcut(QKeySequence("Right"));
  toolbar->addAction(QIcon::fromTheme("media-skip-forward"), "Last Frame",
                     this, SLOT(last()));

  setCentralWidget(view_);

  restoreGeometry(QSettings().value("geometry").toByteArray());
  restoreState(QSettings().value("windowState").toByteArray());

  QStringList args = qApp->arguments();
  args.removeFirst();
  if (args.isEmpty()) {
    open();
  } else {
    open(args);
  }
}
MainWindow::~MainWindow() {
  QSettings().setValue("geometry", saveGeometry());
  QSettings().setValue("windowState", saveState());
}

void MainWindow::open() {
  open(QFileDialog::getOpenFileNames(
      this,
      tr("Select Images"),
      "",
      "Images (*.png *.jpg);;All Files (*.*)"));
}

void MainWindow::open(QStringList paths) {
  tracker_->clear();
  pixmap_ = 0;
  clip_->Open(paths);
  frame_number_.setMaximum(clip_->size() - 1);
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
  frame_number_.setValue(current_frame_);
  tracker_->SetFrame(current_frame_, image);
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

void MainWindow::start() {
  play_action_->setChecked(true);
  play_timer_.start(50);
}

void MainWindow::stop() {
  play_action_->setChecked(false);
  play_timer_.stop();
}

void MainWindow::togglePlay(bool play) {
  if (play) {
    start();
  } else {
    stop();
  }
}

void MainWindow::viewTrack(TrackItem* item) {
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

