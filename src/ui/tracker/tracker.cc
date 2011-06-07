// Copyright 2011 libmv authors
// Initial revision by Matthias Fauconneau.
//
#include "tracker.h"
#include "libmv/simple_pipeline/tracks.h"
#include "libmv/logging/logging.h"
#include "libmv/tools/tool.h"
#include "libmv/image/image.h"
#include "libmv/tracking/klt_region_tracker.h"
#include "libmv/tracking/pyramid_region_tracker.h"
#include "libmv/tracking/retrack_region_tracker.h"

#include <QDebug>
#include <QDesktopWidget>
#include <QImageReader>
#include <QFileDialog>
#include <QGraphicsPixmapItem>
#include <QGraphicsSceneMouseEvent>
#include <QDockWidget>
#include <QString>
#include <QStyle>
#include <QTime>
#include <QCache>
#include <QSettings>

int main(int argc, char *argv[]) {
  libmv::Init("", &argc, &argv);
  QApplication app(argc, argv);
  Tracker window;
  window.show();
  return app.exec();
}

class TrackItem : public QGraphicsRectItem {
public:
  static const int size = 64;
  TrackItem() : QGraphicsRectItem(-size/2,-size/2,size/2,size/2) {
    setPen(QPen(QBrush(Qt::green), 3));
    setFlags(QGraphicsItem::ItemIsSelectable|QGraphicsItem::ItemIsMovable);
  }
  bool hidden;
};

class TrackerScene : public QGraphicsScene {
  // Only add Q_OBJECT when there are slots.
 public:
  TrackerScene(libmv::Tracks *tracks) : tracks_(tracks) {}
  ~TrackerScene() {
    foreach(TrackItem* item, trackItems_) {
      removeItem(item);
      delete item;
    }
  }

  void SetFrame(int frame) {
    LG << "Setting frame to " << frame;

    std::vector<libmv::Marker> markers;
    tracks_->TracksInImage(frame, &markers);
    LG << "Got " << markers.size() << " markers.";

    // TODO(MatthiasF): would be much simpler if libmv API returned all tracks
    // and flag hidden tracks
    foreach(TrackItem* item, trackItems_) item->hidden=true;
    foreach (const libmv::Marker &marker, markers) {
      TrackItem* item = trackItems_[marker.track];
      item->setPos(marker.x,marker.y);
      item->hidden=false;
    }
    foreach(TrackItem* item, trackItems_) {
      if(item->hidden) {
        item->hide();
      } else {
        item->show();
      }
    }
    current_frame_ = frame;
  }

 protected:
  void mousePressEvent(QGraphicsSceneMouseEvent *mouseEvent) {
    QGraphicsScene::mousePressEvent(mouseEvent);
    if (mouseEvent->isAccepted()) return;
    LG << "Got mouse click at " << mouseEvent->scenePos().x()
       << ", " << mouseEvent->scenePos().y();

    if (mouseEvent->button() != Qt::LeftButton) {
      return;
    }

    int new_track = tracks_->MaxTrack() + 1;
    TrackItem* item = new TrackItem();
    trackItems_ << item;
    addItem(item);
    LG << "Inserting new marker for frame " << current_frame_
       << " track " << new_track
       << " with x=" << mouseEvent->scenePos().x()
       << ", y=" << mouseEvent->scenePos().y();

    tracks_->Insert(current_frame_, new_track,
                    mouseEvent->scenePos().x(),
                    mouseEvent->scenePos().y());
    SetFrame(current_frame_);
  }

 private:
  QVector<TrackItem*> trackItems_;
  libmv::Tracks *tracks_;
  int current_frame_;
};

class Clip {
 public:
  void Open(QStringList paths) {
    cache_.setMaxCost(256 * 1024 * 1024);
    if (paths.isEmpty()) {
      return;
    }
    if (paths.size() == 1 && paths[0].endsWith(".avi")) {
      LG << "TODO(MatthiasF): load videos using ffmpeg";
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
      LG << "Caching frame using " << image->byteCount() << " bytes;"
         << " cache has size " << cache_.count();
      cache_.insert(frame, image, image->byteCount());
    }
    return *image;
  }

  int size() const { return image_filenames_.size(); }

 private:
  QList<QString> image_filenames_;
  QCache<int, QImage> cache_;
};

// Copy the region starting at *x0, *y0 with width w, h into region. If the
// region asked for is outside the image border, clipping is done and the
// returned region is smaller than requested. At return, *x0 and *y0 contain
// the top left pixel of *region in the original image (which may not match
// what was passed in). Returns true if any values were copied.
bool CopyRegionFromQImage(QImage image,
                          int w, int h,
                          int *x0, int *y0,
                          libmv::FloatImage *region) {
  const unsigned char *data = image.bits();
  int width = image.width();
  int height = image.height();

  // Clip the region on the upper right.
  if (*x0 < 0) {
    w += *x0;
    *x0 = 0;
  }
  if (*y0 < 0) {
    h += *y0;
    *y0 = 0;
  }

  // Clip the region on the lower left.
  w = std::min(w, std::max(width - *x0, 0));
  h = std::min(h, std::max(height - *y0, 0));

  // The region is entirely outside the given image.
  if (w <= 0 || h <= 0) {
    return false;
  }

  // Now that clipping is done, do the blit.
  region->resize(h, w);
  for (int y = *y0; y < *y0 + h; ++y) {
    for (int x = *x0; x < *x0 + w; ++x) {
      // This assumes BGR row-major ordering for the QImage's raw bytes.
      const unsigned char *pixel = data + (y * width + x) * 4;
      (*region)(y - *y0, x - *x0, 0) = (0.2126 * pixel[2] +
                                        0.7152 * pixel[1] +
                                        0.0722 * pixel[0]) / 255;
    }
  }
  return true;
}

libmv::RegionTracker *CreateRegionTracker() {
  libmv::KltRegionTracker *klt_region_tracker = new libmv::KltRegionTracker;
  klt_region_tracker->half_window_size = 5;
  klt_region_tracker->max_iterations = 200;
  libmv::PyramidRegionTracker *pyramid_region_tracker =
      new libmv::PyramidRegionTracker(klt_region_tracker, 3);
  return new libmv::RetrackRegionTracker(pyramid_region_tracker, 0.2);
}

class View : public QGraphicsView {
public:
  View(QGraphicsScene* scene) {
    setScene(scene);
    setRenderHints(QPainter::Antialiasing|QPainter::SmoothPixmapTransform);
    setFrameShape(QFrame::NoFrame);
    setDragMode(QGraphicsView::ScrollHandDrag);
    setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  }
};

Tracker::Tracker()
  : clip_(new Clip),
    tracks_(new libmv::Tracks()),
    region_tracker_(CreateRegionTracker()),
    scene(new TrackerScene(tracks_.data())),
    view(new View(scene.data())),
    zoomView(new View(scene.data())),
    current_(-1) {
  setWindowTitle("LibMV Simple Tracker");
  setMaximumSize(qApp->desktop()->availableGeometry().size());
  connect(&playTimer, SIGNAL(timeout()), SLOT(next()));

  detailDock = new QDockWidget("Selected Track Details");
  detailDock->setObjectName("detailDock");
  addDockWidget(Qt::TopDockWidgetArea,detailDock);
  zoomView->setMinimumSize(2*TrackItem::size,2*TrackItem::size);
  detailDock->setWidget(zoomView);
  detailDock->hide();

  // Create the toolbar.
  QToolBar* toolbar = addToolBar("Main Toolbar");
  toolbar->setObjectName("mainToolbar");
  toolbar->addAction(QIcon::fromTheme("document-open"), "Open...",
                     this, SLOT(open()));
  toolbar->addAction(QIcon::fromTheme("media-skip-backward"), "First Frame",
                     this, SLOT(first()));
  toolbar->addAction(QIcon::fromTheme("media-seek-backward"),"Previous Frame",
                     this, SLOT(previous()))->setShortcut(QKeySequence("Left"));
  toolbar->addWidget(&frameNumber);
  connect(&frameNumber, SIGNAL(valueChanged(int)), SLOT(seek(int)));
  playAction = toolbar->addAction(QIcon::fromTheme("media-playback-start"),
                                  QKeySequence("Play"));
  playAction->setCheckable(true);
  playAction->setShortcut(QKeySequence("Space"));
  connect(playAction, SIGNAL(triggered(bool)), SLOT(togglePlay(bool)));
  toolbar->addWidget(&slider);
  slider.setOrientation(Qt::Horizontal);
  slider.style()->styleHint(QStyle::SH_Slider_AbsoluteSetButtons);
  connect(&slider, SIGNAL(sliderMoved(int)), SLOT(seek(int)));
  toolbar->addAction(QIcon::fromTheme("media-seek-forward"), "Next Frame",
                     this, SLOT(next()))
      ->setShortcut(QKeySequence("Right"));
  toolbar->addAction(QIcon::fromTheme("media-skip-forward"), "Last Frame",
                     this, SLOT(last()));


  setCentralWidget(view);
  connect(scene.data(),SIGNAL(selectionChanged()),SLOT(selectMarker()));

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
Tracker::~Tracker() {
  QSettings().setValue("geometry", saveGeometry());
  QSettings().setValue("windowState", saveState());
}

void Tracker::open() {
  open(QFileDialog::getOpenFileNames(
      this,
      tr("Select Images"),
      "",
      "Images (*.png *.jpg);;All Files (*.*)"));
}

void Tracker::open(QStringList paths) {
  scene->clear();
  pixmap = 0;
  clip_->Open(paths);
  frameNumber.setMaximum(clip_->size() - 1);
  slider.setMaximum(clip_->size() - 1);
  first();
}

void Tracker::seek(int frame) {
  // Bail out if there's nothing to do.
  if (frame == current_) {
    return;
  }
  if (frame < 0 || frame >= clip_->size()) {
    stop();
    return;
  }

  // Get and display the image.
  QImage image = clip_->Frame(frame);
  if (!pixmap) {
    pixmap = scene->addPixmap(QPixmap::fromImage(image));
    view->setMinimumSize(image.size()/2);
    view->fitInView(pixmap,Qt::KeepAspectRatio);
  } else {
    pixmap->setPixmap(QPixmap::fromImage(image));
  }

  // If the shift is between consecutive frames, track the active trackers
  // from the previous frame into this one.
  if(frame == current_+1) {
    std::vector<libmv::Marker> marker_in_previous_frame;
    tracks_->TracksInImage(current_, &marker_in_previous_frame);
    foreach (const libmv::Marker &marker, marker_in_previous_frame) {
      //if (active_tracks_.find(marker.track) == active_tracks_.end()) {
      //  continue
      //}
      // TODO(keir): For now this uses a fixed 32x32 region. What's needed is
      // an extension to use custom sized boxes around the tracked region.
      int size = 64;
      int half_size = size / 2;

      // [xy][01] is the upper right box corner.
      int x0 = marker.x - half_size;
      int y0 = marker.y - half_size;
      libmv::FloatImage old_patch;
      if (!CopyRegionFromQImage(clip_->Frame(current_), size, size,
                                &x0, &y0,
                                &old_patch)) {
        continue;
      }

      int x1 = marker.x - half_size;
      int y1 = marker.y - half_size;
      libmv::FloatImage new_patch;
      if (!CopyRegionFromQImage(clip_->Frame(frame), size, size,
                                &x1, &y1,
                                &new_patch)) {
        continue;
      }

      double xx0 = marker.x - x0;
      double yy0 = marker.y - y0;
      double xx1 = marker.x - x1;
      double yy1 = marker.y - y1;
      if (region_tracker_->Track(old_patch, new_patch,
                                 xx0, yy0,
                                 &xx1, &yy1)) {
        tracks_->Insert(frame, marker.track, x1 + xx1, y1 + yy1);
        LG << "Tracked (" << xx0 << ", " << yy0 << ") to ("
           << xx1 << ", " << yy1 << ").";
      }
    }
  }

  current_ = frame;
  slider.setValue(frame);
  frameNumber.setValue(frame);

  // Update the view.
  scene->SetFrame(frame);
  if(!scene->selectedItems().isEmpty()) {
    zoomView->fitInView(scene->selectedItems().first(),Qt::KeepAspectRatio);
  }
}

void Tracker::first() {
  seek(0);
}
void Tracker::previous() {
  seek(current_ - 1);
}
void Tracker::next() {
  seek(current_ + 1);
}
void Tracker::last() {
  seek(clip_->size() - 1);
}

void Tracker::start() {
  playAction->setChecked(true);
  playTimer.start(50);
}
void Tracker::stop() {
  playAction->setChecked(false);
  playTimer.stop();
}
void Tracker::togglePlay(bool play) {
  if (play) {
    start();
  } else {
    stop();
  }
}

void Tracker::selectMarker() {
  if(scene->selectedItems().isEmpty()) detailDock->hide();
  else {
    detailDock->show();
    zoomView->fitInView(scene->selectedItems().first(),Qt::KeepAspectRatio);
  }
}

void Tracker::resizeEvent(QResizeEvent *) {
  view->fitInView(pixmap,Qt::KeepAspectRatio);
}
