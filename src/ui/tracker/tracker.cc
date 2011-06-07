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
#include <QString>
#include <QStyle>
#include <QTime>
#include <QCache>

using libmv::FloatImage;
using libmv::Marker;
using std::vector;
using std::pair;
using std::make_pair;

int main(int argc, char *argv[]) {
  libmv::Init("", &argc, &argv);
  QApplication app(argc, argv);
  Tracker window;
  window.show();
  return app.exec();
}

template<typename Container, typename Key>
bool ContainsKey(const Container &container, const Key &key) {
  return container.find(key) != container.end();
}

class TrackerScene : public QGraphicsScene {
  // Only add Q_OBJECT when there are slots.
 public:
  TrackerScene(libmv::Tracks *tracks) : tracks_(tracks) {}
  ~TrackerScene() {
    foreach(QGraphicsItem* item, markers_) {
      removeItem(item);
      delete item;
    }
  }

  void SetFrame(int frame) {
    LG << "Setting frame to " << frame;

    foreach (QGraphicsItem* item, markers_) {
      item->hide();
    }

    vector<Marker> markers;
    tracks_->TracksInImage(frame, &markers);
    LG << "Got " << markers.size() << " markers.";

    const int half_size = 32;
    foreach (const Marker &marker, markers) {
      pair<int, int> key = make_pair(marker.image, marker.track);
      QGraphicsItem *item = NULL;
      if (!ContainsKey(markers_, key)) {
        item = markers_[key] = addRect(marker.x - half_size,
                                       marker.y - half_size,
                                       2 * half_size,
                                       2 * half_size,
                                       QPen(QBrush(Qt::green), 3));
      } else {
        item = markers_[key];
      }
      item->show();
    }
    current_frame_ = frame;
  }

 protected:
  void mousePressEvent(QGraphicsSceneMouseEvent *mouseEvent) {
    LG << "Got mouse click at " << mouseEvent->scenePos().x()
       << ", " << mouseEvent->scenePos().y();

    if (mouseEvent->button() != Qt::LeftButton) {
      return;
    }

    int new_track = tracks_->MaxTrack() + 1;
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
  QMap<pair<int, int>, QGraphicsItem *> markers_;
  libmv::Tracks *tracks_;
  int current_frame_;
};

class Clip {
 public:
  Clip() : image_reader_(NULL) {}
  Clip(QStringList paths) : image_reader_(NULL) {
    Open(paths);
  }

  void Open(QStringList paths) {
    cache_.setMaxCost(512 * 1024 * 1024);
    if (paths.isEmpty()) {
      return;
    }
    if (paths.size() == 1 && paths[0].endsWith(".avi")) {
      // TODO(keir): Video loading is broken; it segfaults.
      LG << "Loading video file: " << paths[0].toStdString();
      image_reader_.reset(new QImageReader(paths[0]));
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
    if (cache_.contains(frame)) {
      LG << "Got cached frame " << frame;
      return *cache_.take(frame);
    }
    LG << "Not cached; reading frame " << frame;
    // Not in the cache.
    QImage *image = new QImage;
    if (image_reader_.get()) {
      image_reader_->jumpToImage(frame);
      *image = image_reader_->read();
      if (image->isNull()) {
        LG << "Error reading frame " << frame;
        // Return the null image anyway.
      }
    } else {
      *image = QImage(image_filenames_[frame]);
    }
    LG << "Caching frame using " << image->byteCount() << " bytes;"
       << " cache has size " << cache_.count();
    cache_.insert(frame, image, image->byteCount());
    return *image;
  }

  int size() const { return image_filenames_.size(); }

 private:
  QList<QString> image_filenames_;
  libmv::scoped_ptr<QImageReader> image_reader_;
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

Tracker::Tracker()
    : current_(-1),
      clip_(new Clip),
      tracks_(new libmv::Tracks),
      region_tracker_(CreateRegionTracker()) {
  setWindowTitle("LibMV Simple Tracker");
  setMaximumSize(qApp->desktop()->availableGeometry().size());

  // Create the toolbar.
  QToolBar* tool_bar = addToolBar("Main Toolbar");
  tool_bar->setObjectName("mainToolbar");
  tool_bar->addAction(QIcon::fromTheme("document-open"), "Open...",
                     this, SLOT(open()));
  tool_bar->addAction(QIcon::fromTheme("media-skip-backward"), "First Frame",
                     this, SLOT(first()));
  tool_bar->addAction(QIcon::fromTheme("media-seek-backward"),"Previous Frame",
                     this, SLOT(previous()))->setShortcut(QKeySequence("Left"));
  tool_bar->addWidget(&frameNumber);
  connect(&frameNumber, SIGNAL(valueChanged(int)), SLOT(seek(int)));
  playAction = tool_bar->addAction(QIcon::fromTheme("media-playback-start"),
                                  QKeySequence("Play"));
  playAction->setCheckable(true);
  playAction->setShortcut(QKeySequence("Space"));
  connect(playAction, SIGNAL(triggered(bool)), SLOT(togglePlay(bool)));
  tool_bar->addWidget(&slider);
  slider.setOrientation(Qt::Horizontal);
  slider.style()->styleHint(QStyle::SH_Slider_AbsoluteSetButtons);
  connect(&slider, SIGNAL(sliderMoved(int)), SLOT(seek(int)));
  tool_bar->addAction(QIcon::fromTheme("media-seek-forward"), "Next Frame",
                     this, SLOT(next()))
          ->setShortcut(QKeySequence("Right"));
  tool_bar->addAction(QIcon::fromTheme("media-skip-forward"), "Last Frame",
                     this, SLOT(last()));

  // Set up the scene.
  scene = new TrackerScene(tracks_);
  view.setScene(scene);
  view.setRenderHints(QPainter::Antialiasing|QPainter::SmoothPixmapTransform);
  view.setFrameShape(QFrame::NoFrame);
  view.setDragMode(QGraphicsView::ScrollHandDrag);
  view.setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  view.setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  setCentralWidget(&view);
  connect(&playTimer, SIGNAL(timeout()), SLOT(next()));

  QStringList args = qApp->arguments();
  args.removeFirst();
  if (args.isEmpty()) {
    open(); 
  } else {
    open(args);
  }
}

Tracker::~Tracker() {
  delete tracks_;
  delete scene;
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
    view.fitInView(pixmap,Qt::KeepAspectRatio);
  } else {
    pixmap->setPixmap(QPixmap::fromImage(image));
  }

  // If the shift is between consecutive frames, track the active trackers
  // from the previous frame into this one.
  vector<Marker> marker_in_previous_frame;
  tracks_->TracksInImage(current_, &marker_in_previous_frame);
  foreach (const Marker &marker, marker_in_previous_frame) {
    //if (active_tracks_.find(marker.track) == active_tracks_.end()) {
    //  continue
    //}
    int size = 64;
    int half_size = size / 2;

    // [xy][01] is the upper right box corner.
    int x0 = marker.x - half_size;
    int y0 = marker.y - half_size;
    FloatImage old_patch;
    if (!CopyRegionFromQImage(current_image_, size, size,
                              &x0, &y0,
                              &old_patch)) {
      // TODO(keir): Must handle this case! Currently no marker delete.
      LG << "Copy from old frame failed.";
      continue;
    }

    int x1 = marker.x - half_size;
    int y1 = marker.y - half_size;
    FloatImage new_patch;
    if (!CopyRegionFromQImage(image, size, size,
                              &x1, &y1,
                              &new_patch)) {
      // TODO(keir): Must handle this case! Currently no marker delete.
      LG << "Copy from new frame failed.";
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
    } else {
      // TODO(keir): Must handle this case! Currently no marker delete.
    }
  }

  current_ = frame;
  current_image_ = image;
  slider.setValue(frame);
  frameNumber.setValue(frame);

  // Now updated the view.
  scene->SetFrame(frame);
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
  playTimer.start(100);
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

void Tracker::resizeEvent(QResizeEvent *) {
  view.fitInView(pixmap,Qt::KeepAspectRatio);
}
