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

using std::vector;

const int kTrackerHalfWindowSize = 5;

class TrackItem : public QGraphicsRectItem {
public:
  static const int size = 64;
  TrackItem(int track)
      : QGraphicsRectItem(-size/2, -size/2, size, size), track(track) {
    setPen(QPen(QBrush(Qt::green), 3));
    setFlags(QGraphicsItem::ItemIsSelectable|QGraphicsItem::ItemIsMovable);
  }

  int track;
};

class TrackerScene : public QGraphicsScene {
  // Only add Q_OBJECT when there are slots.
 public:
  TrackerScene(libmv::Tracks *tracks) : tracks_(tracks), selected_track_(-1) {}
  virtual ~TrackerScene() {}

  void SetFrame(int frame) {
    if (frame == current_frame_) {
      LG << "Ignoring request to set frame to current frame.";
      return;
    }
    LG << "Setting frame to " << frame;

    vector<libmv::Marker> markers;
    tracks_->MarkersInImage(frame, &markers);
    LG << "Got " << markers.size() << " markers.";

    // Set the position of the tracks for this frame.
    QSet<int> tracks_in_new_frame;
    foreach (const libmv::Marker &marker, markers) {
      // Create a visible set to find the tracks that disappeared from the
      // previous frame.
      tracks_in_new_frame << marker.track;

      // Set the position of the marker in the new frame.
      TrackItem *&track_item = track_items_[marker.track];
      Q_ASSERT(track_item);
      track_item->setPos(marker.x, marker.y);
      track_item->show();
      if (selected_track_ == marker.track) {
        track_item->setSelected(true);
      }
    }

    // Hide any tracks that were visible in the last frame but not this one.
    foreach (int track, tracks_in_current_frame_) {
      if (!tracks_in_new_frame.contains(track)) {
        track_items_[track]->hide();
      }
    }
    std::swap(tracks_in_current_frame_, tracks_in_new_frame);
    current_frame_ = frame;
  }

 protected:
  void mousePressEvent(QGraphicsSceneMouseEvent *mouseEvent) {
    QGraphicsScene::mousePressEvent(mouseEvent);
    if (mouseEvent->isAccepted()) {
      return;
    }

    int x = mouseEvent->scenePos().x();
    int y = mouseEvent->scenePos().y();

    int new_track = tracks_->MaxTrack() + 1;

    TrackItem* item = new TrackItem(new_track);
    item->setPos(x, y);
    item->setSelected(true);
    track_items_[new_track] = item;
    addItem(item);

    tracks_->Insert(current_frame_, new_track, x, y);
  }

 private:
  QMap<int, TrackItem *> track_items_;
  QSet<int> tracks_in_current_frame_;
  libmv::Tracks *tracks_;
  int current_frame_;
  int selected_track_;
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
    scene_(new TrackerScene(tracks_.data())),
    view_(new View(scene_.data())),
    zoom_view_(new View(scene_.data())),
    current_frame_(-1),
    current_item_(0) {

  connect(scene_.data(), SIGNAL(selectionChanged()), SLOT(selectMarker()));
  connect(scene_.data(), SIGNAL(changed(QList<QRectF>)), SLOT(moveMarker()));

  setWindowTitle("LibMV Simple Tracker");
  setMaximumSize(qApp->desktop()->availableGeometry().size());
  connect(&play_timer_, SIGNAL(timeout()), SLOT(next()));

  QDockWidget* detailDock = new QDockWidget("Selected Track Details");
  detailDock->setObjectName("detailDock");
  addDockWidget(Qt::TopDockWidgetArea, detailDock);
  zoom_view_->setMinimumSize(2 * TrackItem::size, 2 * TrackItem::size);
  detailDock->setWidget(zoom_view_);

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
  slider_.style()->styleHint(QStyle::SH_Slider_AbsoluteSetButtons);
  connect(&slider_, SIGNAL(sliderMoved(int)), SLOT(seek(int)));
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
  scene_->clear();
  pixmap_ = 0;
  clip_->Open(paths);
  frame_number_.setMaximum(clip_->size() - 1);
  slider_.setMaximum(clip_->size() - 1);
  first();
}

void Tracker::seek(int frame) {
  // Bail out if there's nothing to do.
  if (frame == current_frame_) {
    return;
  }
  if (frame < 0 || frame >= clip_->size()) {
    stop();
    return;
  }
  int previous_frame = current_frame_;
  current_frame_ = frame;

  // Get and display the image.
  QImage image = clip_->Frame(current_frame_);
  if (!pixmap_) {
    pixmap_ = scene_->addPixmap(QPixmap::fromImage(image));
    view_->setMinimumSize(image.size()/2);
    view_->fitInView(pixmap_, Qt::KeepAspectRatio);
  } else {
    pixmap_->setPixmap(QPixmap::fromImage(image));
  }

  // If the shift is between consecutive frames, track the active trackers
  // from the previous frame into this one.
  if (current_frame_ == previous_frame + 1) {
    vector<libmv::Marker> markers_in_previous_frame;
    tracks_->MarkersInImage(previous_frame, &markers_in_previous_frame);
    foreach (const libmv::Marker &marker, markers_in_previous_frame) {
      // TODO(keir): For now this uses a fixed size region. What's needed is
      // an extension to use custom sized boxes around the tracked point.
      int size = 64;
      int half_size = size / 2;

      // [xy][01] is the upper right box corner.
      int x0 = marker.x - half_size;
      int y0 = marker.y - half_size;
      libmv::FloatImage old_patch;
      if (!CopyRegionFromQImage(clip_->Frame(previous_frame), size, size,
                                &x0, &y0,
                                &old_patch)) {
        continue;
      }

      int x1 = marker.x - half_size;
      int y1 = marker.y - half_size;
      libmv::FloatImage new_patch;
      if (!CopyRegionFromQImage(clip_->Frame(current_frame_), size, size,
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
        tracks_->Insert(current_frame_, marker.track, x1 + xx1, y1 + yy1);
        LG << "Tracked (" << xx0 << ", " << yy0 << ") to ("
           << xx1 << ", " << yy1 << ").";
      }
    }
  }

  zoom_view_->clearFocus();
  slider_.setValue(current_frame_);
  frame_number_.setValue(current_frame_);
  scene_->SetFrame(current_frame_);
}

void Tracker::first() {
  seek(0);
}

void Tracker::previous() {
  seek(current_frame_ - 1);
}

void Tracker::next() {
  seek(current_frame_ + 1);
}

void Tracker::last() {
  seek(clip_->size() - 1);
}

void Tracker::start() {
  play_action_->setChecked(true);
  play_timer_.start(50);
}

void Tracker::stop() {
  play_action_->setChecked(false);
  play_timer_.stop();
}

void Tracker::togglePlay(bool play) {
  if (play) {
    start();
  } else {
    stop();
  }
}

void Tracker::selectMarker() {
  if (scene_->selectedItems().isEmpty()) {
    current_item_ = NULL;
  } else {
    // Assume only TrackItem* are selectable.
    current_item_ = static_cast<TrackItem *>(scene_->selectedItems().first());
    if (!zoom_view_->hasFocus()) {
      zoom_view_->fitInView(current_item_, Qt::KeepAspectRatio);
    }
  }
}

void Tracker::moveMarker() {
  if(current_item_) {
    tracks_->Insert(current_frame_,
                    current_item_->track,
                    current_item_->pos().x(),
                    current_item_->pos().y());
    if (!zoom_view_->hasFocus()) {
      zoom_view_->fitInView(current_item_, Qt::KeepAspectRatio);
    }
  }
}

void Tracker::resizeEvent(QResizeEvent *) {
  view_->fitInView(pixmap_, Qt::KeepAspectRatio);
}

int main(int argc, char *argv[]) {
  libmv::Init("", &argc, &argv);
  QApplication app(argc, argv);
  Tracker window;
  window.show();
  return app.exec();
}
