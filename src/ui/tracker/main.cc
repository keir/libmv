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
#include "ui/tracker/scene.h"

#include "libmv/tools/tool.h"
#include "libmv/simple_pipeline/initialize_reconstruction.h"
#include "libmv/simple_pipeline/bundle.h"
#include "libmv/simple_pipeline/pipeline.h"
#include "libmv/simple_pipeline/uncalibrated_reconstructor.h"
#include "libmv/simple_pipeline/camera_intrinsics.h"

#include <QApplication>
#include <QFileDialog>
#include <QFormLayout>
#include <QDockWidget>
#include <QToolButton>
#include <QSettings>
#include <QToolBar>
#include <QAction>
#include <QCache>
#include <QMenu>
#include <QTime>

#include <float.h>

void Clip::Open(QString path) {
  clear();
  cache_.setMaxCost(64 * 1024 * 1024);
  foreach (QString file, QDir(path).entryList(QStringList("*.jpg") << "*.png",
                                              QDir::Files, QDir::Name)) {
    append(QDir(path).filePath(file));
  }
}

QImage Clip::Image(int frame) {
  QImage* image = cache_[frame];
  if (!image) {
    image = new QImage(value(frame));
    cache_.insert(frame, image, image->byteCount());
  }
  return *image;
}

MainWindow::MainWindow()
  : clip_(new Clip(this)),
    tracks_(new libmv::Tracks()),
    intrinsics_(new libmv::CameraIntrinsics()),
    reconstruction_(new libmv::EuclideanReconstruction()),
    scene_(new Scene(intrinsics_, reconstruction_)),
    tracker_(new Tracker(tracks_, scene_, scene_)),
    current_frame_(-1) {
  setWindowTitle("Tracker");
  setCentralWidget(tracker_);
  connect(tracker_, SIGNAL(trackChanged(QVector<int>)),
          this, SLOT(updateZooms(QVector<int>)));

  QToolBar* toolbar = addToolBar("Main Toolbar");
  toolbar->setObjectName("mainToolbar");

  toolbar->addAction(QIcon(":/open"), "Open a new sequence...",
                     this, SLOT(open()));

  QAction* tracker_action_ = toolbar->addAction(QIcon(":/view-image"),
                                                "Tracker View");
  tracker_action_->setCheckable(true);
  tracker_action_->setChecked(true);
  connect(tracker_action_, SIGNAL(triggered(bool)),
          tracker_, SLOT(setVisible(bool)));

  zoom_action_ = toolbar->addAction(QIcon(":/view-zoom"), "Zoom View");
  zoom_action_->setCheckable(true);
  zoom_action_->setChecked(true);
  connect(zoom_action_, SIGNAL(triggered(bool)), this, SLOT(toggleZoom(bool)));

  QDockWidget* scene_dock = new QDockWidget("Scene View");
  scene_dock->setObjectName("sceneDock");
  addDockWidget(Qt::BottomDockWidgetArea, scene_dock);
  scene_dock->setWidget(scene_);
  scene_dock->toggleViewAction()->setIcon(QIcon(":/view-scene"));
  scene_dock->toggleViewAction()->setChecked(false);
  toolbar->addAction(scene_dock->toggleViewAction());
  connect(scene_, SIGNAL(imageChanged(int)), SLOT(seek(int)));
  connect(scene_, SIGNAL(trackChanged(QVector<int>)),
          tracker_, SLOT(select(QVector<int>)));

  toolbar->addSeparator();

  QToolButton* delete_button = new QToolButton();
  toolbar->addWidget(delete_button);
  QMenu* delete_popup = new QMenu();
  delete_popup->addAction(QIcon(":/delete"),
                          "Delete current marker",
                          tracker_, SLOT(deleteSelectedMarkers()));
  QAction* delete_track = delete_popup->addAction(QIcon(":/delete-row"),
                                                "Delete current track",
                                                tracker_,
                                                SLOT(deleteSelectedTracks()));
  delete_button->setMenu(delete_popup);
  delete_button->setDefaultAction(delete_track);
  delete_button->setPopupMode(QToolButton::MenuButtonPopup);
  connect(delete_popup, SIGNAL(triggered(QAction*)),
          delete_button, SLOT(setDefaultAction(QAction*)));

  track_action_ = toolbar->addAction(QIcon(":/record"), "Track selected markers");
  track_action_->setCheckable(true);
  connect(track_action_, SIGNAL(triggered(bool)), SLOT(toggleTracking(bool)));
  connect(tracker_action_, SIGNAL(triggered(bool)),
          track_action_, SLOT(setVisible(bool)));

  keyframe_action_ = toolbar->addAction(QIcon(":/keyframe"), "Add Keyframe");
  keyframe_action_->setCheckable(true);
  connect(keyframe_action_, SIGNAL(triggered(bool)), SLOT(toggleKeyframe(bool)));
  connect(tracker_action_, SIGNAL(triggered(bool)),
          keyframe_action_, SLOT(setVisible(bool)));

  toolbar->addAction(QIcon(":/solve"), "Solve reconstruction",
                     this, SLOT(solve()));
  QAction* add_action = toolbar->addAction(QIcon(":/add"), "Add object",
                                           scene_, SLOT(add()));
  connect(scene_dock->toggleViewAction(), SIGNAL(triggered(bool)),
          add_action, SLOT(setVisible(bool)));

  QAction* link_action = toolbar->addAction(
        QIcon(":/link"), "Link active object to selected bundles",
        scene_, SLOT(link()));
  connect(scene_dock->toggleViewAction(), SIGNAL(triggered(bool)),
          link_action, SLOT(setVisible(bool)));

  toolbar->addSeparator();

  toolbar->addAction(QIcon(":/skip-backward"), "Seek to first frame",
                     this, SLOT(first()));
  toolbar->addAction(QIcon(":/step-backward"), "Step to previous frame",
                     this, SLOT(previous()))->setShortcut(QKeySequence("Left"));
  backward_action_ = toolbar->addAction(QIcon(":/play-backward"),
                                        "Play sequence backwards");
  backward_action_->setCheckable(true);
  connect(backward_action_, SIGNAL(triggered(bool)),
          SLOT(toggleBackward(bool)));
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
  toolbar->addAction(QIcon(":/step-forward"), "Next Frame", this, SLOT(next()))
      ->setShortcut(QKeySequence("Right"));
  toolbar->addAction(QIcon(":/skip-forward"), "Last Frame", this, SLOT(last()));

  toolbar->addWidget(&keyframe_label_);
  keyframe_label_.setToolTip("Shows number of frame (since previous|until next) "
                             "keyframe and common tracks between current frame "
                             "and (previous|next) keyframe in parenthesis");

  restoreGeometry(QSettings().value("geometry").toByteArray());
  restoreState(QSettings().value("windowState").toByteArray());
}
void MainWindow::Save(QString name, QByteArray data) {
  if (data.isEmpty()) return;
  QFile file(QDir(path_).filePath(name));
  if (file.open(QFile::WriteOnly | QIODevice::Truncate)) {
    file.write(data);
  }
}
MainWindow::~MainWindow() {
  QSettings().setValue("geometry", saveGeometry());
  QSettings().setValue("windowState", saveState());
  QSettings().setValue("currentFrame", current_frame_);
  if (clip_->isEmpty()) return;
  Save("tracks", tracker_->Save());
  Save("cameras", scene_->SaveCameras());
  Save("bundles", scene_->SaveBundles());
  Save("objects", scene_->SaveObjects());
  Save("keyframes", QByteArray((char*)keyframes_.data(),
                               keyframes_.size()*sizeof(int)));
  delete reconstruction_;
  delete tracks_;
}

QByteArray MainWindow::Load(QString name) {
  QFile file(QDir(path_).filePath(name));
  return file.open(QFile::ReadOnly) ? file.readAll() : QByteArray();
}

struct Parameter {
  const char* name;
  const char* suffix;
  double min;
  double max;
  double value;
};

void MainWindow::open() {
  open(QFileDialog::getExistingDirectory(this, "Select sequence folder"));
  if (clip_->isEmpty()) return;
  QSize size = clip_->Image(0).size();
  QDialog dialog(this);
  dialog.setWindowTitle("Camera Parameters");
  QFormLayout layout(&dialog);

  const int kCount = 4;
  const Parameter parameters[kCount] = {
    {"Focal Length",              "px", 0,  FLT_MAX,       size.width()*2  },
    {"Principal Point (X)",       "px", 0,  size.width(),  size.width()/2  },
    {"Principal Point (Y)",       "px", 0,  size.height(), size.height()/2 },
    {"1st Radial Distortion",     "",  -1,  1,             0               },
  };
  QDoubleSpinBox spinbox[kCount];
  QByteArray data = Load("settings");
  data.reserve(kCount*sizeof(float));
  float* values = reinterpret_cast<float*>(data.data());
  for (int i = 0; i < kCount; i++) {
    Parameter parameter = parameters[i];
    spinbox[i].setSuffix(parameter.suffix);
    spinbox[i].setRange(parameter.min, parameter.max);
    if (data.isEmpty()) {
      spinbox[i].setValue(parameter.value);
    } else {
      spinbox[i].setValue(values[i]);
    }
    layout.addRow(parameter.name, &spinbox[i]);
  }
  dialog.exec();
  data.resize(kCount*sizeof(float));
  for (int i = 0; i < kCount; i++) {
    values[i] = spinbox[i].value();
  }
  Save("settings", data);
  intrinsics_->SetFocalLength(values[0]);
  intrinsics_->set_principal_point(values[1],values[2]);
  intrinsics_->set_radial_distortion(values[3], 0, 0);
}

void MainWindow::open(QString path) {
  if (path.isEmpty() || !QDir(path).exists()) return;
  clip_->Open(path);
  if (clip_->isEmpty()) return;
  path_ = path;
  setWindowTitle(QString("Tracker - %1").arg(QDir(path).dirName()));
  tracker_->Load(Load("tracks"));

  foreach(QString path, QDir(path_).entryList(QStringList("*.dae"))) {
    QFile file(path);
    scene_->LoadCOLLADA(&file);
  }
  //scene_->LoadCameras(Load("cameras"));
  //scene_->LoadBundles(Load("bundles"));
  //scene_->LoadObjects(Load("objects"));
  QByteArray data = Load("keyframes");
  const int *keyframes = reinterpret_cast<const int*>(data.constData());
  for (size_t i = 0; i < data.size() / sizeof(int); ++i) {
    keyframes_ << keyframes[i];
  }
  //scene_->upload();
  spinbox_.setMaximum(clip_->size() - 1);
  slider_.setMaximum(clip_->size() - 1);
  int frame = QSettings().value("currentFrame", 0).toInt();
  if(frame < clip_->size()) {
    seek(frame);
  } else {
    seek(0);
  }
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

  slider_.setValue(current_frame_);
  spinbox_.setValue(current_frame_);
  tracker_->SetImage(current_frame_, clip_->Image(current_frame_),
                     track_action_->isChecked());
  if (keyframes_.contains(current_frame_)) {
    keyframe_action_->setChecked(true);
  } else {
    keyframe_action_->setChecked(false);
    int i = 0;
    for(; i < keyframes_.count(); i++) {
      if(keyframes_[i] >= current_frame_) break;
    }
    QString text;
    if(i>0) {
      int previous = keyframes_[i-1];
      int common =
          tracks_->MarkersForTracksInBothImages(current_frame_,previous).size()/2;
      text += QString("%1 (%2) |").arg(current_frame_-previous).arg(common);
    }
    if(i<keyframes_.count()) {
      int next = keyframes_[i];
      int common =
          tracks_->MarkersForTracksInBothImages(current_frame_,next).size()/2;
      text += QString("| %1 (%2)").arg(next-current_frame_).arg(common);
    }
    //keyframe_label_.setText(text);
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

void MainWindow::toggleKeyframe(bool keyframe) {
  stop();
  if (keyframe) {
    int i = 0;
    for(; i < keyframes_.count(); i++) {
      if (keyframes_[i] >= current_frame_) {
        Q_ASSERT(keyframes_[i] != current_frame_);
        break;
      }
    }
    keyframes_.insert(i, current_frame_);
  } else {
    if (keyframes_.contains(current_frame_)) {
      keyframes_.remove(keyframes_.indexOf(current_frame_));
    }
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

void MainWindow::toggleZoom(bool zoom) {
  if (zoom) {
    foreach (QDockWidget* dock, zooms_docks_) {
      dock->show();
    }
  } else {
    foreach (QDockWidget* dock, zooms_docks_) {
      dock->hide();
    }
  }
}

void MainWindow::updateZooms(QVector<int> tracks) {
  if (!zoom_action_->isChecked()) return;
  if (!zooms_docks_.isEmpty() &&
      tracks.size()*zooms_docks_.first()->width() > width()) {
    tracks.resize(width() / zooms_docks_.first()->width());
  }
  for (int i = tracks.size(); i < zooms_docks_.size(); i++) {
    removeDockWidget(zooms_docks_[i]);
    delete zooms_docks_[i];
    // zoom widget is owned by dock ?
  }
  for (int i = zooms_docks_.size(); i < tracks.size(); i++) {
    QDockWidget* dock = new QDockWidget(QString("Marker #%1").arg(tracks[i]));
    dock->setObjectName(QString("zoom%1").arg(tracks[i]));
    addDockWidget(Qt::TopDockWidgetArea, dock);
    Zoom* zoom = new Zoom(0, tracker_);
    dock->setWidget(zoom);
    addDockWidget(Qt::TopDockWidgetArea, dock);
    zooms_ << zoom;
    zooms_docks_ << dock;
  }
  zooms_docks_.resize(tracks.size());
  zooms_.resize(tracks.size());
  for (int i = 0; i < tracks.size(); i++) {
    zooms_[i]->SetMarker(current_frame_, tracks[i]);
  }
}

void MainWindow::solve() {
  // Invert the camera intrinsics.
  /*
  libmv::vector<libmv::Marker> markers = tracks_->AllMarkers();
  for (int i = 0; i < markers.size(); ++i) {
    intrinsics_->InvertIntrinsics(markers[i].x,
                                  markers[i].y,
                                  &(markers[i].x),
                                  &(markers[i].y));
  }
  libmv::Tracks normalized_tracks(markers);

  Q_ASSERT(keyframes_.size() >= 2);

  libmv::vector<libmv::Marker> keyframe_markers =
    normalized_tracks.MarkersForTracksInBothImages(keyframes_[0],
                                                   keyframes_[1]);

  libmv::ReconstructTwoFrames(keyframe_markers, reconstruction_);
  libmv::Bundle(normalized_tracks, reconstruction_);
  libmv::CompleteReconstruction(normalized_tracks, reconstruction_);
  libmv::ReprojectionError(*tracks_, *reconstruction_, *intrinsics_);
  */
  QSize size = clip_->Image(0).size();
  libmv::UncalibratedReconstructor reconstructor(size.width(),
                                                 size.height(),
                                                 keyframes_[0],
                                                 keyframes_[1],
                                                 *tracks_);
  *reconstruction_ = reconstructor.euclidean_reconstruction();

  libmv::EuclideanReprojectionError(*tracks_, *reconstruction_, *intrinsics_);
  scene_->upload();
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

