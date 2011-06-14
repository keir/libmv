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

#include "ui/tracker/view.h"
#include "ui/tracker/gl.h"

#include <QMouseEvent>
#include <QKeyEvent>

#include "libmv/simple_pipeline/reconstruction.h"

using libmv::Camera;
using libmv::Point;
using libmv::Vec3;
using libmv::Mat3;

View::View(QWidget *parent) :
  QGLWidget(QGLFormat(QGL::SampleBuffers),parent),
  reconstruction_(new libmv::Reconstruction()),
  grab(0), pitch(PI/2), yaw(0), speed(1), walk(0), strafe(0), jump(0),
  selectedImage(-1) {
  setAutoFillBackground(false);
  setFocusPolicy(Qt::StrongFocus);
  makeCurrent();
  glInit();

  /// TODO(MatthiasF): real bundles
  const int nofTracks = 65536;
  for(int track = 0; track < nofTracks; track++) {
    vec3 random = vec3(rand()-RAND_MAX/2,rand()-RAND_MAX/2,rand()-RAND_MAX/2)*(256.0/RAND_MAX);
    reconstruction_->InsertPoint(track,Vec3(random.x,random.y,random.z));
  }

  /// TODO(MatthiasF): real cameras
  const int nofImages=256;
  for(int image = 0; image < nofImages; image++) {
    vec3 random = vec3(rand()-RAND_MAX/2,rand()-RAND_MAX/2,rand()-RAND_MAX/2)*(256.0/RAND_MAX);
    float angles[3];
    for(int i=0;i<3;i++) angles[i]=2*PI*rand()/RAND_MAX;
    mat4 transform;
    transform.rotateX(angles[0]);
    transform.rotateY(angles[1]);
    transform.rotateZ(angles[2]);
    Mat3 R;
    for(int i = 0 ; i < 3 ; i++) for(int j = 0 ; j < 3 ; j++) {
      R(i,j)=transform(i,j);
    }
    reconstruction_->InsertCamera(image,R,Vec3(random.x,random.y,random.z));
  }

  upload();
}
View::~View() {}

void View::LoadCameras(QByteArray data) {
  const Camera *cameras = reinterpret_cast<const Camera *>(data.constData());
  for (size_t i = 0; i < data.size() / sizeof(Camera); ++i) {
    Camera camera = cameras[i];
    reconstruction_->InsertCamera(camera.image,camera.R,camera.t);
  }
}

void View::LoadPoints(QByteArray data) {
  const Point *points = reinterpret_cast<const Point *>(data.constData());
  for (size_t i = 0; i < data.size() / sizeof(Point); ++i) {
    Point point = points[i];
    reconstruction_->InsertPoint(point.track,point.X);
  }
}

QByteArray View::SaveCameras() {
  std::vector<Camera> cameras = reconstruction_->AllCameras();
  return QByteArray(reinterpret_cast<char *>(cameras.data()),
                    cameras.size() * sizeof(Camera));
}

QByteArray View::SavePoints() {
  std::vector<Point> points = reconstruction_->AllPoints();
  return QByteArray(reinterpret_cast<char *>(points.data()),
                    points.size() * sizeof(Point));
}

void addCamera(Camera camera, QVector<vec3>& lines) {
  mat4 rotation;
  for(int i = 0 ; i < 3 ; i++) for(int j = 0 ; j < 3 ; j++) {
    rotation(i,j)=camera.R(i,j);
  }
  mat4 transform;
  transform.translate(vec3(camera.t.x(), camera.t.y(), camera.t.z()));
  transform = transform * rotation;
  vec3 base[4] = { vec3(-1,-1,1), vec3(1,-1,1), vec3(1,1,1), vec3(-1,1,1) };
  for(int i=0; i<4; i++) {
    lines << transform*vec3(0,0,0) << transform*base[i];
    lines << transform*base[i] << transform*base[(i+1)%4];
  }
}

void View::upload() {
  std::vector<Point> allPoints = reconstruction_->AllPoints();
  QVector<vec3> points; points.reserve(allPoints.size());
  foreach(Point point, allPoints) {
    points << vec3( point.X.x(), point.X.y(), point.X.z() );
  }
  foreach(int track, selectedTracks) {
    Point point = *reconstruction_->PointForTrack(track);
    vec3 p( point.X.x(), point.X.y(), point.X.z() );
    points << p << p << p;
  }
  bundles.primitiveType = 1;
  bundles.upload(points.constData(), points.count(), sizeof(vec3));

  std::vector<Camera> allCameras = reconstruction_->AllCameras();
  QVector<vec3> lines; lines.reserve(allCameras.size()*16);
  foreach(Camera camera, allCameras) addCamera(camera,lines);
  if(selectedImage>=0) {
    addCamera(*reconstruction_->CameraForImage(selectedImage),lines);
    addCamera(*reconstruction_->CameraForImage(selectedImage),lines);
    addCamera(*reconstruction_->CameraForImage(selectedImage),lines);
  }
  cameras.primitiveType = 2;
  cameras.upload(lines.constData(), lines.count(), sizeof(vec3));
}

void View::paintGL() {
  int w=width(), h=height();
  projection=mat4(); projection.perspective(PI/4, (float)w/h, 1, 16384);
  view=mat4(); view.rotateX(-pitch); view.rotateZ(-yaw); view.translate(-position);
  glViewport(0,0,w,h);
  glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
  glBlendFunc(GL_ONE,GL_ONE);
  glEnable(GL_BLEND);

  /// Display bundles
  static GLShader bundleShader;
  if(!bundleShader.id) {
    bundleShader.compile(glsl("vertex bundle"),glsl("fragment bundle"));
    bundleShader.bindFragments("color");
  }
  bundleShader.bind();
  bundleShader["viewProjectionMatrix"] = projection*view;
  bundleShader["pointSize"] = (float)w;
  bundles.bind();
  bundles.bindAttribute(&bundleShader,"position",3);
  glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
  glEnable(GL_POINT_SPRITE);
  bundles.draw();

  /// Display cameras
  static GLShader cameraShader;
  if(!cameraShader.id) {
    cameraShader.compile(glsl("vertex camera"),glsl("fragment camera"));
    cameraShader.bindFragments("color");
  }
  cameraShader.bind();
  cameraShader["viewProjectionMatrix"] = projection*view;
  cameraShader.bind();
  cameras.bind();
  cameras.bindAttribute(&cameraShader,"position",3);
  cameras.draw();
}

void View::keyPressEvent(QKeyEvent* e) {
  if( e->key() == Qt::Key_W && walk<1 ) walk++;
  if( e->key() == Qt::Key_A && strafe>-1 ) strafe--;
  if( e->key() == Qt::Key_S && walk>-1 ) walk--;
  if( e->key() == Qt::Key_D && strafe<1 ) strafe++;
  if( e->key() == Qt::Key_Control && jump>-1 ) jump--;
  if( e->key() == Qt::Key_Space && jump<1 ) jump++;
  if(!timer.isActive()) timer.start(16,this);
}
void View::keyReleaseEvent(QKeyEvent* e) {
  if( e->key() == Qt::Key_W ) walk--;
  if( e->key() == Qt::Key_A ) strafe++;
  if( e->key() == Qt::Key_S ) walk++;
  if( e->key() == Qt::Key_D ) strafe--;
  if( e->key() == Qt::Key_Control ) jump++;
  if( e->key() == Qt::Key_Space ) jump--;
}
void View::mousePressEvent(QMouseEvent* e) {
  drag=e->pos();
}

void View::mouseReleaseEvent(QMouseEvent* e) {
  if(grab) {
    setCursor(QCursor());
    grab=false;
  } else {
    mat4 transform = projection;
    transform.rotateX(-pitch);
    transform.rotateZ(-yaw);
    //FIXME: projection seems wrong, selection is closer to screen center than cursor.
    vec3 d = transform.inverse() * normalize(vec3(2.0*e->x()/width()-1,1-2.0*e->y()/height(),1));
    float minD=1;
    int hitTrack=-1,hitImage=-1;
    foreach(Point point, reconstruction_->AllPoints()) {
      vec3 o = vec3(point.X.x(),point.X.y(),point.X.z())-position;
      double t = dot(d,o) / dot(d,d);
      if (t < 0) continue;
      vec3 p = t * d;
      if( length(p-o) < minD ) {
        minD = length(p-o);
        hitTrack = point.track;
      }
    }
    foreach(Camera camera, reconstruction_->AllCameras()) {
      vec3 o = vec3(camera.t.x(),camera.t.y(),camera.t.z())-position;
      double t = dot(d,o) / dot(d,d);
      if (t < 0) continue;
      vec3 p = t * d;
      if( length(p-o) < minD ) {
        minD = length(p-o);
        hitTrack = -1; hitImage=camera.image;
      }
    }
    if(hitTrack>=0) {
      if(selectedTracks.contains(hitTrack)) {
        selectedTracks.remove(selectedTracks.indexOf(hitTrack));
      } else {
        selectedTracks << hitTrack;
      }
      emit trackChanged(hitTrack);
    } else {
      selectedTracks.clear();
    }
    if(hitImage>=0) {
      selectedImage=hitImage;
      emit imageChanged(hitImage);
    } else {
      selectedImage=-1;
    }
    upload();
    update();
  }
}
void View::mouseMoveEvent(QMouseEvent *e) {
  if(!grab) {
    if((drag-e->pos()).manhattanLength()<16) return;
    grab=true;
    setCursor(QCursor(Qt::BlankCursor));
    QCursor::setPos(mapToGlobal(QPoint(width()/2,height()/2)));
    return;
  }
  QPoint delta = e->pos()-QPoint(width()/2,height()/2);
  yaw=yaw-delta.x()*PI/width(); pitch=qBound(0.0,pitch-delta.y()*PI/width(),PI);
  QCursor::setPos(mapToGlobal(QPoint(width()/2,height()/2)));
  if(!timer.isActive()) update();
}
void View::timerEvent(QTimerEvent*) {
  mat4 view; view.rotateZ(yaw); view.rotateX(pitch);
  velocity += view*vec3(strafe*speed,0,-walk*speed)+vec3(0,0,jump*speed);
  velocity *= 31.0/32; position += velocity;
  if( length(velocity) < 0.01 ) timer.stop();
  update();
}
