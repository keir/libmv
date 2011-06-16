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

#ifndef UI_TRACKER_SCENE_H_
#define UI_TRACKER_SCENE_H_

#include <QGLWidget>
#include <QTimer>
#include "ui/tracker/gl.h"

namespace libmv {
class Point;
class Camera;
class Reconstruction;
} // namespace libmv

struct Object {
  mat4 transform;
  QVector<int> tracks;
  void position(libmv::Reconstruction* reconstruction, vec3* min, vec3* max);
};

class Scene : public QGLWidget {
  Q_OBJECT
 public:
  Scene(QGLWidget *shareWidget = 0);
  ~Scene();
  void LoadCameras(QByteArray data);
  void LoadBundles(QByteArray data);
  void LoadObjects(QByteArray data);
  QByteArray SaveCameras();
  QByteArray SaveBundles();
  QByteArray SaveObjects();
  void SetImage(int);
  void RenderOverlay(int w,int h,int image);
  
 public slots:
  void select(QVector<int>);
  void add();
  void link();
  void upload();

 signals:
  void trackChanged(QVector<int> track);
  void imageChanged(int image);
  void objectChanged();

 protected:
  void paintGL();
  void keyPressEvent(QKeyEvent*);
  void keyReleaseEvent(QKeyEvent*);
  void mousePressEvent(QMouseEvent*);
  void mouseMoveEvent(QMouseEvent*);
  void mouseReleaseEvent(QMouseEvent*);
  void timerEvent(QTimerEvent*);

 private:
  void DrawPoint(libmv::Point point, QVector<vec3>& points);
  void DrawCamera(libmv::Camera camera, QVector<vec3>& lines);
  void DrawObject(Object object, QVector<vec3>& quads);

  QScopedPointer<libmv::Reconstruction> reconstruction_;
  QVector<Object> objects_;

  GLBuffer bundles_;
  GLBuffer cameras_;
  GLBuffer cubes_;

  QPoint drag_;
  bool grab_;
  float pitch_,yaw_,speed_;
  int walk_,strafe_,jump_;
  vec3 position_;
  vec3 velocity_;
  vec3 momentum_;
  QBasicTimer timer_;
  mat4 projection_;
  mat4 view_;

  QVector<int> selected_tracks_;
  int current_image_;
  Object* active_object_;
};

#endif
