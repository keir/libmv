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

#ifndef UI_TRACKER_MULTIVIEW_H_
#define UI_TRACKER_MULTIVIEW_H_

#include <QGLWidget>
#include <QTimer>
#include "gl.h"

namespace libmv {
class Reconstruction;
} // namespace libmv

class View : public QGLWidget {
  Q_OBJECT
 public:
  View(QWidget *parent = 0);
  ~View();

  /*void Load(QByteArray data);
  QByteArray Save();*/
  
 protected:
  void paintGL();
  void keyPressEvent(QKeyEvent*);
  void keyReleaseEvent(QKeyEvent*);
  void mousePressEvent(QMouseEvent*);
  void mouseMoveEvent(QMouseEvent*);
  void mouseReleaseEvent(QMouseEvent*);
  void timerEvent(QTimerEvent*);

 private:
  QScopedPointer<libmv::Reconstruction> reconstruction_;

  QPoint drag;
  bool grab;
  float pitch,yaw,speed;
  int walk,strafe,jump;
  vec3 position;
  vec3 velocity;
  vec3 momentum;
  QBasicTimer timer;
  mat4 projection,view;

  GLBuffer points;
};

#endif
