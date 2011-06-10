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

View::View(QWidget *parent) :
  QGLWidget(QGLFormat(QGL::SampleBuffers),parent),
  grab(0), pitch(PI/2), yaw(0), speed(2), walk(0), strafe(0), jump(0) {
  setAutoFillBackground(false);
  setFocusPolicy(Qt::StrongFocus);
  makeCurrent();
  glInit();
}
View::~View() {}

void View::paintGL() {
  int w=width(),h=height();
  projection=mat4(); projection.perspective(PI/4, (float)w/h, 1, 16384);
  view=mat4(); view.rotateX(-pitch); view.rotateZ(-yaw); view.translate( -position );
  /// TODO(MatthiasF): display bundles, cameras, images, objects
}

void View::keyPressEvent(QKeyEvent* e) {
  if( e->key() == Qt::Key_W && walk<1 ) walk++;
  if( e->key() == Qt::Key_A && strafe>-1 ) strafe--;
  if( e->key() == Qt::Key_S && walk>-1 ) walk--;
  if( e->key() == Qt::Key_D && strafe<1 ) strafe++;
  if( e->key() == Qt::Key_Control && jump>-1 ) jump--;
  if( e->key() == Qt::Key_Space && jump<1 ) jump++;
  if( e->key() == Qt::Key_Shift ) speed=8;
  if(!timer.isActive()) timer.start(16,this);
}
void View::keyReleaseEvent(QKeyEvent* e) {
  if( e->key() == Qt::Key_W ) walk--;
  if( e->key() == Qt::Key_A ) strafe++;
  if( e->key() == Qt::Key_S ) walk++;
  if( e->key() == Qt::Key_D ) strafe--;
  if( e->key() == Qt::Key_Control ) jump++;
  if( e->key() == Qt::Key_Space ) jump--;
  if( e->key() == Qt::Key_Q ) velocity=vec3(0,0,0);
  if( e->key() == Qt::Key_Shift ) speed=2;
}
void View::mousePressEvent(QMouseEvent* e) {
  drag=e->pos();
}
void View::mouseReleaseEvent(QMouseEvent*) {
  if(grab) {
    setCursor(QCursor());
    grab=false;
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
  if( length(velocity) > 0.1 ) update(); else timer.stop();
}
