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
#include <QDebug>
View::View(QWidget *parent) :
  QGLWidget(QGLFormat(QGL::SampleBuffers),parent),
  grab(0), pitch(PI/2), yaw(0), speed(1), walk(0), strafe(0), jump(0) {
  setAutoFillBackground(false);
  setFocusPolicy(Qt::StrongFocus);
  makeCurrent();
  glInit();

  /// TODO(MatthiasF): real bundles
  const int N=65536;
  vec3 pointData[N];
  for(int n=0; n<N; n++) pointData[n]=vec3(rand()-RAND_MAX/2,rand()-RAND_MAX/2,rand()-RAND_MAX/2)*(256.0/RAND_MAX);
  bundles.primitiveType = 1;
  bundles.upload(pointData,sizeof(pointData)/sizeof(vec3),sizeof(vec3));

  /// TODO(MatthiasF): real cameras
  const int C=256;
  vec3 lineData[C*16];
  for(int c=0; c<C; c++) {
    float angles[3];
    for(int i=0;i<3;i++) angles[i]=2*PI*rand()/RAND_MAX;
    mat4 transform;
    transform.rotateX(angles[0]);
    transform.rotateY(angles[1]);
    transform.rotateZ(angles[2]);
    transform.translate(vec3(rand()-RAND_MAX/2,rand()-RAND_MAX/2,rand()-RAND_MAX/2)*(256.0/RAND_MAX));
    transform.scale(16);
    vec3 base[4] = { vec3(-1,-1,1), vec3(1,-1,1), vec3(1,1,1), vec3(-1,1,1) };
    for(int i=0; i<4; i++) {
      lineData[c*16+i*4+0] = transform*vec3(0,0,0);
      lineData[c*16+i*4+1] = transform*base[i];
      lineData[c*16+i*4+2] = transform*base[i];
      lineData[c*16+i*4+3] = transform*base[(i+1)%4];
    }
  }
  cameras.primitiveType = 2;
  cameras.upload(lineData,sizeof(lineData)/sizeof(vec3),sizeof(vec3));
}
View::~View() {}

void View::paintGL() {
  int w=width(), h=height();
  projection=mat4(); projection.perspective(PI/4, (float)w/h, 1, 16384);
  view=mat4(); view.rotateX(-pitch); view.rotateZ(-yaw); view.translate(-position);
  glViewport(0,0,w,h);
  glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

  /// Display bundles
  static GLShader bundleShader;
  if(!bundleShader.id) {
    bundleShader.compile(glsl("vertex bundle"),glsl("fragment bundle"));
    bundleShader.bindFragments("color");
  }
  bundleShader.bind();
  bundleShader["viewProjectionMatrix"] = projection*view;
  bundles.bind();
  bundles.bindAttribute(&bundleShader,"position",3);
  glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
  glEnable(GL_POINT_SPRITE);
  glBlendFunc(GL_ONE,GL_ONE);
  glEnable(GL_BLEND);
  bundles.draw();
  glDisable(GL_BLEND);

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
  if( length(velocity) < 0.01 ) timer.stop();
  update();
}
