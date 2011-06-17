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

#include "ui/tracker/scene.h"
#include "ui/tracker/gl.h"

#include "libmv/simple_pipeline/reconstruction.h"

#include <QMouseEvent>
#include <QKeyEvent>

using libmv::Camera;
using libmv::Point;
using libmv::Vec3;
using libmv::Mat3;
using libmv::Mat34;

static void P_From_KRt(const Mat3 &K, const Mat3 &R, const Vec3 &t, Mat34 *P) {
  P->block<3, 3>(0, 0) = R;
  P->col(3) = t;
  (*P) = K * (*P);
}

/// TODO(MatthiasF): keep this define as long we don't have reconstruction.
#define RANDOM

void Object::position(libmv::Reconstruction* reconstruction, vec3* min, vec3* max) {
  vec3 mean;
  if(!tracks.isEmpty()) {
    foreach(int track, tracks) {
      Point point = *reconstruction->PointForTrack(track);
      mean += vec3(point.X.x(), point.X.y(), point.X.z());
    }
    mean /= tracks.count();
  }
  *min = mean-vec3(1,1,1);
  *max = mean+vec3(1,1,1);
}

QDataStream& operator>>(QDataStream& s, mat4& v) {
  s.readRawData((char*)v.data,sizeof(v.data));
  return s;
}
QDataStream& operator<<(QDataStream& s, const mat4& v) {
  s.writeRawData((char*)v.data,sizeof(v.data));
  return s;
}
QDataStream& operator>>(QDataStream& s, Object& v) {
  return s >> v.transform >> v.tracks;
}
QDataStream& operator<<(QDataStream& s, const Object& v) {
  return s << v.transform << v.tracks;
}

Scene::Scene(libmv::Reconstruction *reconstruction, QGLWidget *shareWidget) :
  QGLWidget(QGLFormat(QGL::SampleBuffers),0,shareWidget),
  reconstruction_(reconstruction),
  grab_(0), pitch_(PI/2), yaw_(0), speed_(1), walk_(0), strafe_(0), jump_(0),
  current_image_(-1), active_object_(0) {
  setAutoFillBackground(false);
  setFocusPolicy(Qt::StrongFocus);
  makeCurrent();
  glInitialize();

#ifdef RANDOM
  const int nofTracks = 65536;
  for(int track = 0; track < nofTracks; track++) {
    vec3 random = vec3(rand()-RAND_MAX/2,rand()-RAND_MAX/2,rand()-RAND_MAX/2)*(256.0/RAND_MAX);
    reconstruction_->InsertPoint(track,Vec3(random.x,random.y,random.z));
  }

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
#endif
}
Scene::~Scene() {}

void Scene::LoadCameras(QByteArray data) {
  const Camera *cameras = reinterpret_cast<const Camera *>(data.constData());
  for (size_t i = 0; i < data.size() / sizeof(Camera); ++i) {
    Camera camera = cameras[i];
    reconstruction_->InsertCamera(camera.image,camera.R,camera.t);
  }
}

void Scene::LoadBundles(QByteArray data) {
  const Point *points = reinterpret_cast<const Point *>(data.constData());
  for (size_t i = 0; i < data.size() / sizeof(Point); ++i) {
    Point point = points[i];
    reconstruction_->InsertPoint(point.track,point.X);
  }
}

void Scene::LoadObjects(QByteArray data) {
  QDataStream stream(data);
  stream >> objects_;
}

void Scene::LoadCalibration(QByteArray data) {
  calibration_ = *(Calibration*)data.constData();
}

QByteArray Scene::SaveCameras() {
#ifdef RANDOM
  return QByteArray();
#endif
  std::vector<Camera> cameras = reconstruction_->AllCameras();
  return QByteArray(reinterpret_cast<char *>(cameras.data()),
                    cameras.size() * sizeof(Camera));
}

QByteArray Scene::SaveBundles() {
#ifdef RANDOM
  return QByteArray();
#endif
  std::vector<Point> points = reconstruction_->AllPoints();
  return QByteArray(reinterpret_cast<char *>(points.data()),
                    points.size() * sizeof(Point));
}

QByteArray Scene::SaveObjects() {
  if(objects_.isEmpty()) return QByteArray();
  QByteArray data;
  QDataStream stream(&data,QIODevice::WriteOnly);
  stream << objects_;
  return data;
}

void Scene::SetImage(int image) {
  current_image_ = image;
}

void Scene::select(QVector<int> tracks) {
  selected_tracks_ = tracks;
  upload();
}

void Scene::add() {
  objects_ << Object();
  active_object_ = &objects_.last();
  upload();
  emit objectChanged();
}

void Scene::link() {
  if(active_object_) {
    active_object_->tracks = selected_tracks_;
    upload();
    emit objectChanged();
  }
}

void Scene::DrawPoint(Point point, QVector<vec3>& points) {
  points << vec3( point.X.x(), point.X.y(), point.X.z() );
}

void Scene::DrawCamera(Camera camera, QVector<vec3>& lines) {
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

void Scene::DrawObject(Object object, QVector<vec3>& quads) {
  vec3 min,max;
  object.position(reconstruction_,&min,&max);
  const int indices[6*4] = { 0,2,6,4,      1,3,7,5,     0,1,5,4,      2,3,7,6,     0,1,3,2,      4,5,7,6, };
  //const vec3 faces[6] = { vec3(-1,0,0), vec3(1,0,0), vec3(0,-1,0), vec3(0,1,0), vec3(0,0,-1), vec3(0,0,1) };
  for(int i=0;i<6*4;i++) {
    int m=indices[i];
    quads << vec3(m&1?min.x:max.x,m&2?min.y:max.y,m&4?min.z:max.z);
    //quads << faces[i/4]; //no need for normals without any lights
  }
}

void Scene::upload() {
  std::vector<Point> all_points = reconstruction_->AllPoints();
  QVector<vec3> points; points.reserve(all_points.size());
  foreach(Point point, all_points) {
    DrawPoint(point,points);
  }
  foreach(int track, selected_tracks_) {
    Point point = *reconstruction_->PointForTrack(track);
    DrawPoint(point,points);
    DrawPoint(point,points);
    DrawPoint(point,points);
  }
  bundles_.primitiveType = 1;
  bundles_.upload(points.constData(), points.count(), sizeof(vec3));

  std::vector<Camera> all_cameras = reconstruction_->AllCameras();
  QVector<vec3> lines; lines.reserve(all_cameras.size()*16);
  foreach(Camera camera, all_cameras) DrawCamera(camera,lines);
  if(current_image_>=0) {
    DrawCamera(*reconstruction_->CameraForImage(current_image_),lines);
    DrawCamera(*reconstruction_->CameraForImage(current_image_),lines);
    DrawCamera(*reconstruction_->CameraForImage(current_image_),lines);
  }
  cameras_.primitiveType = 2;
  cameras_.upload(lines.constData(), lines.count(), sizeof(vec3));

  QVector<vec3> quads; quads.reserve(objects_.size()*6*4*2);
  foreach(Object object, objects_) {
    DrawObject(object,quads);
  }
  if(active_object_) {
    DrawObject(*active_object_,quads);
    DrawObject(*active_object_,quads);
    DrawObject(*active_object_,quads);
  }
  cubes_.primitiveType = 4;
  cubes_.upload(quads.constData(), quads.count(), sizeof(vec3));

  update();
}

void Scene::Render(int w,int h,int image) {
  /// Compute camera projection
  mat4 transform;
  if(image >=0) {
    // TODO(MatthiasF): match real image projection.
    Camera camera = *reconstruction_->CameraForImage(image);
    Mat3 K;
    K << calibration_.focal_length, 0, (w-1)/2,
         0, calibration_.focal_length, (h-1)/2,
         0, 0,                         1;
    Mat34 P;
    P_From_KRt(K, camera.R, camera.t, &P);
    for(int j = 0 ; j < 4 ; j++) {
      transform(0,j) = P(0,j);
      transform(1,j) = P(1,j);
      transform(2,j) = 0;
      transform(3,j) = P(2,j);
    }
  } else {
    projection_=mat4(); projection_.perspective(PI/4, (float)w/h, 1, 16384);
    view_=mat4(); view_.rotateX(-pitch_); view_.rotateZ(-yaw_); view_.translate(-position_);
    transform = projection_ * view_;
  }

  /// Display bundles
  static GLShader bundle_shader;
  if(!bundle_shader.id) {
    bundle_shader.compile(glsl("vertex transform distort bundle"),glsl("fragment bundle"));
  }
  bundle_shader.bind();
  bundle_shader["transform"] = transform;
  bundle_shader["center"] = vec2(0,0);
  bundle_shader["K1"] = calibration_.radial_distortion[0];
  bundles_.bind();
  bundles_.bindAttribute(&bundle_shader,"position",3);
  glAdditiveBlendMode();
  bundles_.draw();

  /// Display cameras
  static GLShader camera_shader;
  if(!camera_shader.id) {
    camera_shader.compile(glsl("vertex transform camera"),glsl("fragment camera"));
  }
  camera_shader.bind();
  camera_shader["transform"] = transform;
  cameras_.bind();
  cameras_.bindAttribute(&camera_shader,"position",3);
  cameras_.draw();

  /// Display objects
  static GLShader object_shader;
  if(!object_shader.id) {
    object_shader.compile(glsl("vertex transform object"),glsl("fragment object"));
  }
  object_shader.bind();
  object_shader["transform"] = transform;
  cubes_.bind();
  cubes_.bindAttribute(&object_shader,"position",3);
  cubes_.draw();
}


void Scene::paintGL() {
  glBindWindow(width(),height());
  Render(width(),height(),-1);
}

void Scene::keyPressEvent(QKeyEvent* e) {
  if( e->key() == Qt::Key_W && walk_ < 1 ) walk_++;
  if( e->key() == Qt::Key_A && strafe_ > -1 ) strafe_--;
  if( e->key() == Qt::Key_S && walk_ > -1 ) walk_--;
  if( e->key() == Qt::Key_D && strafe_ < 1 ) strafe_++;
  if( e->key() == Qt::Key_Control && jump_ > -1 ) jump_--;
  if( e->key() == Qt::Key_Space && jump_ < 1 ) jump_++;
  if(!timer_.isActive()) timer_.start(16,this);
}

void Scene::keyReleaseEvent(QKeyEvent* e) {
  if( e->key() == Qt::Key_W ) walk_--;
  if( e->key() == Qt::Key_A ) strafe_++;
  if( e->key() == Qt::Key_S ) walk_++;
  if( e->key() == Qt::Key_D ) strafe_--;
  if( e->key() == Qt::Key_Control ) jump_++;
  if( e->key() == Qt::Key_Space ) jump_--;
}

void Scene::mousePressEvent(QMouseEvent* e) {
  drag_=e->pos();
}

//from "An Efficient and Robust Ray-Box Intersection Algorithm"
static bool intersect(vec3 min, vec3 max, vec3 O, vec3 D, float& t) {
    if(O>min && O<max) return true;
    float tmin = ((D.x >= 0?min:max).x - O.x) / D.x;
    float tmax = ((D.x >= 0?max:min).x - O.x) / D.x;
    float tymin= ((D.y >= 0?min:max).y - O.y) / D.y;
    float tymax= ((D.y >= 0?max:min).y - O.y) / D.y;
    if( (tmin > tymax) || (tymin > tmax) ) return false;
    if(tymin>tmin) tmin=tymin; if(tymax<tmax) tmax=tymax;
    float tzmin= ((D.z >= 0?min:max).z - O.z) / D.z;
    float tzmax= ((D.z >= 0?max:min).z - O.z) / D.z;
    if( (tmin > tzmax) || (tzmin > tmax) ) return false;
    if(tzmin>tmin) tmin=tzmin; if(tzmax<tmax) tmax=tzmax;
    if(tmax <= 0) return false; t=tmax;/*tmax is nearest point*/ return true;
}

void Scene::mouseReleaseEvent(QMouseEvent* e) {
  if(grab_) {
    setCursor(QCursor());
    grab_=false;
  } else {
    mat4 transform = projection_;
    transform.rotateX(-pitch_);
    transform.rotateZ(-yaw_);
    //FIXME: projection seems wrong, selection is closer to screen center than cursor.
    vec3 d = transform.inverse() * normalize(vec3(2.0*e->x()/width()-1,1-2.0*e->y()/height(),1));
    float min_distance=1;
    int hit_track=-1,hit_image=-1,hit_object=-1;
    foreach(Point point, reconstruction_->AllPoints()) {
      vec3 o = vec3(point.X.x(),point.X.y(),point.X.z())-position_;
      double t = dot(d,o) / dot(d,d);
      if (t < 0) continue;
      vec3 p = t * d;
      if( length(p-o) < min_distance ) {
        min_distance = length(p-o);
        hit_track = point.track;
      }
    }
    foreach(Camera camera, reconstruction_->AllCameras()) {
      vec3 o = vec3(camera.t.x(),camera.t.y(),camera.t.z())-position_;
      double t = dot(d,o) / dot(d,d);
      if (t < 0) continue;
      vec3 p = t * d;
      if( length(p-o) < min_distance ) {
        min_distance = length(p-o);
        hit_track = -1; hit_image=camera.image;
      }
    }
    float minZ=16384; int i=0;
    foreach(Object object, objects_) {
      vec3 min,max;
      object.position(reconstruction_,&min,&max);
      float z=0;
      if( intersect(min,max,position_,d,z) && z < minZ ) {
        minZ = z;
        hit_track = -1; hit_image=-1; hit_object=i;
      }
      i++;
    }
    if(hit_track>=0) {
      if(selected_tracks_.contains(hit_track)) {
        selected_tracks_.remove(selected_tracks_.indexOf(hit_track));
      } else {
        selected_tracks_ << hit_track;
      }
      emit trackChanged(selected_tracks_);
    } else {
      selected_tracks_.clear();
      current_image_=-1;
      active_object_=0;
    }
    if(hit_image>=0) {
      current_image_=hit_image;
      emit imageChanged(hit_image);
    }
    if(hit_object>=0) {
      active_object_ = &objects_[hit_object];
      if(selected_tracks_.isEmpty()) {
        selected_tracks_ = active_object_->tracks;
      }
    }
    upload();
  }
}
void Scene::mouseMoveEvent(QMouseEvent *e) {
  if(!grab_) {
    if((drag_-e->pos()).manhattanLength()<16) return;
    grab_=true;
    setCursor(QCursor(Qt::BlankCursor));
    QCursor::setPos(mapToGlobal(QPoint(width()/2,height()/2)));
    return;
  }
  QPoint delta = e->pos()-QPoint(width()/2,height()/2);
  yaw_=yaw_-delta.x()*PI/width();
  pitch_=qBound(0.0,pitch_-delta.y()*PI/width(),PI);
  QCursor::setPos(mapToGlobal(QPoint(width()/2,height()/2)));
  if(!timer_.isActive()) update();
}
void Scene::timerEvent(QTimerEvent*) {
  mat4 view; view.rotateZ(yaw_); view.rotateX(pitch_);
  velocity_ += view*vec3(strafe_*speed_,0,-walk_*speed_)+vec3(0,0,jump_*speed_);
  velocity_ *= 31.0/32; position_ += velocity_;
  if( length(velocity_) < 0.01 ) timer_.stop();
  update();
}
