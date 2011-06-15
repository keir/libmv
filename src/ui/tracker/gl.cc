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

#include "ui/tracker/gl.h"

#include <QDebug>
#include <QImage>
#include <QFile>
#include <QStringList>

#ifdef GLEW
#include "GL/glew.h"
#else
#define GL_GLEXT_PROTOTYPES
#include "GL/gl.h"
#endif

#ifdef WIN32
#include "GL/wglew.h"
#else
#ifdef GLEW
#include "GL/glxew.h"
#else
extern "C" int glXSwapIntervalSGI(int interval);
#endif
#endif

void glInitialize() {
#ifdef GLEW
  glewInit();
#endif
#ifdef WIN32
  wglSwapIntervalEXT(1);
#else
#if 0
  glXSwapIntervalSGI(1);
#endif
#endif
}

void GLUniform::operator=( int v ) { if(id>=0) glUniform1i(id,v); }
void GLUniform::operator=( float v ) { if(id>=0) glUniform1f(id,v); }
void GLUniform::operator=( vec2 v ) { if(id>=0) glUniform2f(id,v.x,v.y); }
void GLUniform::operator=( vec3 v ) { if(id>=0) glUniform3f(id,v.x,v.y,v.z); }
void GLUniform::operator=( vec4 v ) { if(id>=0) glUniform4f(id,v.x,v.y,v.z,v.w); }
void GLUniform::operator=( mat3 m ) { if(id>=0) glUniformMatrix3fv(id,1,0,m.data); }
void GLUniform::operator=( mat4 m ) { if(id>=0) glUniformMatrix4fv(id,1,0,m.data); }
void GLUniform::set(vec3* data,int size) { if(id>=0) glUniform3fv(id,size,(float*)data); }
void GLUniform::set(vec4* data,int size) { if(id>=0) glUniform4fv(id,size,(float*)data); }

bool GLShader::compile(QString vertex, QString fragment) {
  if(!id) id = glCreateProgram();
  QByteArray vertexSource = vertex.toAscii(); const char* vertexString = vertexSource.constData();
  int vertexShader = glCreateShader(GL_VERTEX_SHADER); glShaderSource(vertexShader,1,&vertexString,0);
  glCompileShader(vertexShader); glAttachShader(id,vertexShader);
  {int l=0; glGetShaderiv(vertexShader,GL_INFO_LOG_LENGTH,&l);
    if(l>1) { QByteArray msg(l,0); glGetShaderInfoLog(vertexShader,l,0,msg.data()); qDebug()<<msg; return false; } }
  QByteArray fragmentSource = fragment.toAscii(); const char* fragmentString = fragmentSource.constData();
  int fragmentShader = glCreateShader(GL_FRAGMENT_SHADER); glShaderSource(fragmentShader,1,&fragmentString,0);
  glCompileShader(fragmentShader); glAttachShader(id,fragmentShader);
  {int l=0; glGetShaderiv(fragmentShader,GL_INFO_LOG_LENGTH,&l);
    if(l>1) { QByteArray msg(l,0); glGetShaderInfoLog(fragmentShader,l,0,msg.data()); qDebug()<<msg; return false; } }
  glLinkProgram(id);
  {int l=0; glGetProgramiv(id,GL_INFO_LOG_LENGTH,&l);
    if(l>1) { QByteArray msg(l,0); glGetProgramInfoLog(id,l,0,msg.data()); qDebug()<<msg; return false; } }
  return true;
}
void GLShader::bind() { glUseProgram(id); }
void GLShader::bindSamplers(const char* tex0, const char* tex1, const char* tex2, const char* tex3,
                            const char* tex4, const char* tex5, const char* tex6, const char* tex7) {
  glUniform1i((*this)[tex0].id,0);
  if(tex1) glUniform1i((*this)[tex1].id,1);
  if(tex2) glUniform1i((*this)[tex2].id,2);
  if(tex3) glUniform1i((*this)[tex3].id,3);
  if(tex4) glUniform1i((*this)[tex4].id,4);
  if(tex5) glUniform1i((*this)[tex5].id,5);
  if(tex6) glUniform1i((*this)[tex6].id,6);
  if(tex7) glUniform1i((*this)[tex7].id,7);
}
void GLShader::bindFragments(const char* /*frag0*/, const char* /*frag1*/) {
#ifndef GLEW
#if 0
  glBindFragDataLocationEXT(id,0,frag0);
  if(frag1) glBindFragDataLocationEXT(id,1,frag1);
#endif
#endif
}
int GLShader::attribLocation(const char* name ) {
  int location = attribLocations.value(name,-1);
  if(location<0) attribLocations[name]=location=glGetAttribLocation(id,name);
  return location;
}
GLUniform GLShader::operator[](const char* name) {
  int location = uniformLocations.value(name,-1);
  if(location<0) uniformLocations[name]=location=glGetUniformLocation(id,name);
  return GLUniform(location);
}

void GLBuffer::upload(const void* data, int count) {
  if(!indexBuffer) glGenBuffers(1, &indexBuffer);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexBuffer);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, count*sizeof(uint), data, GL_STATIC_DRAW );
  indexCount = count;
}
void GLBuffer::upload(const void* data, int count, int size) {
  if(!vertexBuffer) glGenBuffers(1, &vertexBuffer);
  glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
  glBufferData(GL_ARRAY_BUFFER, count*size, data, GL_STATIC_DRAW );
  vertexCount = count;
  vertexSize = size;
}
void GLBuffer::bind() { glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer); glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,0); }
void GLBuffer::bindAttribute(GLShader* program, const char* name, int elementSize, size_t offset) {
  int location = program->attribLocation(name);
  if(location<0) { qWarning()<<"unused attrib"<<name; return; }
  glVertexAttribPointer(location, elementSize, GL_FLOAT, 0, vertexSize, (void*)offset);
  glEnableVertexAttribArray(location);
}
void GLBuffer::draw() {
  uint mode[] = { 0, GL_POINTS, GL_LINES, GL_TRIANGLES, GL_QUADS };
  if(primitiveType==1) {
    glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
    glEnable(GL_POINT_SPRITE);
  }
  if(indexBuffer) {
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexBuffer);
    glDrawElements(mode[primitiveType],indexCount,GL_UNSIGNED_INT,0);
  } else {
    glDrawArrays(mode[primitiveType],0,vertexCount);
  }
}

void GLTexture::upload(QImage image,bool mipmap) {
  if(!id) glGenTextures(1, &id);
  glBindTexture(GL_TEXTURE_2D, id);
  glTexImage2D(GL_TEXTURE_2D,0,image.depth()/8,width=image.width(),height=image.height(),
               0,GL_BGRA,GL_UNSIGNED_BYTE,image.constBits());
  if(mipmap) {
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glGenerateMipmap(GL_TEXTURE_2D);
  } else {
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  }
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, 2.0);
}
void GLTexture::allocate(int width, int height, int format) {
  this->width=width; this->height=height;
  if(!id) glGenTextures(1, &id);
  glBindTexture(GL_TEXTURE_2D, id);
  if(format&Shadow) {
    glTexImage2D(GL_TEXTURE_2D,0,GL_DEPTH_COMPONENT16,width,height,0,GL_DEPTH_COMPONENT,GL_UNSIGNED_INT,0);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_R_TO_TEXTURE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_GEQUAL );
  } else if(format&Depth) glTexImage2D(GL_TEXTURE_2D,0,GL_DEPTH_COMPONENT32,width,height,0,GL_DEPTH_COMPONENT,GL_UNSIGNED_INT,0);
  else if(format&Gamma) glTexImage2D(GL_TEXTURE_2D,0,GL_SRGB8,width,height,0,GL_SRGB,GL_UNSIGNED_BYTE,0);
  else if(format&Float) glTexImage2D(GL_TEXTURE_2D,0,GL_RGBA16F,width,height,0,GL_RGBA,GL_UNSIGNED_BYTE,0);
  else glTexImage2D(GL_TEXTURE_2D,0,GL_RGBA8,width,height,0,GL_BGRA,GL_UNSIGNED_BYTE,0);
  if(format&Bilinear) {
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, format&Mipmap?GL_LINEAR_MIPMAP_LINEAR:GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  } else {
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, format&Mipmap?GL_LINEAR_MIPMAP_NEAREST:GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  }
  if(format&Anisotropic) glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, 2.0);
  if(format&Clamp) {
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  }
}
GLTexture::~GLTexture() {
  if(id) glDeleteTextures(1,&id);
}
void GLTexture::bind(int sampler) {
  glActiveTexture(GL_TEXTURE0+sampler);
  glBindTexture(GL_TEXTURE_2D, id);
}
void GLTexture::generateMipmap() {
  glBindTexture(GL_TEXTURE_2D, id);
  glGenerateMipmap(GL_TEXTURE_2D);
}
void GLTexture::bindSamplers(GLTexture tex0, GLTexture tex1, GLTexture tex2, GLTexture tex3) {
  tex0.bind(0); tex1.bind(1); tex2.bind(2); tex3.bind(3);
}

void GLFrameBuffer::attach(GLTexture depth, GLTexture color0, GLTexture color1) {
  this->depth=depth; this->color0=color0; this->color1=color1;
  if(!id) glGenFramebuffers(1,&id);
  glBindFramebuffer(GL_FRAMEBUFFER,id);
  if(depth.id) glFramebufferTexture2D(GL_FRAMEBUFFER,GL_DEPTH_ATTACHMENT,GL_TEXTURE_2D,depth.id,0);
  if(color0.id) glFramebufferTexture2D(GL_FRAMEBUFFER,GL_COLOR_ATTACHMENT0,GL_TEXTURE_2D,color0.id,0);
  if(color1.id) glFramebufferTexture2D(GL_FRAMEBUFFER,GL_COLOR_ATTACHMENT1,GL_TEXTURE_2D,color1.id,0);
}
GLFrameBuffer::~GLFrameBuffer() {
  if(id) glDeleteFramebuffers(1,&id);
  if(pbo) glDeleteBuffers(1,&pbo);
}
void GLFrameBuffer::bind(bool clear) {
  glBindFramebuffer(GL_DRAW_FRAMEBUFFER,id);
  if(depth.id) glViewport(0,0,depth.width,depth.height);
  else glViewport(0,0,color0.width,color0.height);
  GLenum buffers[]={GL_COLOR_ATTACHMENT0,GL_COLOR_ATTACHMENT1};
  glDrawBuffers(color1.id?2:color0.id?1:0,buffers);
  glDepthMask(depthWrite);
  if(clear) glClear( (depthWrite?GL_DEPTH_BUFFER_BIT:0) | (color0.id?GL_COLOR_BUFFER_BIT:0) );
}
void GLFrameBuffer::bindSamplers() { GLTexture::bindSamplers(depth,color0,color1); }
void GLFrameBuffer::bindWindow(int w, int h) {
  glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
  glViewport(0,0,w,h);
  {GLenum buffers[]={GL_FRONT};glDrawBuffers(1, buffers);}
  glClear(GL_DEPTH_BUFFER_BIT|GL_COLOR_BUFFER_BIT);
}
const uchar* GLFrameBuffer::map() {
  glBindFramebuffer(GL_READ_FRAMEBUFFER,id);
  glReadBuffer(GL_COLOR_ATTACHMENT0);
  if(!pbo) glGenBuffers(1,&pbo);
  glBindBuffer(GL_PIXEL_PACK_BUFFER,pbo);
  glBufferData(GL_PIXEL_PACK_BUFFER,color0.width*color0.height*4,0,GL_STREAM_READ);
  glReadPixels(0,0,color0.width,color0.height,GL_BGRA,GL_UNSIGNED_BYTE,0);
  return (uchar*)glMapBuffer(GL_PIXEL_PACK_BUFFER, GL_READ_ONLY);
}
void GLFrameBuffer::unmap() {
  glUnmapBuffer(GL_PIXEL_PACK_BUFFER);
  glBindBuffer(GL_PIXEL_PACK_BUFFER,0);
}

static QMap<uint,bool> state;
GLState::operator bool() {
  return state[id];
}
void GLState::operator=(bool bit) {
  if(state[id]!=bit) {
    if(bit) glEnable(id);
    else glDisable(id);
    state[id]=bit;
  }
}

GLState CullFace(GL_CULL_FACE);
GLState DepthTest(GL_DEPTH_TEST);

void glAdditiveBlendMode() {
  glBlendFunc(GL_ONE,GL_ONE);
  glEnable(GL_BLEND);
}

static QString filterGLSL(QFile& file,QStringList tags) {
  QRegExp filter("(\\w*) \\{");
  QString l=file.readLine().simplified();
  if(!filter.exactMatch(l)) return l;
  bool enabled = tags.contains(filter.cap(1));
  QString out;
  for(int nest=0;(l=filterGLSL(file,tags))!="}"||nest>0;) {
    if(l.isEmpty()) continue;
    if(enabled) out+=l+"\n";
    nest += l.count("{")-l.count("}");
  }
  return out;
}
QString glsl(QString tags) {
  QFile file(":/shader.glsl");
  file.open(QFile::ReadOnly);
  QString global,main;
  while(!file.atEnd()) {
    foreach(QString line,filterGLSL(file,tags.split(" ")).split("\n")) {
      if(!line.indexOf(QRegExp("uniform|in|out|(float|vec[1234]|mat[234]) [a-zA-Z0-9]*\\("))) global+=line+"\n";
      else main+=line+"\n";
    }
  }
  return "#version 130\n"+global+"\nvoid main() {\n"+main+"\n}\n";
}
