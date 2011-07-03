QT += opengl
HEADERS += scene.h gl.h tracker.h main.h
SOURCES += scene.cc gl.cc tracker.cc main.cc
OTHER_FILES += shader.glsl
RESOURCES = tracker.qrc
INCLUDEPATH += ../..
INCLUDEPATH += /usr/include/eigen3/
#LIBS += -L../../../bin-dbg/lib/ -lsimple_pipeline_d -limage_d -ltracking_d -lmultiview_d -lglog
LIBS += -L../../../bin-opt/lib/ -lsimple_pipeline -limage -ltracking -lmultiview

win32:CONFIG+=glew
exists(/usr/include/GL/glew.h):CONFIG+=glew
glew {
 DEFINES += GLEW
 win32 {
  HEADERS += GL/glew.h GL/wglew.h
  SOURCES += glew.c
  DEFINES += GLEW_STATIC
 }
 unix {
  LIBS += -lGLEW
 }
}

profiler {
 LIBS += -lprofiler
 DEFINES += PPROF
}

OBJECTS_DIR=build
MOC_DIR=build
RCC_DIR=build
DESTDIR=build

OTHER_FILES += CMakeLists.txt ../../libmv/simple_pipeline/CMakeLists.txt
OTHER_FILES += ../../libmv/image/image_pyramid.cc ../../libmv/image/image_pyramid.h
OTHER_FILES += ../../libmv/image/convolve.cc ../../libmv/image/convolve.h
OTHER_FILES += ../../libmv/simple_pipeline/*.h ../../libmv/simple_pipeline/*.cc
OTHER_FILES += ../../libmv/tracking/*.h ../../libmv/tracking/*.cc
OTHER_FILES += ../../libmv/multiview/fundamental.cc ../../libmv/multiview/projection.cc
