HEADERS += tracker.h
SOURCES += tracker.cc
INCLUDEPATH += ../..
INCLUDEPATH += /usr/include/eigen3/
INCLUDEPATH += ../../third_party/glog/src

#LIBS += -L../../../bin-dbg/lib/ -limage_d -ltracking_d -lsimple_pipeline_d
LIBS += -L../../../bin-opt/lib/ -limage -ltracking -lsimple_pipeline
LIBS += -lglog -lgflags -ljpeg

profiler {
 LIBS += -lprofiler
 DEFINES += PPROF
}

OBJECTS_DIR=build
MOC_DIR=build
DESTDIR=build

OTHER_FILES += ../../libmv/correspondence/klt.cc ../../libmv/correspondence/klt.h
OTHER_FILES += ../../libmv/image/image_pyramid.cc ../../libmv/image/image_pyramid.h
OTHER_FILES += ../../libmv/image/convolve.cc ../../libmv/image/convolve.h
