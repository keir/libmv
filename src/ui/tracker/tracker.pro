HEADERS += tracker.h main.h
SOURCES += tracker.cc main.cc
RESOURCES = tracker.qrc
INCLUDEPATH += ../..
INCLUDEPATH += /usr/include/eigen3/
INCLUDEPATH += ../../third_party/glog/src

CONFIG(debug, debug|release) {
 LIBS += -L../../../bin-dbg/lib/ -limage_d -ltracking_d -lsimple_pipeline_d -lglog -lgflags
} else {
 LIBS += -L../../../bin-opt/lib/ -limage -ltracking -lsimple_pipeline
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
