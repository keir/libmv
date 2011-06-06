FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})
SET(SOURCES tracker.cc klt.cc)
QT4_WRAP_CPP(SOURCES tracker.h)
ADD_EXECUTABLE(qt-tracker ${SOURCES})
TARGET_LINK_LIBRARIES(qt-tracker
                      ${QT_QTCORE_LIBRARY}
                      ${QT_QTGUI_LIBRARY}
                      simple_pipeline
                      tracking
                      correspondence
                      image
                      gflags
                      glog
                      flann
                      reconstruction)
LIBMV_INSTALL_EXE(qt-tracker)
