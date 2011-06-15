FIND_PACKAGE(Qt4 REQUIRED)
SET(QT_USE_QTOPENGL ON)
INCLUDE(${QT_USE_FILE})
SET(SOURCES view.cc gl.cc tracker.cc main.cc)
QT4_ADD_RESOURCES(SOURCES tracker.qrc)
QT4_WRAP_CPP(SOURCES view.h gl.h tracker.h main.h)
ADD_EXECUTABLE(qt-tracker ${SOURCES})
TARGET_LINK_LIBRARIES(qt-tracker
                      ${QT_LIBRARIES}
                      simple_pipeline
                      tracking
                      multiview
                      image
                      gflags
                      glog)
LIBMV_INSTALL_EXE(qt-tracker)
