FIND_PACKAGE(Qt4 REQUIRED)
SET(QT_USE_QTOPENGL ON)
INCLUDE(${QT_USE_FILE})
find_package(OpenGL)
INCLUDE_DIRECTORIES(${OPENGL_INCLUDE_DIR})
SET(SOURCES main.cc tracker.cc scene.cc gl.cc)
QT4_ADD_RESOURCES(SOURCES tracker.qrc)
QT4_WRAP_CPP(SOURCES main.h tracker.h scene.h)
ADD_EXECUTABLE(qt-tracker ${SOURCES})
TARGET_LINK_LIBRARIES(qt-tracker
                      ${QT_LIBRARIES}
                      ${OPENGL_LIBRARIES}
                      simple_pipeline
                      tracking
                      multiview
                      image)
LIBMV_INSTALL_EXE(qt-tracker)
