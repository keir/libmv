# TODO(keir): Update this with the new API.
#ADD_EXECUTABLE(track track.cc)
#TARGET_LINK_LIBRARIES(track image correspondence image gflags glog)
#LIBMV_INSTALL_EXE(track)

IF (BUILD_TESTS)
ADD_EXECUTABLE(experimental experimental.cc)
TARGET_LINK_LIBRARIES(experimental
                      multiview_test_data
                      multiview
                      numeric
                      pthread
                      gflags
                      glog
                      )
LIBMV_INSTALL_EXE(experimental)
ENDIF (BUILD_TESTS)

ADD_EXECUTABLE(tracker tracker.cc)
TARGET_LINK_LIBRARIES(tracker
                      image
                      correspondence
                      numeric
                      multiview
                      flann
                      glog
                      gflags
                      detector
                      descriptor
                      fast
                      daisy
                      reconstruction
                      )
LIBMV_INSTALL_EXE(tracker)

ADD_EXECUTABLE(nViewMatching nViewMatching.cc)
TARGET_LINK_LIBRARIES(nViewMatching
                      correspondence
                      image
                      gflags
                      glog
                      detector
                      fast
                      flann
                      descriptor
                      multiview
                      daisy
                      )
LIBMV_INSTALL_EXE(nViewMatching)

ADD_EXECUTABLE(reconstruct_video reconstruct_video.cc)
TARGET_LINK_LIBRARIES(reconstruct_video
                      image
                      correspondence
                      numeric
                      multiview
                      flann
                      glog
                      gflags
                      detector
                      descriptor
                      fast
                      daisy
                      reconstruction
                      )
LIBMV_INSTALL_EXE(reconstruct_video)

ADD_EXECUTABLE(undistort undistort.cc)
TARGET_LINK_LIBRARIES(undistort
                      image
                      correspondence
                      numeric
                      multiview
                      camera
                      glog
                      gflags
                      flann
                      )
LIBMV_INSTALL_EXE(reconstruct_video)

