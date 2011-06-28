ADD_LIBRARY(libmv_test_main testing_main.cc)
TARGET_LINK_LIBRARIES(libmv_test_main gtest gflags glog )

# make the name of debug libraries end in _d.
SET_TARGET_PROPERTIES(libmv_test_main PROPERTIES DEBUG_POSTFIX "_d")
