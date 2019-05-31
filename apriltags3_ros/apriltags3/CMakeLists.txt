cmake_minimum_required(VERSION 2.8.3)
project(apriltags3)

set(CMAKE_C_FLAGS "-std=gnu99 -fPIC -Wall -Wno-unused-parameter -Wno-unused-function -I. -O4 -fno-strict-overflow")

find_package(catkin REQUIRED)
find_package(Threads REQUIRED)
find_package(OpenCV QUIET)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES apriltags3
  CATKIN_DEPENDS
)

include_directories(include include/apriltag/common)
include_directories(include include/apriltag)
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Threads_INCLUDE_DIRS}
)

add_library(apriltags3
  include/apriltag/apriltag.c
  include/apriltag/apriltag_pose.c
  include/apriltag/tag25h9.c
  include/apriltag/tagCircle49h12.c
  include/apriltag/tagStandard52h13.c
  include/apriltag/apriltag_quad_thresh.c
  include/apriltag/tag36h11.c
  include/apriltag/tagCustom48h12.c
  include/apriltag/tag16h5.c
  include/apriltag/tagCircle21h7.c
  include/apriltag/tagStandard41h12.c

  include/apriltag/common/g2d.c
  include/apriltag/common/getopt.c
  include/apriltag/common/homography.c
  include/apriltag/common/image_u8.c
  include/apriltag/common/image_u8x3.c
  include/apriltag/common/image_u8x4.c
  include/apriltag/common/matd.c
  include/apriltag/common/pam.c
  include/apriltag/common/pjpeg.c
  include/apriltag/common/pjpeg-idct.c
  include/apriltag/common/pnm.c
  include/apriltag/common/string_util.c
  include/apriltag/common/svd22.c
  include/apriltag/common/time_util.c
  include/apriltag/common/unionfind.c
  include/apriltag/common/workerpool.c
  include/apriltag/common/zarray.c
  include/apriltag/common/zhash.c
  include/apriltag/common/zmaxheap.c

  )
target_link_libraries(apriltags3
 ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS apriltags3
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
