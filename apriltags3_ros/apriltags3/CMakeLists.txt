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

include_directories(include include/apriltags/common)
include_directories(include include/apriltags)
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Threads_INCLUDE_DIRS}
)

add_library(apriltags3
  include/apriltags/apriltag.c
  include/apriltags/apriltag_pose.c
  include/apriltags/tag25h9.c
  include/apriltags/tagCircle49h12.c
  include/apriltags/tagStandard52h13.c
  include/apriltags/apriltag_quad_thresh.c
  include/apriltags/tag36h11.c
  include/apriltags/tagCustom48h12.c
  include/apriltags/tag16h5.c
  include/apriltags/tagCircle21h7.c
  include/apriltags/tagStandard41h12.c

  include/apriltags/common/g2d.c
  include/apriltags/common/getopt.c
  include/apriltags/common/homography.c
  include/apriltags/common/image_u8.c
  include/apriltags/common/image_u8x3.c
  include/apriltags/common/image_u8x4.c
  include/apriltags/common/matd.c
  include/apriltags/common/pam.c
  include/apriltags/common/pjpeg.c
  include/apriltags/common/pjpeg-idct.c
  include/apriltags/common/pnm.c
  include/apriltags/common/string_util.c
  include/apriltags/common/svd22.c
  include/apriltags/common/time_util.c
  include/apriltags/common/unionfind.c
  include/apriltags/common/workerpool.c
  include/apriltags/common/zarray.c
  include/apriltags/common/zhash.c
  include/apriltags/common/zmaxheap.c

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
