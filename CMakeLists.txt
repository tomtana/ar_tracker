cmake_minimum_required(VERSION 2.8.3)
project(ar_pose_stereo)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  tf
)

find_package(OpenCV)


catkin_package(
  INCLUDE_DIRS include/ar_pose_stereo ARToolkit/include
  LIBRARIES ${PROJECT_NAME} ARToolkit/lib
  CATKIN_DEPENDS
)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include/ar_pose_stereo
  ${PROJECT_SOURCE_DIR}/ARToolkit/include
  ARToolkit/lib
)

link_directories(${PROJECT_SOURCE_DIR}/ARToolkit/lib)
add_library(ARStereo SHARED src/ARStereo.cpp src/ARMarkerSquare.c)
target_link_libraries(ARStereo ${catkin_LIBRARIES} ARgsub_lite ARvideo AR AR2 ARICP ARUtil)

add_executable(ar_pose_stereo src/ar_pose_stereo.cpp)
target_link_libraries(ar_pose_stereo ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ARgsub_lite ARvideo AR AR2 ARICP ARUtil ARStereo AR)

install(TARGETS ar_pose_stereo
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
