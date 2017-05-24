cmake_minimum_required(VERSION 2.8.3)
project(ar_tracker)

find_package(
        catkin REQUIRED COMPONENTS
        cv_bridge
        image_transport
        roscpp
        sensor_msgs
        image_geometry
        tf
)

find_package(OpenCV)


catkin_package(
        INCLUDE_DIRS include/ar_tracker ARToolkit/include include
        LIBRARIES ${PROJECT_NAME} ARToolkit/lib
        CATKIN_DEPENDS
)
include_directories(
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/include/ar_tracker
        ${PROJECT_SOURCE_DIR}/ARToolkit/include
        ARToolkit/lib
        include
        ${PROJECT_SOURCE_DIR}/src
        ${PROJECT_SOURCE_DIR}/include
)

link_directories(${PROJECT_SOURCE_DIR}/ARToolkit/lib)

add_executable(
        ar_tracker_node
        src/ar_tracker.cpp
        src/ARTracker.cpp
        src/ARMarkerSquare.c
        src/RoiTracker.cpp)

target_link_libraries(
        ar_tracker_node
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ARgsub_lite
        ARvideo
        AR
        AR2
        ARICP
        ARUtil
        AR
        ARgsub_lite
        ARvideo
        AR
        AR2
        ARICP
        ARUtil
)

install(TARGETS ar_tracker_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
