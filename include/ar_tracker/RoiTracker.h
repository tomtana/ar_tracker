//
// Created by tman on 5/23/17.
//



#ifndef PROJECT_ROITRACKER_H
#define PROJECT_ROITRACKER_H

#include <ros/ros.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf/transform_broadcaster.h>

class RoiTracker{
private:
    ros::Time time_call_last;
    tf::Vector3 tf_prev;
    image_geometry::PinholeCameraModel cam_model;

public:
    RoiTracker();
    void fromCameraInfo(sensor_msgs::CameraInfo camera_info);
    sensor_msgs::CameraInfo getCameraInfo();
    image_geometry::PinholeCameraModel getCameraModel();
    bool updateRoi(tf::Vector3 pos3d, int roi_width, int roi_height, double marker_size, ros::Time now, bool predict=true, float dt_max=0.3);
    bool enlargeRoi(int step_x=50,int step_y=25, int every_n_iter=0);
};

#endif //PROJECT_ROITRACKER_H

