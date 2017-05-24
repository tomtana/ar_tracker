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
    /**
     * Constructor
     */
    RoiTracker();
    /**
     * initialize RoiTracker with a camera info file
     * @param camera_info camera info
     */
    void fromCameraInfo(sensor_msgs::CameraInfo camera_info);
    /**
     * Returns current camera info file including the current Roi set
     * @return camera info
     */
    sensor_msgs::CameraInfo getCameraInfo();
    /**
     * Returns the camera model which is internally used to track the roi
     * @return camera model
     */
    image_geometry::PinholeCameraModel getCameraModel();
    /**
     * Updates the current roi with the given parameters
     * @param pos3d the current 3d position of the marker
     * @param roi_width =the width_total/2 the roi should have [pxl] :
     * @param roi_height =the height_total/2 the roi should have [pxl]
     * @param marker_size size in [m] of the currently used marker
     * @param now pass the current time
     * @param predict choose if prediction should be enabled or not
     * @param dt_max the maximum delay within a gradient is valid
     * @return
     */
    bool updateRoi(tf::Vector3 pos3d, int roi_width, int roi_height, double marker_size, ros::Time now, bool predict=true, float dt_max=0.3);
    /**
     * Enlarges the current roi with the given parameters.
     * @param step_x roi increases in each direction with step_x
     * @param step_y roi increases in each direction with step_y
     * @param every_n_iter enlarge only every every_n_iter interation
     * @return
     */
    bool enlargeRoi(int step_x=50,int step_y=25, int every_n_iter=0);
};

#endif //PROJECT_ROITRACKER_H

