/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   ARStereo.h
 * Author: Thomas Fontana
 *
 * Created on May 24, 2016, 2:05 PM
 */
#ifndef ARSTEREO_H
#define ARSTEREO_H

//ROS includes
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <string.h>
#include <tf/transform_broadcaster.h>
#include <image_geometry/pinhole_camera_model.h>

extern "C" 
{
//ARToolkit includes
#include <stdio.h>
#include <stdlib.h>					// malloc(), free()
#include <GL/glut.h>
#include <AR/config.h>
#include <AR/video.h>
#include <AR/param.h>			// arParamDisp()
#include <AR/ar.h>
#include <AR/gsub_lite.h>
#include <ARMarkerSquare.h>
}

#define SCALE_DEFAULT 0.6
#define SCALE_WAIT_RESET 50
#define ROI_WAIT_RESET 0
#define ROI_REGION_GROW_X 50.0
#define ROI_REGION_GROW_Y 25.0


class ARStereo{
private:
    
    // ROS Parameter
    ros::NodeHandle _nh;
    std::string _cameraImageLeftTopic = "/left/image_raw";
    std::string _cameraInfoLeftNewTopic  = "/left/camera_info_new";
    static const float UnitAR2ROS= 0.001;
    image_transport::ImageTransport _it;
    image_transport::Publisher _pub;
    image_transport::Subscriber _sub_il;
    ros::Subscriber _sub_cil;
    ros::Subscriber _sub_ciln;
    ros::Publisher _pub_ciln;
    bool _il_received;
    bool _cil_received;
    bool _ciln_received;
    sensor_msgs::CameraInfo _cam_info_left_ros;
    image_geometry::PinholeCameraModel _cam_model_left;
    cv_bridge::CvImagePtr _capture_left;
    ros::Time _capture_time_left;
    ros::Time _capture_time_prev;
    tf::Transform tf_prev;
    bool _init=false;
    tf::TransformBroadcaster _tf_br;

    
    //ARToolkit parameter
    ARParam _cam_param_left_art;

    // Markers.
    ARMarkerSquare *markersSquare = NULL;
    int markersSquareCount = 0;

// Marker detection.
    ARHandle		*gARHandleL = NULL;
    long			 gCallCountMarkerDetect = 0;
    ARPattHandle	*gARPattHandle = NULL;
    int           gARPattDetectionMode;
    int selected_marker=0; //holds the status of the marker which was selected to track. right now it is hardcoded for the first element.
    float image_scale=SCALE_DEFAULT; //parameter stores the current scaling of the image
    int marker_size_max = 40; //maximum pixel size of marker
    int marker_roi_x = 70;
    int marker_roi_y = 70;


    //file path to marker -> maybe replace through ros parameter
    const std::string markerConfigDataFilename = "/home/tman/ros_ws/src/ar_pose_stereo/Data/markers.dat";
    const std::string objectDataFilename = "/home/tman/ros_ws/src/ar_pose_stereo/Data/objects.dat";
    
    // Transformation matrix retrieval.
    AR3DHandle	*gAR3DHandleL = NULL;
    ARdouble      transL2R[3][4];
    ARdouble      transR2L[3][4];
    
    //image
    ARUint8		*gARTImageL = NULL;

    //Camera Parameter
    ARParamLT *gCparamLTL = NULL;

public:
    
    
    
    ARStereo(ros::NodeHandle & nh);
    
    ~ARStereo();

    void updateCameraInfo(const sensor_msgs::CameraInfo &);
    void imageLeftCallback(const sensor_msgs::ImageConstPtr& incoming_img);
    void cameraInfoLeftCallback(const sensor_msgs::CameraInfoConstPtr &);
    void arParamUpdate(ARHandle* handle, ARParam *param);
    void ARInit();
    void safeMarker(ARMarkerInfo *target);
    void mainLoop();

};
    
#endif /* ARSTEREO_H */

