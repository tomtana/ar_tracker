/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   ARTracker.h
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

//Own Classes
#include <RoiTracker.h>

extern "C" 
{
//ARToolkit includes
#include <stdio.h>
#include <stdlib.h>
#include <GL/glut.h>
#include <AR/config.h>
#include <AR/video.h>
#include <AR/param.h>
#include <AR/ar.h>
#include <AR/gsub_lite.h>
#include <ARMarkerSquare.h>
}

#define SCALE_DEFAULT 0.6
#define SCALE_WAIT_RESET 50
#define ROI_WAIT_RESET 0
#define ROI_REGION_GROW_X 50.0
#define ROI_REGION_GROW_Y 25.0


class ARTracker{
private:
    
    /// ROS
    ros::NodeHandle _nh; ///node handle
    static const float UnitAR2ROS= 0.001;
    image_transport::ImageTransport _it;
    image_transport::Publisher _pub;
    image_transport::Subscriber _sub_il;
    ros::Subscriber _sub_cil;
    ros::Publisher _pub_ciln;
    bool _img_received;
    bool _cam_info_received;
    bool _cam_info_up_to_date;
    sensor_msgs::CameraInfo _cam_info;
    image_geometry::PinholeCameraModel _cam_model;
    cv_bridge::CvImagePtr _capture_left;
    ros::Time _capture_time_left;
    bool _init=false;
    tf::TransformBroadcaster _tf_br;

    ///Own Classes
    RoiTracker _roi_tracker;

    ///ARtoolkit
    //structure holding parameter
    ARParam _ar_cam_param;
    //Infos about Markers.
    ARMarkerSquare *_ar_markers_square = NULL;
    int _markers_square_count = 0;
    // Structures holding information ofr the detection process
    ARHandle		*_ar_handle = NULL;
    long			 _call_count_marker_detect = 0;
    ARPattHandle	*_ar_patt_handle = NULL;
    int           gARPattDetectionMode;
    //file path to marker -> maybe replace through ros parameter
    std::string _ar_marker_config_data_filename;
    //detection settings

    /**
     * Tracking Modi;
     *   AR_USE_TRACKING_HISTORY              0
     *   AR_NOUSE_TRACKING_HISTORY            1
     *   AR_USE_TRACKING_HISTORY_V2           2
     *
     *   default is AR_USE_TRACKING_HISTORY_V2
     */
    int _ar_tracking_mode;

    /**
     * Threshold modi:
     * The way ar toolkit determines the thresholds for the marker detection. The following choices are available
     * AR_LABELING_THRESH_MODE_MANUAL = 0,
     * AR_LABELING_THRESH_MODE_AUTO_MEDIAN =1,
     * AR_LABELING_THRESH_MODE_AUTO_OTSU =2,
     * AR_LABELING_THRESH_MODE_AUTO_ADAPTIVE =3,
     * AR_LABELING_THRESH_MODE_AUTO_BRACKETING=4
     */
    int _ar_threshold_mode;

    /**
     * Ar toolkit image type to use.
     *
     *    AR_TEMPLATE_MATCHING_COLOR               0
     *    AR_TEMPLATE_MATCHING_MONO                1
     *    AR_MATRIX_CODE_DETECTION                 2
     *    AR_TEMPLATE_MATCHING_COLOR_AND_MATRIX    3
     *    AR_TEMPLATE_MATCHING_MONO_AND_MATRIX     4
     *
     *    default is AR_TEMPLATE_MATCHING_MONO
     */
    int ar_pattern_detection_mode;

    int ar_thresh_mode_auto_intervall;

    int _ar_thresh; //threshold for manual thesholding

    ///Settings for detection process
    int _selected_marker=0; //holds the status of the marker which was selected to track. right now it is hardcoded for the first element.
    float image_scale=SCALE_DEFAULT; //parameter stores the current scaling of the image
    int marker_size_max = 40; //maximum pixel size of marker
    int marker_roi_x = 70;
    int marker_roi_y = 70;



    
    // Transformation matrix retrieval.
    AR3DHandle	*gAR3DHandleL = NULL;
    ARdouble      transL2R[3][4];
    ARdouble      transR2L[3][4];


    //Camera Parameter
    ARParamLT *gCparamLTL = NULL;

public:
    
    
    
    ARTracker(ros::NodeHandle & nh);
    
    ~ARTracker();

    void updateCameraInfo(const sensor_msgs::CameraInfo &);
    void imageLeftCallback(const sensor_msgs::ImageConstPtr& incoming_img);
    void cameraInfoLeftCallback(const sensor_msgs::CameraInfoConstPtr &);
    void arParamUpdate(ARHandle* handle, ARParam *param);
    void ARInit();
    void mainLoop();

};
    
#endif /* ARSTEREO_H */

