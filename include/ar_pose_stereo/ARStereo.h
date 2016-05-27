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

class ARStereo{
private:
    
    // ROS Parameter
    ros::NodeHandle _nh;
    const std::string _cameraImageLeftTopic = "/camera/image_raw";
    const std::string _cameraInfoLeftTopic  = "/camera/camera_info";
    const std::string _cameraImageRightTopic = "/camera/right/image_raw";
    const std::string _cameraInfoRightTopic  = "/camera/right/camera_info";
    static const float UnitAR2ROS= 0.001;
    image_transport::ImageTransport _it;
    image_transport::Publisher _pub;
    image_transport::Subscriber _sub_il;
    image_transport::Subscriber _sub_ir;
    ros::Subscriber _sub_cil;
    ros::Subscriber _sub_cir;
    bool _il_received;
    bool _ir_received;
    bool _cil_received;
    bool _cir_received;
    sensor_msgs::CameraInfo _cam_info_left_ros;
    sensor_msgs::CameraInfo _cam_info_right_ros;
    cv_bridge::CvImagePtr _capture_left;
    cv_bridge::CvImagePtr _capture_right;
    bool _init=false;
    tf::TransformBroadcaster _tf_br;
    
    //ARToolkit parameter
    ARParam _cam_param_left_art;
    ARParam _cam_param_right_art;
    
    // Markers.
    ARMarkerSquare *markersSquare = NULL;
    int markersSquareCount = 0;

// Marker detection.
    ARHandle		*gARHandleL = NULL;
    ARHandle		*gARHandleR = NULL;
    long			 gCallCountMarkerDetect = 0;
    ARPattHandle	*gARPattHandle = NULL;
    int           gARPattDetectionMode;
    
    //file path to marker -> maybe replace through ros parameter
    const std::string markerConfigDataFilename = "/home/tman/catkin_ws/src/ar_pose_stereo/Data/markers.dat";
    const std::string objectDataFilename = "/home/tman/catkin_ws/src/ar_pose_stereo/Data/objects.dat";
    
    // Transformation matrix retrieval.
    AR3DHandle	*gAR3DHandleL = NULL;
    AR3DHandle	*gAR3DHandleR = NULL;
    AR3DStereoHandle	*gAR3DStereoHandle = NULL;
    ARdouble      transL2R[3][4];
    ARdouble      transR2L[3][4];
    
    //image
    ARUint8		*gARTImageL = NULL;
    ARUint8		*gARTImageR = NULL;
    
    //Camera Parameter
    ARParamLT *gCparamLTL = NULL;
    ARParamLT *gCparamLTR = NULL;
    
public:
    
    
    
    ARStereo(ros::NodeHandle & nh);
    
    ~ARStereo();
  
    void imageLeftCallback(const sensor_msgs::ImageConstPtr& incoming_img);
    void cameraInfoLeftCallback(const sensor_msgs::CameraInfoConstPtr &);
    void imageRightCallback(const sensor_msgs::ImageConstPtr& incoming_img);
    void cameraInfoRightCallback(const sensor_msgs::CameraInfoConstPtr &);
    void ARInit();
    void mainLoop();
};
    
#endif /* ARSTEREO_H */

