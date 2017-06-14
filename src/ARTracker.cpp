#include <ARTracker.h>
#include <opencv2/highgui/highgui.hpp>
#include <boost/filesystem.hpp>
#include <regex>
#include <cursesf.h>

namespace fs = ::boost::filesystem;


ARTracker::ARTracker(ros::NodeHandle & nh):
        _nh(nh),
        _it(_nh),
        _img_received(false),
        _cam_info_received(false),
        _cam_info_up_to_date(true),
        _call_count_marker_detect(0),
        _markers_square_count(0)
{
    std::string topic_image_raw; //image topic

    //read param from ros
    _nh.param<std::string>(ros::this_node::getName()+"/camera",topic_image_raw,"/left"); //image topic
    _nh.param<std::string>(ros::this_node::getName()+"/marker_file", _ar_marker_config_data_filename,"../../../src/ar_tracker/Data/markers.dat"); //path to marker config file
    _nh.param<int>(ros::this_node::getName()+"/ar_tracking_mode", _ar_tracking_mode, AR_USE_TRACKING_HISTORY_V2);
    _nh.param<int>(ros::this_node::getName()+"/ar_thresh_mode", _ar_thresh_mode, AR_LABELING_THRESH_MODE_AUTO_ADAPTIVE);
    _nh.param<int>(ros::this_node::getName()+"/ar_thresh_auto_inv", ar_thresh_mode_auto_intervall, 10);
    _nh.param<int>(ros::this_node::getName()+"/ar_thresh", _ar_thresh, 100);
    _nh.param<int>(ros::this_node::getName()+"/ar_pattern_detection_mode", _ar_pattern_detection_mode, AR_TEMPLATE_MATCHING_MONO);

    _nh.param<int>(ros::this_node::getName()+"/marker_to_track", _marker_to_track, 0);
    _nh.param<double>(ros::this_node::getName()+"/scale_default", _scale_default, 0.6);
    _nh.param<int>(ros::this_node::getName()+"/marker_size_max", _marker_size_max, 40);
    _nh.param<int>(ros::this_node::getName()+"/scale_wait_reset",_scale_wait_reset , 50);
    _nh.param<int>(ros::this_node::getName()+"/roi_region_grow_x", _roi_region_grow_x, 50);
    _nh.param<int>(ros::this_node::getName()+"/roi_region_grow_y", _roi_region_grow_y, 25);

    _nh.param<int>(ros::this_node::getName()+"/roi_wait_reset", _roi_wait_reset, 0);
    _nh.param<int>(ros::this_node::getName()+"/roi_height", _roi_height, 70);
    _nh.param<int>(ros::this_node::getName()+"/roi_width", _roi_width, 70);
    _nh.param<bool>(ros::this_node::getName()+"/show_image_window", _show_image_window, true);
    _nh.param<bool>(ros::this_node::getName()+"/predict_roi", _predict_roi, true);
    _nh.param<double>(ros::this_node::getName()+"/predict_roi_dt", _predict_roi_dt, 0.3);

    //set current scale to scale default value
    _image_scale=(float)_scale_default;


    //init all parameters and so on and so forth
    //_pub = _it.advertise("test", 1);
    ROS_INFO("\n\nLOADED PARAMETERS:");
    ROS_INFO("camera_info: \t\t\t%s/image_raw", topic_image_raw.data());
    ROS_INFO("image_raw: \t\t\t%s/camera_info", topic_image_raw.data());
    _sub_il = _it.subscribe(topic_image_raw+"/image_raw", 1, &ARTracker::imageLeftCallback, this);
    _sub_cil = _nh.subscribe(topic_image_raw +"/camera_info",1,&ARTracker::cameraInfoLeftCallback ,this);

    ROS_INFO("AR_LABELING_WORK_SIZE: \t\t%d",AR_LABELING_WORK_SIZE);
    ROS_INFO("marker_file: \t\t\t%s",_ar_marker_config_data_filename.data());
    ROS_INFO("ar_tracking_mode: \t\t%d",_ar_tracking_mode);
    ROS_INFO("ar_thresh_mode: \t\t%d",_ar_thresh_mode);
    ROS_INFO("ar_thresh_auto_inv: \t\t%d",ar_thresh_mode_auto_intervall);
    ROS_INFO("ar_thresh: \t\t\t%d",_ar_thresh);
    ROS_INFO("ar_pattern_detection_mode: \t\t%d",_ar_pattern_detection_mode);
    ROS_INFO("marker_to_track: \t\t%d",_marker_to_track);
    ROS_INFO("scale_default: \t\t\t%.2f",_scale_default);
    ROS_INFO("marker_size_max: \t\t%d",_marker_size_max);
    ROS_INFO("scale_wait_reset: \t\t%d",_scale_wait_reset);
    ROS_INFO("roi_region_grow_x: \t\t%d",_roi_region_grow_x);
    ROS_INFO("roi_region_grow_y: \t\t%d",_roi_region_grow_y);
    ROS_INFO("roi_wait_reset: \t\t%d",_roi_wait_reset);
    ROS_INFO("roi_height: \t\t\t%d",_roi_height);
    ROS_INFO("roi_width: \t\t\t%d",_roi_width);
    ROS_INFO("show_image_window: \t\t%d",_show_image_window);
    ROS_INFO("predict_roi: \t\t\t%d",_predict_roi);
    ROS_INFO("predict_roi_dt: \t\t%.2f",_predict_roi_dt);

    //check if marker file exists
    if (FILE *file = fopen(_ar_marker_config_data_filename.c_str(), "r")) {
        fclose(file);
    } else {
        ROS_ERROR("Marker file doesnt exist: %s", _ar_marker_config_data_filename.c_str());
        exit(-1);
    }

}

void ARTracker::ARInit(){


    ROS_INFO_ONCE("Initializing ARToolkit..");

    if ((gCparamLTL = arParamLTCreate(&_ar_cam_param, AR_PARAM_LT_DEFAULT_OFFSET)) == NULL) {
        ROS_ERROR("arParamLTCreate: Error: arParamLTCreate.\n");
    }

    _ar_patt_handle = arPattCreateHandle();
	if (!_ar_patt_handle) {
		ROS_ERROR("arPattCreateHandle: Error creating pattern handle.");
		exit(-1);
	}

    _ar_handle = arCreateHandle(gCparamLTL);
    if (!_ar_handle) {
        ROS_ERROR("arCreateHandle: Error creating AR handle.\n");
		exit(-1);
    }
    arPattAttach(_ar_handle, _ar_patt_handle);

    if (arSetPixelFormat(_ar_handle, AR_PIXEL_FORMAT_MONO) < 0) {
        ROS_ERROR("arSetPixelFormat: Error setting pixel format.\n");
		exit(-1);
    }

    gAR3DHandleL = ar3DCreateHandle(&gCparamLTL->param);
    if (!gAR3DHandleL ) {
        ROS_ERROR("ar3DCreateHandle: Error creating 3D handle.\n");
		exit(-1);
    }

    arUtilMatInv((const ARdouble (*)[4])transL2R, transR2L);
    ROS_INFO_ONCE("Left Camera Calibration:",arParamDisp((const ARParam*) &_ar_cam_param));
    ROS_INFO_ONCE("",arParamDisp((const ARParam*) &_ar_cam_param));



    /// Load marker(s).
    newMarkers(_ar_marker_config_data_filename.c_str(), _ar_patt_handle, &_ar_markers_square, &_markers_square_count, &gARPattDetectionMode);
    if(_markers_square_count==0){
        ROS_ERROR("No markers found in config file %s", _ar_marker_config_data_filename.c_str());
        exit(-1);
    }
    ROS_INFO_ONCE("Marker count = %d\n", _markers_square_count);

    ///Configure detection
    arSetMarkerExtractionMode(_ar_handle, _ar_tracking_mode);
    arSetLabelingThreshMode(_ar_handle,static_cast<AR_LABELING_THRESH_MODE>(_ar_thresh_mode));
    arSetLabelingThresh(_ar_handle,_ar_thresh );

    //in case auto thresholding methods are chosen (except the adaptive method) you can specify here the intervall the threshold gets updated
    arSetLabelingThreshModeAutoInterval(_ar_handle,ar_thresh_mode_auto_intervall);
    arSetDebugMode(_ar_handle,AR_DEBUG_DISABLE);
    arSetPatternDetectionMode(_ar_handle, ar_pattern_detection_mode);

    // Other application-wide marker options. Once set, these apply to all markers in use in the application.
    // If you are using standard ARToolKit picture (template) markers, leave commented to use the defaults.
    // If you are usign a different marker design (see http://www.artoolworks.com/support/app/marker.php )
    // then uncomment and edit as instructed by the marker design application.
    //arSetLabelingMode(_ar_handle, AR_LABELING_BLACK_REGION); // Default = AR_LABELING_BLACK_REGION
    //arSetLabelingMode(gARHandleR, AR_LABELING_BLACK_REGION); // Default = AR_LABELING_BLACK_REGION
    //arSetBorderSize(_ar_handle, 0.25f); // Default = 0.25f
    //arSetBorderSize(gARHandleR, 0.25f); // Default = 0.25f
    //arSetMatrixCodeType(_ar_handle, AR_MATRIX_CODE_3x3); // Default = AR_MATRIX_CODE_3x3
    //arSetMatrixCodeType(gARHandleR, AR_MATRIX_CODE_3x3); // Default = AR_MATRIX_CODE_3x3
}

ARTracker::~ARTracker(){
    arPattDetach(_ar_handle);
    arPattDeleteHandle(_ar_patt_handle);
    ar3DDeleteHandle(&gAR3DHandleL);
    arDeleteHandle(_ar_handle);
    arParamLTFree(&gCparamLTL);
    deleteMarkers(&_ar_markers_square, &_markers_square_count, _ar_patt_handle);
}

void ARTracker::updateArParam(ARHandle *handle, ARParam *param) {
    if(arPattDetach(_ar_handle)<0) ROS_ERROR("arPattDetach: no success");
    if(arDeleteHandle(_ar_handle)<0) ROS_ERROR("arDeleteHandle: no success");
    if(ar3DDeleteHandle(&gAR3DHandleL)<0)ROS_ERROR("ar3DDeleteHandle: no success");
    if(arParamLTFree(&gCparamLTL)<0) ROS_ERROR("arParamLTFree: no success");

    if ((gCparamLTL = arParamLTCreate(&_ar_cam_param, AR_PARAM_LT_DEFAULT_OFFSET)) == NULL) {
        ROS_ERROR("setupCamera(): Error: arParamLTCreate.\n");
    }

    _ar_handle = arCreateHandle(gCparamLTL);
    if (!_ar_handle) {
        ROS_ERROR("Error creating AR handle.\n");
    }

    arPattAttach(_ar_handle, _ar_patt_handle);

    if (arSetPixelFormat(_ar_handle, AR_PIXEL_FORMAT_MONO) < 0) {
        ROS_ERROR("Error setting pixel format.\n");
        exit(-1);
    }

    gAR3DHandleL = ar3DCreateHandle(&gCparamLTL->param);

    if (!gAR3DHandleL ) {
        ROS_ERROR("Error creating 3D handle.\n");
        exit(-1);
    }
    arSetMarkerExtractionMode(_ar_handle, _ar_tracking_mode);
    arSetLabelingThreshMode(_ar_handle, static_cast<AR_LABELING_THRESH_MODE>(_ar_thresh_mode));
    arSetLabelingThresh(_ar_handle,_ar_thresh );
    //in case auto thresholding methods are chosen (except the adaptive method) you can specify here the intervall the threshold gets updated
    arSetLabelingThreshModeAutoInterval(_ar_handle,ar_thresh_mode_auto_intervall);
    arSetDebugMode(_ar_handle,AR_DEBUG_DISABLE);
    arSetPatternDetectionMode(_ar_handle, ar_pattern_detection_mode);
}


void ARTracker::imageLeftCallback(const sensor_msgs::ImageConstPtr& incoming_img){
    //check if camera info was received
    if(!_cam_info_received){
        ROS_DEBUG("imageLeftCallback:\twaiting for camera_info");
        return;
    }

    ROS_DEBUG("Image Left received.");

    try
    {
        _capture_left = cv_bridge::toCvCopy (incoming_img, sensor_msgs::image_encodings::MONO8);

    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    //set capture time
    _capture_time_left=incoming_img->header.stamp;

    _img_received=true;

}



void ARTracker::cameraInfoLeftCallback(const sensor_msgs::CameraInfoConstPtr & cam_info){
    ROS_DEBUG("CameraInfo Left received.");
    image_geometry::PinholeCameraModel cam_model_tmp;
    cam_model_tmp.fromCameraInfo(cam_info);
    _roi_tracker.fromCameraInfo(*(cam_info));

    if(_cam_model.rawRoi()==cam_model_tmp.rawRoi() && _cam_model.initialized() && _cam_model.intrinsicMatrix()==cam_model_tmp.intrinsicMatrix() && _cam_info_received==true){
        ROS_DEBUG("cameraInfoLeftCallback: camera_info received but not updated since it has not changed");
        return;
    }else{
        ROS_DEBUG("cameraInfoLeftCallback:\tcamera info has changed.");
        //std::cout<<cam_model_tmp.cameraInfo()<<std::endl;
    }
    //copy new camera info and create pinhole model
    _cam_info = (*cam_info);
    _cam_model=cam_model_tmp;


    //if roi active adjust the intrinsics and update

    memcpy(_ar_cam_param.mat,&_cam_model.intrinsicMatrix()(0,0), 3*sizeof(double));
    memcpy(&_ar_cam_param.mat[1][0],&_cam_model.intrinsicMatrix()(1,0), 3*sizeof(double));
    memcpy(&_ar_cam_param.mat[2][0],&_cam_model.intrinsicMatrix()(2,0), 3*sizeof(double));

    //adjusting the scale
    _ar_cam_param.mat[0][0] *= _image_scale; //fx*scale
    _ar_cam_param.mat[0][2] *= _image_scale; //cx*scale
    _ar_cam_param.mat[1][1] *= _image_scale; //fy*scale
    _ar_cam_param.mat[1][2] *= _image_scale; //cy*scale


    _ar_cam_param.xsize = (int)round(_cam_model.reducedResolution().width*_image_scale);
    _ar_cam_param.ysize = (int)round(_cam_model.reducedResolution().height*_image_scale);


    //set 0 to the last column
    _ar_cam_param.mat[0][3] = 0;
    _ar_cam_param.mat[1][3] = 0;
    _ar_cam_param.mat[2][3] = 0;


    _ar_cam_param.dist_factor[6] = (int)(_cam_model.cx()*_image_scale);//_cam_info.K[2];       // x0 = cX from openCV calibration
    _ar_cam_param.dist_factor[7] = (int)(_cam_model.cy()*_image_scale);//_cam_info.K[5];       // y0 = cY from openCV calibration
    if ( _cam_info.distortion_model == "plumb_bob" && _cam_info.D.size() == 5){
      _ar_cam_param.dist_factor[0]= _cam_info.D[0];  //k0
      _ar_cam_param.dist_factor[1]= _cam_info.D[1];  //k1
      _ar_cam_param.dist_factor[2]= _cam_info.D[2];  //p0
      _ar_cam_param.dist_factor[3]= _cam_info.D[3];  //p1
      _ar_cam_param.dist_factor[4]= _cam_model.fx()*_image_scale; // _cam_info.K[0];  //fx
      _ar_cam_param.dist_factor[5]= _cam_model.fy()*_image_scale;//_cam_info.K[4];  //fy
    }
    else{
      //_ar_cam_param.dist_factor[2] = 0;                  // We don't know the right value, so ignore it
        ROS_ERROR("Camera left: Distortion Parameters from ROS MSG mismatch ARToolkit Model");
    }
    _ar_cam_param.dist_factor[8] = 1.0;                  // scale factor

    _ar_cam_param.dist_function_version=4;

    ROS_INFO_ONCE("Continue with AR initialization..");
    ARInit();
    ROS_INFO_ONCE("ARToolkit initialized");
    _init=true;


    //ROS_INFO("Fake camera_info msg for left camera");
    //cameraInfoRightCallback(cam_info);

    _cam_info_received=true;
    //unsibscribe from topic
    ROS_INFO("Camera Info received. Unsubscribing from topic..");
    _sub_cil.shutdown();
}

void ARTracker::updateCameraInfo(const sensor_msgs::CameraInfo &cam_info){
    ROS_DEBUG("CameraInfo LeftNew received.");
    image_geometry::PinholeCameraModel cam_model_tmp;
    cam_model_tmp.fromCameraInfo(cam_info);

    if(_cam_model.rawRoi()==cam_model_tmp.rawRoi() && _cam_model.initialized() && _cam_model.intrinsicMatrix()==cam_model_tmp.intrinsicMatrix() && _cam_info_up_to_date){
        ROS_DEBUG("updateCameraInfo: camera_info received but not updated since it has not changed");
        return;
    }else{
        ROS_DEBUG("updateCameraInfo:\tcamera info has changed.");
        //std::cout<<cam_model_tmp.cameraInfo()<<std::endl;
    }
    //copy new camera info and create pinhole model
    _cam_info = (cam_info);
    _cam_model=cam_model_tmp;

    ///if roi active adjust the intrinsics and update
    memcpy(_ar_cam_param.mat,&_cam_model.intrinsicMatrix()(0,0), 3*sizeof(double));
    memcpy(&_ar_cam_param.mat[1][0],&_cam_model.intrinsicMatrix()(1,0), 3*sizeof(double));
    memcpy(&_ar_cam_param.mat[2][0],&_cam_model.intrinsicMatrix()(2,0), 3*sizeof(double));

    //adjusting the scale
    _ar_cam_param.mat[0][0] *= _image_scale;
    _ar_cam_param.mat[0][2] *= _image_scale;
    _ar_cam_param.mat[1][1] *= _image_scale;
    _ar_cam_param.mat[1][2] *= _image_scale;

    //compute new image size
    //todo compute always image size which is odd/even.. maybe needed when preprocessing is wanted
    _ar_cam_param.xsize = (int)round(_cam_model.reducedResolution().width*_image_scale);
    _ar_cam_param.ysize = (int)round(_cam_model.reducedResolution().height*_image_scale);

    _ar_cam_param.dist_factor[6] = (int)(_cam_model.cx()*_image_scale);//_cam_info.K[2];       // x0 = cX from openCV calibration
    _ar_cam_param.dist_factor[7] = (int)(_cam_model.cy()*_image_scale);//_cam_info.K[5];       // y0 = cY from openCV calibration

    if ( _cam_info.distortion_model == "plumb_bob" && _cam_info.D.size() == 5){
        _ar_cam_param.dist_factor[4]= _cam_model.fx()*_image_scale; // _cam_info.K[0];  //fx
        _ar_cam_param.dist_factor[5]= _cam_model.fy()*_image_scale;//_cam_info.K[4];  //fy
    }
    else{
        //_ar_cam_param.dist_factor[2] = 0;                  // We don't know the right value, so ignore it
        ROS_ERROR("Camera left: Distortion Parameters from ROS MSG mismatch ARToolkit Model");
    }
    //update ar_detection parameters
    updateArParam(_ar_handle, &_ar_cam_param);
    //set the state again true
    _cam_info_up_to_date=true;
}



void ARTracker::mainLoop(void)
{
		//check if ar toolkit and ros is initialized
		if(!_init){
			ROS_INFO_THROTTLE(1,"Waiting for initialization:");
			ROS_INFO_THROTTLE(1,"status: img received: \t\t%d",_img_received);
			ROS_INFO_THROTTLE(1,"status: cam info received: \t%d",_cam_info_received);
			return;
		}
		
    ROS_INFO_ONCE("Start Tracking");

    AR2VideoBufferT *buffL;
    ARMarkerInfo* markerInfoL;
    int markerNumL;
    ARdouble err;
    ARdouble transR[3][4];
    ARPose poseR;
    int i, j, kL, kR;
    static ros::Time time_prev;
    ros::Duration time_elapsed;
    float framerate=0;



    //check if new image was received and reset the variable to false
    if(!_img_received ){
        ROS_INFO_THROTTLE(1,"Waiting for image..");
        return;
    }else{
        _img_received=false;
    }
    //convert from opencv
    ROS_INFO_ONCE("Starting detection");


    //copy original image
    _img_full=_capture_left->image.clone();

    //If Roi, then extraxct it from the image
    if(_cam_info.roi.height>0 && _cam_info.roi.width>0){
        _capture_left->image=_capture_left->image(_cam_model.rawRoi()).clone();
        ROS_DEBUG("Mat Size Roi: [%d,%d]",_capture_left->image.rows ,_capture_left->image.cols);
    }
    //scale image
    if(_image_scale<1){
        //resize image
        cv::Size size_new(_ar_cam_param.xsize,_ar_cam_param.ysize);
        cv::resize(_capture_left->image,_capture_left->image,size_new,cv::INTER_AREA);
        ROS_DEBUG("Mat Size after Scaling: [%d,%d]",size_new.height,size_new.width);
    }
    //equalizeHist( _capture_left->image, _capture_left->image );

    //cv::GaussianBlur( _capture_left->image, _capture_left->image, cv::Size( 7,7 ), 0, 0 );
    /*
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
    clahe->setClipLimit(2);
    clahe->setTilesGridSize(cv::Size(7,7));
    clahe->apply(_capture_left->image,_capture_left->image);
     */
    if(_show_image_window) {
        cv::namedWindow("after preprocess", CV_WINDOW_AUTOSIZE);
        cv::imshow("after preprocess", _capture_left->image);
        cv::waitKey(20);
    }

    //Cast CV image to ar toolkit image
    ARUint8		*ar_image  = (ARUint8 *) ((IplImage) _capture_left->image).imageData;
    //update parameters
    if (ar_image) {
        _call_count_marker_detect++; // Increment ARToolKit FPS counter.
        //compute framerate
        time_elapsed=(ros::Time::now()-time_prev);
        framerate=(1.0/(float)time_elapsed.toSec());
        time_prev=ros::Time::now();


        // Detect the markers in the video frame.
        if (arDetectMarker(_ar_handle, ar_image) < 0) {

            exit(-1);
        }



        // Get detected markers
        markerInfoL = arGetMarker(_ar_handle);
        markerNumL = arGetMarkerNum(_ar_handle);

        // Update markers.
        for (i = 0; i < _markers_square_count; i++) {
            _ar_markers_square[i].validPrev = _ar_markers_square[i].valid;
            _ar_markers_square[i].valid = FALSE;

            // Check through the marker_info array for highest confidence
            // visible marker matching our preferred pattern.
            kL = kR = -1;
            if (_ar_markers_square[i].patt_type == AR_PATTERN_TYPE_TEMPLATE) {
                for (j = 0; j < markerNumL; j++) {
                    if (_ar_markers_square[i].patt_id == markerInfoL[j].idPatt) {
                        if (kL == -1) {
                            if (markerInfoL[j].cfPatt >= _ar_markers_square[i].matchingThreshold) kL = j; // First marker detected.
                        } else if (markerInfoL[j].cfPatt > markerInfoL[kL].cfPatt) kL = j; // Higher confidence marker detected.
                    }
                }
                if (kL != -1) {
                   // ROS_INFO("Marker Left Cam Detected");
                    markerInfoL[kL].id = markerInfoL[kL].idPatt;
                    markerInfoL[kL].cf = markerInfoL[kL].cfPatt;
                    markerInfoL[kL].dir = markerInfoL[kL].dirPatt;
                }

            } else {
                for (j = 0; j < markerNumL; j++) {
                    if (_ar_markers_square[i].patt_id == markerInfoL[j].idMatrix) {
                        if (kL == -1) {
                            if (markerInfoL[j].cfMatrix >= _ar_markers_square[i].matchingThreshold) kL = j; // First marker detected.
                        } else if (markerInfoL[j].cfMatrix > markerInfoL[kL].cfMatrix) kL = j; // Higher confidence marker detected.
                    }
                }
                if (kL != -1) {
                    markerInfoL[kL].id = markerInfoL[kL].idMatrix;
                    markerInfoL[kL].cf = markerInfoL[kL].cfMatrix;
                    markerInfoL[kL].dir = markerInfoL[kL].dirMatrix;
                }
            }
            if (kL != -1 ) {
                err = arGetTransMatSquare(gAR3DHandleL,&markerInfoL[kL],_ar_markers_square[i].marker_width, _ar_markers_square[i].trans);
                if (err < 10.0) {
                    _ar_markers_square[i].valid = TRUE;
                }
            }

            if (_ar_markers_square[i].valid) {
                // Filter the pose estimate.
                if (_ar_markers_square[i].ftmi) {
                    if (arFilterTransMat(_ar_markers_square[i].ftmi, _ar_markers_square[i].trans, !_ar_markers_square[i].validPrev) < 0) {
                        ARLOGe("arFilterTransMat error with marker %d.\n", i);
                    }
                }

                // We have a new pose, so set that.
                tf::Transform tf;
                ARdouble ARquat[4];
                ARdouble ARpos[3];
                tf::Quaternion ROSquat;
                std::string frame_id="Patt.ID=";
                frame_id.append(boost::lexical_cast<std::string>(_ar_markers_square[i].patt_id));
                arUtilMat2QuatPos(_ar_markers_square[i].trans,ARquat,ARpos);
                //ROSquat.setValue(-ARquat[0],-ARquat[1],-ARquat[2],ARquat[3]);
                ROSquat.setValue(ARquat[0],ARquat[1],ARquat[2],ARquat[3]);
                tf.setOrigin(tf::Vector3(ARpos[0]*UnitAR2ROS,ARpos[1]*UnitAR2ROS,ARpos[2]*UnitAR2ROS));
                tf.setRotation(ROSquat);
                _tf_br.sendTransform(tf::StampedTransform(tf, _capture_time_left, _cam_info.header.frame_id ,frame_id));
                //ROS_INFO("%f  %f  %f  %f", _ar_markers_square[i].trans[0][0],_ar_markers_square[i].trans[0][1], _ar_markers_square[i].trans[0][2],_ar_markers_square[i].trans[0][3] );
                //ROS_INFO("%f  %f  %f  %f", _ar_markers_square[i].trans[1][0],_ar_markers_square[i].trans[1][1], _ar_markers_square[i].trans[1][2],_ar_markers_square[i].trans[1][3] );
                //ROS_INFO("%f  %f  %f  %f", _ar_markers_square[i].trans[2][0],_ar_markers_square[i].trans[2][1], _ar_markers_square[i].trans[2][2],_ar_markers_square[i].trans[2][3] );
                //arglCameraViewRH((const ARdouble (*)[4])_ar_markers_square[i].trans, _ar_markers_square[i].pose.T, 1.0f /*VIEW_SCALEFACTOR*/);
                //arUtilMatMul((const ARdouble (*)[4])transL2R, (const ARdouble (*)[4])_ar_markers_square[i].trans, transR);
                //arglCameraViewRH((const ARdouble (*)[4])transR, poseR.T, 1.0f /*VIEW_SCALEFACTOR*/);
                // Tell any dependent objects about the update.
                ROS_INFO("[%2d] Result:\terr: %.2f\tposXYZ: [%.2f,\t%.2f,\t%.2f]\t%.2f fps\n", i, err,tf.getOrigin().x(),tf.getOrigin().y(),tf.getOrigin().z(),framerate);
                //marker has been detected, now reduce scale of the image and
                //if (_ar_markers_square[i].validPrev && i== _marker_to_track) {
                if ( i== _marker_to_track) {

                    extractSample(tf);

                    ///compute new scale
                    //get vertex and compute average length of pixel of the marker in the image
                    double x=_ar_markers_square[i].marker_height/1000.0;
                    double y=0;
                    double z=tf.getOrigin().length();
                    cv::Point3d p3d(x,0,z);
                    cv::Point2d p2d= _cam_model.project3dToPixel(p3d);
                    //todo check in detail here, somehow is the y component not 0 as expected but slightly around 0
                    //ROS_INFO("P2D roi [%.1f,%.1f]",p2d.x,p2d.y);
                    //p2d = _cam_model.unrectifyPoint(p2d);
                    //ROS_INFO("P2D roi unrectified [%.1f,%.1f]",p2d.x,p2d.y);

                    p2d=_cam_model.toFullResolution(p2d);
                    //ROS_INFO("P2D full [%.1f,%.1f]",p2d.x,p2d.y);

                    p2d.x-=_cam_model.fullIntrinsicMatrix()(0,2);
                    p2d.y-=_cam_model.fullIntrinsicMatrix()(1,2);
                    //ROS_INFO("P2D final [%.1f,%.1f]",p2d.x,p2d.y);
                    int l= abs(p2d.x);
                    //check if length of the marker projection is bigger than the maximal specified marker size and
                    //compute the new scale
                    if(l>_marker_size_max){
                        _image_scale=(float)(_marker_size_max)/(float)l;
                        ROS_DEBUG("New scale= %0.2f , current marker size in pixel= %d",_image_scale,l);
                        if(_image_scale<=0){
                            ROS_ERROR("Image scale <=0");
                        }
                    }
                    //if the length is smaller set the image to the maximal resolution
                    else{
                        _image_scale=1;
                    }

                    //update roi
                    _roi_tracker.updateRoi(tf.getOrigin(),_roi_width/_image_scale,_roi_height/_image_scale,_ar_markers_square[i].marker_height/1000.0,_capture_time_left,_predict_roi,_predict_roi_dt);
                    //publish and update
                    //_pub_ciln.publish(_roi_tracker.getCameraInfo());
                    updateCameraInfo(_roi_tracker.getCameraInfo());
                    return;

                }
            }
                //marker[i] was not detected
            else {
                //if marker i is the one we ware tracking
                 if (!_ar_markers_square[i].validPrev && i== _marker_to_track){
                     //if cnt reached SCALE_WAIT_RESET reset scale to default value
                     static int cnt_scale=0;
                     if(cnt_scale>=_scale_wait_reset){
                         ROS_INFO("Reset scale");
                         _image_scale=(float)_scale_default;
                         cnt_scale=0;
                         _cam_info_up_to_date=false;
                     }else{
                         cnt_scale++;
                     }

                     //if scale has been reset or ROI has been increased update parameters
                     if(_cam_info_up_to_date==false || _roi_tracker.enlargeRoi(_roi_region_grow_x,_roi_region_grow_y,_roi_wait_reset)){
                         updateCameraInfo(_roi_tracker.getCameraInfo());
                         return;
                     }


                }
            }
        }
        } else {

    }

}


void ARTracker::extractSample(tf::Transform &cam2marker) {

    /// todo: there is somewhere an error.. probalby the marker was not trained the right way.. anyway there is a rotation of -90 ° around the z axis of the marker when detected
    /// be aware !!
    tf::Transform pallet_left;
    tf::Transform pallet_right;
    pallet_left.setIdentity();
    pallet_right.setIdentity();

    image_geometry::PinholeCameraModel model ;
    sensor_msgs::CameraInfo info=_cam_info;
    info.roi.y_offset=0;
    info.roi.x_offset=0;
    info.roi.width=0;
    info.roi.height=0;
    model.fromCameraInfo(info);

    float x_off=0.44; //x offsetz to the origin of the pallet frame
    float y_off=0.69; // y offset to the origin of the pallet frame
    float object_width=0.9;
    float object_height=0.1;
    int boarder_x=10;
    int boarder_y=5;
    static long cnt=0;
    cv::Mat img;
    cv::Size obj_size_scaled(160,32); //the roi will be scaled that it fits the width and the area will be increased that it fits the height
    std::string dir_pos="/home/tman/ros_ws/image_samples/pos/";
    std::string dir_neg="/home/tman/ros_ws/image_samples/neg/";
    std::string file="images.txt";

    static bool is_open=false;

    //open directory search for all files with *.jpg extension, sort them and find the highest number
    if(!is_open){
        fs::path root(dir_pos);
        std::vector<fs::path> ret;

        if(!fs::exists(root) || !fs::is_directory(root)) {
            ROS_ERROR("Dir doesnt exist");
            exit(1);
        }

        fs::recursive_directory_iterator it(root);
        fs::recursive_directory_iterator endit;

        while(it != endit)
        {
            if(fs::is_regular_file(*it) && it->path().extension() == ".jpg") ret.push_back(it->path().filename());
            ++it;

        }
        std::sort(ret.begin(),ret.end(), [&ret] (fs::path & one, fs::path & two){
            return one.string()< two.string();
        } );

        std::string last =ret.back().string();
        std::regex r("[1-9][[:digit:]]+");
        std::smatch match;

        //check if number was matched and set the cnt variable accoringly
        if(std::regex_search(last,match,r)){
            cnt=std::stoi(match.str())+1;
            ROS_INFO("Highest number of image name: %s",match.str().c_str());
            ROS_INFO("Starting naming at: %u", cnt);

        }



        is_open=true;
    }


    if(true){
        pallet_left.setOrigin(tf::Vector3(y_off,-x_off,0)); //notice here the x and y values are exchanged because of this bug
        pallet_right=pallet_left;
        tf::Vector3 orig_right=pallet_right.getOrigin();
        orig_right.setY(orig_right.y()+ object_width);
        pallet_right.setOrigin(orig_right);
        pallet_left  = cam2marker*pallet_left;
        pallet_right = cam2marker*pallet_right;
    }
    else{
        pallet_left=cam2marker;
        pallet_left.setOrigin(tf::Vector3(-x_off+cam2marker.getOrigin().x(),y_off+cam2marker.getOrigin().y(),cam2marker.getOrigin().z()));
    }
    //ROS_INFO("[Pallet Left XYZ: [%.2f,\t%.2f,\t%.2f]\n",pallet_left.getOrigin().x(),pallet_left.getOrigin().y(),pallet_left.getOrigin().z());
    //ROS_INFO("[Pallet Right XYZ: [%.2f,\t%.2f,\t%.2f]\n",pallet_right.getOrigin().x(),pallet_right.getOrigin().y(),pallet_right.getOrigin().z());
    double y,p,r;
    cam2marker.getBasis().getEulerYPR(y,p,r);
    //ROS_INFO("Rot: [yaw,pitch,roll] = [%.2f,\t%.2f,\t%.2f]",y,p,r);

    //compute corner points of roi
    cv::Point bl=model.project3dToPixel(cv::Point3d(pallet_left.getOrigin().x(),pallet_left.getOrigin().y(),pallet_left.getOrigin().z()));
    cv::Point ul=model.project3dToPixel(cv::Point3d(pallet_left.getOrigin().x(),pallet_left.getOrigin().y()-object_height,pallet_left.getOrigin().z()));

    cv::Point br=model.project3dToPixel(cv::Point3d(pallet_right.getOrigin().x(),pallet_right.getOrigin().y(),pallet_right.getOrigin().z()));
    cv::Point ur=model.project3dToPixel(cv::Point3d(pallet_right.getOrigin().x(),pallet_right.getOrigin().y()-object_height,pallet_right.getOrigin().z()));


    //add boarder to the points
    bl += cv::Point(-boarder_x,boarder_y);
    br += cv::Point(boarder_x,boarder_y);
    ul += cv::Point(-boarder_x,-boarder_y);
    ur += cv::Point(boarder_x,-boarder_y);

    //get bounding box of object
    std::vector<cv::Point> pts {bl,br,ul,ur};
    cv::Rect obj_rect=cv::boundingRect(pts);

    ROS_DEBUG("Bounding Rect:");
    ROS_DEBUG_STREAM(obj_rect);
    //ROS_INFO("Points:");
    //ROS_INFO_STREAM(std::cout<<"bl: "<<bl<<"ul: "<<ul<<"br: "<<br<<"ur: "<<ur<<std::endl);

    //check if points lie in the image
    cv::Rect rect=cv::Rect(cv::Point(), _img_full.size());

    //if object not in image return
    if(!(rect.contains(bl) && rect.contains(ul) && rect.contains(br)  && rect.contains(ur))){
        ROS_ERROR("Object lies outside image");
        return;
    }


    //determine required scale
    float scale=1;
    if(obj_rect.width>obj_size_scaled.width){
        scale=(float)obj_size_scaled.width/(float)(obj_rect.width);
    }else{
        ROS_ERROR("Image smaller than chosen scale");
        return;
    }

    ROS_DEBUG("Scale: \t %.2f",scale);
    //resize roi
    cv::Rect obj_rect_scaled = cv::Rect((int)round(obj_rect.x*scale),(int)round(obj_rect.y*scale),(int)round(obj_rect.width*scale),(int)round(obj_rect.height*scale));
    ROS_DEBUG("Bounding Rect scaled:");
    ROS_DEBUG_STREAM(obj_rect_scaled);

    //check if the height is smaller the required size
    if(obj_rect_scaled.height>obj_size_scaled.height){
        ROS_ERROR("After scaling still to big to fit the required minimum height");
        return;
    }

    //resize rectancle to a precise height of the scaled object size
    int y_corr=obj_size_scaled.height-obj_rect_scaled.height;
    if(y_corr>0){
        //check if rect would fit image
        if((obj_rect_scaled.y+obj_rect_scaled.height+y_corr)>round(_img_full.rows*scale)){
            ROS_ERROR("Enlarging rectangle failed..");
            return;
        }
        obj_rect_scaled.height+=y_corr;
    }

    //rectify
    model.rectifyImage(_img_full,img);
    //now resize image
    cv::resize(_img_full,img,cv::Size(0,0),scale,scale,cv::INTER_AREA);

    //generate negative sample
    int x_ran=0;
    int y_ran=0;
    do{
        x_ran = std::rand() % (img.cols-obj_rect_scaled.width);
        y_ran = std::rand() % (img.rows-obj_rect_scaled.height);
    }
    while(obj_rect_scaled.contains(cv::Point(x_ran,y_ran))
          ||    obj_rect_scaled.contains(cv::Point(x_ran+obj_rect_scaled.width,y_ran+obj_rect_scaled.height))
          ||    obj_rect_scaled.contains(cv::Point(x_ran,y_ran+obj_rect_scaled.height))
          ||    obj_rect_scaled.contains(cv::Point(x_ran+obj_rect_scaled.width,y_ran)));
    cv::Rect obj_neg_rect_scaled=cv::Rect(x_ran,y_ran,obj_rect_scaled.width,obj_rect_scaled.height);
    //get roi
    cv::Mat out_pos = img(obj_rect_scaled);
    cv::Mat out_neg = img(obj_neg_rect_scaled);

    //final check if dimensions are right
    if(out_pos.cols!= obj_size_scaled.width || out_pos.rows != obj_size_scaled.height){
        ROS_ERROR("THERE MUST BE SOME ERROR IN THE PROGRAMM !!!");
        ROS_INFO("Out size:");
        ROS_INFO_STREAM(out_pos.size);
        exit(1);
    }

    std::string img_name;
    char img_name_c[10];
    std::sprintf(img_name_c,"%.7u",cnt);
    cv::imshow("ROI of pallet_left", out_pos);
    cv::imshow("ROI of negative sample",out_neg);
    cv::imwrite(dir_pos+img_name_c+".jpg",out_pos);
    cv::imwrite(dir_neg+img_name_c+".jpg",out_neg);
    cv::waitKey(10);
    cnt++;
}
