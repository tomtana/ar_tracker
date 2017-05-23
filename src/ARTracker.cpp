#include <ARTracker.h>
#include <opencv2/highgui/highgui.hpp>

ARTracker::ARTracker(ros::NodeHandle & nh):
        _nh(nh),
        _it(_nh),
        _img_received(false),
        _cam_info_received(false),
        _cam_info_up_to_date(true)
{
    std::string topic_image_raw; //image topic
    //read param
    _nh.param<std::string>(ros::this_node::getName()+"/camera",topic_image_raw,"/left"); //image topic
    _nh.param<std::string>(ros::this_node::getName()+"/marker_file", _ar_marker_config_data_filename, "../../../src/ar_tracker/Data/markers.dat"); //path to marker config file
    _nh.param<int>(ros::this_node::getName()+"/ar_tracking_mode", _ar_tracking_mode, AR_USE_TRACKING_HISTORY_V2);
    _nh.param<int>(ros::this_node::getName()+"/ar_thresh_mode", _ar_threshold_mode, AR_LABELING_THRESH_MODE_AUTO_ADAPTIVE);
    _nh.param<int>(ros::this_node::getName()+"/ar_thesh_auto_inv", ar_thresh_mode_auto_intervall, 10);
    _nh.param<int>(ros::this_node::getName()+"/ar_thesh", _ar_thresh, 100);

    //init all parameters and so on and so forth
    //_pub = _it.advertise("test", 1);
    ROS_INFO("Subscribing to camera topics: \n%s/image_raw \n%s/camera_info", topic_image_raw.data(),topic_image_raw.data());
    _sub_il = _it.subscribe(topic_image_raw+"/image_raw", 1, &ARTracker::imageLeftCallback, this);
    _sub_cil = _nh.subscribe(topic_image_raw +"/camera_info",1,&ARTracker::cameraInfoLeftCallback ,this);


    ROS_INFO("AR_LABELING_WORK_SIZE: %d",AR_LABELING_WORK_SIZE);



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
    ROS_INFO_ONCE("Marker count = %d\n", _markers_square_count);

    ///Configure detection
    arSetMarkerExtractionMode(_ar_handle, _ar_tracking_mode);
    arSetLabelingThreshMode(_ar_handle,static_cast<AR_LABELING_THRESH_MODE>(_ar_threshold_mode));
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
}

void ARTracker::arParamUpdate(ARHandle *handle,ARParam *param) {
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
    arSetLabelingThreshMode(_ar_handle, static_cast<AR_LABELING_THRESH_MODE>(_ar_threshold_mode));
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
        if(_cam_info.roi.height>0 && _cam_info.roi.width>0){
            _capture_left = cv_bridge::toCvCopy (incoming_img, sensor_msgs::image_encodings::MONO8);
            _capture_left->image=_capture_left->image(_cam_model.rawRoi()).clone();
            ROS_INFO("Mat Size: [%d,%d]",_capture_left->image.rows ,_capture_left->image.cols);
        }else{
            _capture_left = cv_bridge::toCvCopy (incoming_img, sensor_msgs::image_encodings::MONO8);

        }
        if(image_scale<1){
            //resize image
            cv::Size size_new(_ar_cam_param.xsize,_ar_cam_param.ysize);
            cv::resize(_capture_left->image,_capture_left->image,size_new,cv::INTER_AREA);
            ROS_DEBUG("Mat Size after Scaling: [%d,%d]",size_new.height,size_new.width);
        }

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
    _ar_cam_param.mat[0][0] *= image_scale; //fx*scale
    _ar_cam_param.mat[0][2] *= image_scale; //cx*scale
    _ar_cam_param.mat[1][1] *= image_scale; //fy*scale
    _ar_cam_param.mat[1][2] *= image_scale; //cy*scale


    _ar_cam_param.xsize = (int)round(_cam_model.reducedResolution().width*image_scale);
    _ar_cam_param.ysize = (int)round(_cam_model.reducedResolution().height*image_scale);


    //set 0 to the last column
    _ar_cam_param.mat[0][3] = 0;
    _ar_cam_param.mat[1][3] = 0;
    _ar_cam_param.mat[2][3] = 0;


    _ar_cam_param.dist_factor[6] = (int)(_cam_model.cx()*image_scale);//_cam_info.K[2];       // x0 = cX from openCV calibration
    _ar_cam_param.dist_factor[7] = (int)(_cam_model.cy()*image_scale);//_cam_info.K[5];       // y0 = cY from openCV calibration
    if ( _cam_info.distortion_model == "plumb_bob" && _cam_info.D.size() == 5){
      _ar_cam_param.dist_factor[0]= _cam_info.D[0];  //k0
      _ar_cam_param.dist_factor[1]= _cam_info.D[1];  //k1
      _ar_cam_param.dist_factor[2]= _cam_info.D[2];  //p0
      _ar_cam_param.dist_factor[3]= _cam_info.D[3];  //p1
      _ar_cam_param.dist_factor[4]= _cam_model.fx()*image_scale; // _cam_info.K[0];  //fx
      _ar_cam_param.dist_factor[5]= _cam_model.fy()*image_scale;//_cam_info.K[4];  //fy
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
        ROS_INFO("updateCameraInfo:\tcamera info has changed.");
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
    _ar_cam_param.mat[0][0] *= image_scale;
    _ar_cam_param.mat[0][2] *= image_scale;
    _ar_cam_param.mat[1][1] *= image_scale;
    _ar_cam_param.mat[1][2] *= image_scale;

    //compute new image size
    //todo compute always image size which is odd/even.. maybe needed when preprocessing is wanted
    _ar_cam_param.xsize = (int)round(_cam_model.reducedResolution().width*image_scale);
    _ar_cam_param.ysize = (int)round(_cam_model.reducedResolution().height*image_scale);

    //set 0 to the last column
    _ar_cam_param.mat[0][3] = 0;
    _ar_cam_param.mat[1][3] = 0;
    _ar_cam_param.mat[2][3] = 0;

    _ar_cam_param.dist_factor[6] = (int)(_cam_model.cx()*image_scale);//_cam_info.K[2];       // x0 = cX from openCV calibration
    _ar_cam_param.dist_factor[7] = (int)(_cam_model.cy()*image_scale);//_cam_info.K[5];       // y0 = cY from openCV calibration

    if ( _cam_info.distortion_model == "plumb_bob" && _cam_info.D.size() == 5){
        _ar_cam_param.dist_factor[0]= _cam_info.D[0];  //k0
        _ar_cam_param.dist_factor[1]= _cam_info.D[1];  //k1
        _ar_cam_param.dist_factor[2]= _cam_info.D[2];  //p0
        _ar_cam_param.dist_factor[3]= _cam_info.D[3];  //p1
        _ar_cam_param.dist_factor[4]= _cam_model.fx()*image_scale; // _cam_info.K[0];  //fx
        _ar_cam_param.dist_factor[5]= _cam_model.fy()*image_scale;//_cam_info.K[4];  //fy
    }
    else{
        //_ar_cam_param.dist_factor[2] = 0;                  // We don't know the right value, so ignore it
        ROS_ERROR("Camera left: Distortion Parameters from ROS MSG mismatch ARToolkit Model");
    }
    _ar_cam_param.dist_factor[8] = 1.0;                  // scale factor
    _ar_cam_param.dist_function_version=4;

    //update ar_detection parameters
    arParamUpdate(_ar_handle,&_ar_cam_param);
    //set the state again true
    _cam_info_up_to_date=true;
}



void ARTracker::mainLoop(void)
{
    static ros::Time time_prev;
    ros::Duration time_elapsed;
    float framerate=0;

    if(_init){
        ROS_DEBUG("Start Tracking");

        AR2VideoBufferT *buffL;
        ARMarkerInfo* markerInfoL;
        int markerNumL;
        ARdouble err;
        ARdouble transR[3][4];
        ARPose poseR;
        int i, j, kL, kR;



        //check if new image was received and reset the variable to false
        if(!_img_received){
            ROS_DEBUG("Waiting for image..");
            return;
        }else{
            _img_received=false;
        }
        // Grab a video frame.
        //convert from opencv
        ROS_DEBUG("Starting detection");
        equalizeHist( _capture_left->image, _capture_left->image );
        int s;

        //cv::GaussianBlur( _capture_left->image, _capture_left->image, cv::Size( 7,7 ), 0, 0 );
        /*
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
        clahe->setClipLimit(2);
        clahe->setTilesGridSize(cv::Size(7,7));
        clahe->apply(_capture_left->image,_capture_left->image);
         */
        cv::namedWindow("after preprocess",CV_WINDOW_AUTOSIZE);
        cv::imshow("after preprocess",_capture_left->image);
        cv::waitKey(20);

        //Cast CV image to ar toolkit image
        ARUint8		*ar_image  = (ARUint8 *) ((IplImage) _capture_left->image).imageData;
        //update parameters
        if (ar_image) {
            //cv::imshow("test ",_capture_left->image);
            //cv::waitKey(0);
            _call_count_marker_detect++; // Increment ARToolKit FPS counter.

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

                    if (err < 10.0) _ar_markers_square[i].valid = TRUE;


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
                    _tf_br.sendTransform(tf::StampedTransform(tf, _capture_time_left, "pylon_camera_left_node" ,frame_id));
                    //ROS_INFO("%f  %f  %f  %f", _ar_markers_square[i].trans[0][0],_ar_markers_square[i].trans[0][1], _ar_markers_square[i].trans[0][2],_ar_markers_square[i].trans[0][3] );
                    //ROS_INFO("%f  %f  %f  %f", _ar_markers_square[i].trans[1][0],_ar_markers_square[i].trans[1][1], _ar_markers_square[i].trans[1][2],_ar_markers_square[i].trans[1][3] );
                    //ROS_INFO("%f  %f  %f  %f", _ar_markers_square[i].trans[2][0],_ar_markers_square[i].trans[2][1], _ar_markers_square[i].trans[2][2],_ar_markers_square[i].trans[2][3] );
                    arglCameraViewRH((const ARdouble (*)[4])_ar_markers_square[i].trans, _ar_markers_square[i].pose.T, 1.0f /*VIEW_SCALEFACTOR*/);
                    arUtilMatMul((const ARdouble (*)[4])transL2R, (const ARdouble (*)[4])_ar_markers_square[i].trans, transR);
                    arglCameraViewRH((const ARdouble (*)[4])transR, poseR.T, 1.0f /*VIEW_SCALEFACTOR*/);
                    // Tell any dependent objects about the update.
                    time_elapsed=(ros::Time::now()-time_prev);
                    framerate=(1.0/(float)time_elapsed.toSec());
                    time_prev=ros::Time::now();
                    ROS_INFO("[%2d] Result:\terr: %.2f\tposXYZ: [%.2f,\t%.2f,\t%.2f]\t%.2f fps\n", i, err,tf.getOrigin().x(),tf.getOrigin().y(),tf.getOrigin().z(),framerate);
                    //marker has been detected, now reduce scale of the image and
                    //if (_ar_markers_square[i].validPrev && i== _selected_marker) {
                    if ( i== _selected_marker) {

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
                        if(l>marker_size_max){
                            image_scale=(float)(marker_size_max)/(float)l;
                            ROS_INFO("New scale= %0.2f , l= %d",image_scale,l);
                        }else{
                            image_scale=SCALE_DEFAULT;
                        }

                        //update roi
                        _roi_tracker.updateRoi(tf.getOrigin(),marker_roi_x,marker_roi_y,_ar_markers_square[i].marker_height/1000.0,_capture_time_left,0.3);
                        //publish and update
                        _pub_ciln.publish(_roi_tracker.getCameraInfo());
                        updateCameraInfo(_roi_tracker.getCameraInfo());
                        return;

                    }
                }
                else {

                     if (!_ar_markers_square[i].validPrev && i== _selected_marker){
                         //reset scale
                         static int cnt_scale=0;
                         if(cnt_scale>=SCALE_WAIT_RESET){
                             ROS_INFO("Reset scale");
                             image_scale=SCALE_DEFAULT;
                             cnt_scale=0;
                             _cam_info_up_to_date=false;
                         }else{
                             cnt_scale++;
                         }

                         //if scale has been reset or ROI has been increased update parameters
                         if(_cam_info_up_to_date==false || _roi_tracker.enlargeRoi(ROI_REGION_GROW_X,ROI_REGION_GROW_Y,ROI_WAIT_RESET)){
                             updateCameraInfo(_roi_tracker.getCameraInfo());
                             return;
                         }


                    }
                }
            }
                // Tell GLUT the display has changed.
            } else {

        }
    }

}

