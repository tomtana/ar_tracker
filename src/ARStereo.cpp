#include <ARStereo.h>

ARStereo::ARStereo(ros::NodeHandle & nh):
        _nh(nh),
        _it(_nh),
        _il_received(false),
        _ir_received(false),
        _cil_received(false),
        _cir_received(false)
{
    //init all parameters and so on and so forth
    //_pub = _it.advertise("test", 1);
    _sub_il = _it.subscribe(_cameraImageLeftTopic, 1, &ARStereo::imageLeftCallback, this);
    _sub_cil = _nh.subscribe(_cameraInfoLeftTopic,1,&ARStereo::cameraInfoLeftCallback ,this);
    
}
    
void ARStereo::ARInit(){
    
    ROS_INFO("Initializing ARToolkit..");
    
    char    transL2RDefault[] = "Data/transL2R.dat";
    transL2R[0][0]=1;
    transL2R[0][1]=0;
    transL2R[0][2]=0;
    transL2R[0][3]=-0.001;
    transL2R[1][0]=0;
    transL2R[1][1]=1;
    transL2R[1][2]=0;
    transL2R[1][3]=0;
    transL2R[2][0]=0;
    transL2R[2][1]=0;
    transL2R[2][2]=1;
    transL2R[2][3]=0;
     //
    // AR init.
    //
    if ((gCparamLTL = arParamLTCreate(&_cam_param_left_art, AR_PARAM_LT_DEFAULT_OFFSET)) == NULL) {
        ROS_ERROR("setupCamera(): Error: arParamLTCreate.\n");
    }
    if ((gCparamLTR = arParamLTCreate(&_cam_param_right_art, AR_PARAM_LT_DEFAULT_OFFSET)) == NULL) {
    ROS_ERROR("setupCamera(): Error: arParamLTCreate.\n");
    }
    
    // Init AR.
    gARPattHandle = arPattCreateHandle();
	if (!gARPattHandle) {
		ARLOGe("Error creating pattern handle.\n");
		exit(-1);
	}
    
    gARHandleL = arCreateHandle(gCparamLTL);
    gARHandleR = arCreateHandle(gCparamLTR);
    if (!gARHandleL || !gARHandleR) {
        ROS_ERROR("Error creating AR handle.\n");
		exit(-1);
    }
    arPattAttach(gARHandleL, gARPattHandle);
    arPattAttach(gARHandleR, gARPattHandle);
    
    if (arSetPixelFormat(gARHandleL, AR_PIXEL_FORMAT_BGR) < 0 ||
        arSetPixelFormat(gARHandleR, AR_PIXEL_FORMAT_BGR) < 0) {
        ROS_ERROR("Error setting pixel format.\n");
		exit(-1);
    }
    
    gAR3DHandleL = ar3DCreateHandle(&gCparamLTL->param);
    gAR3DHandleR = ar3DCreateHandle(&gCparamLTR->param);
    if (!gAR3DHandleL || !gAR3DHandleR) {
        ROS_ERROR("Error creating 3D handle.\n");
		exit(-1);
    }
    
    /*
     * Parameters are transmitted by camer_info via ros topic
    if (arParamLoadExt(transL2RDefault, transL2R) < 0) {
        ROS_ERROR("Error: arParamLoadExt.\n");
        exit(-1);
    }
     */
    
     
    arUtilMatInv((const ARdouble (*)[4])transL2R, transR2L);
    arParamDispExt(transL2R);
    arParamDisp((const ARParam*) &_cam_param_left_art);
    gAR3DStereoHandle = ar3DStereoCreateHandle(&(gCparamLTL->param), &(gCparamLTR->param), AR_TRANS_MAT_IDENTITY, transL2R);
    if (!gAR3DStereoHandle) {
        ROS_ERROR("Error: ar3DCreateHandle.\n");
        exit(-1);
    }

    //
    // Markers setup.
    //
    
    // Load marker(s).
    newMarkers(markerConfigDataFilename.c_str(), gARPattHandle, &markersSquare, &markersSquareCount, &gARPattDetectionMode);
    ROS_INFO("Marker count = %d\n", markersSquareCount);
    
    //
    // Other ARToolKit setup.
    //
    
    arSetMarkerExtractionMode(gARHandleL, AR_USE_TRACKING_HISTORY_V2);
    arSetMarkerExtractionMode(gARHandleR, AR_USE_TRACKING_HISTORY_V2);
    //arSetMarkerExtractionMode(gARHandleL, AR_NOUSE_TRACKING_HISTORY);
    //arSetMarkerExtractionMode(gARHandleR, AR_NOUSE_TRACKING_HISTORY);
    //arSetLabelingThreshMode(gARHandleL, AR_LABELING_THRESH_MODE_MANUAL); // Uncomment to force manual thresholding.
    //arSetLabelingThreshMode(gARHandleR, AR_LABELING_THRESH_MODE_MANUAL); // Uncomment to force manual thresholding.
    
    // Set the pattern detection mode (template (pictorial) vs. matrix (barcode) based on
    // the marker types as defined in the marker config. file.
    arSetPatternDetectionMode(gARHandleL, gARPattDetectionMode); // Default = AR_TEMPLATE_MATCHING_COLOR
    arSetPatternDetectionMode(gARHandleR, gARPattDetectionMode); // Default = AR_TEMPLATE_MATCHING_COLOR
    
    // Other application-wide marker options. Once set, these apply to all markers in use in the application.
    // If you are using standard ARToolKit picture (template) markers, leave commented to use the defaults.
    // If you are usign a different marker design (see http://www.artoolworks.com/support/app/marker.php )
    // then uncomment and edit as instructed by the marker design application.
    //arSetLabelingMode(gARHandleL, AR_LABELING_BLACK_REGION); // Default = AR_LABELING_BLACK_REGION
    //arSetLabelingMode(gARHandleR, AR_LABELING_BLACK_REGION); // Default = AR_LABELING_BLACK_REGION
    //arSetBorderSize(gARHandleL, 0.25f); // Default = 0.25f
    //arSetBorderSize(gARHandleR, 0.25f); // Default = 0.25f
    //arSetMatrixCodeType(gARHandleL, AR_MATRIX_CODE_3x3); // Default = AR_MATRIX_CODE_3x3
    //arSetMatrixCodeType(gARHandleR, AR_MATRIX_CODE_3x3); // Default = AR_MATRIX_CODE_3x3
}

ARStereo::~ARStereo(){
    
}

void ARStereo::imageLeftCallback(const sensor_msgs::ImageConstPtr& incoming_img){
    ROS_DEBUG("Image Left received.");
    #if ROS_VERSION_MINIMUM(1, 9, 0)
    try
    {
      _capture_left = cv_bridge::toCvCopy (incoming_img, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    #else
    ROS_ERROR("Your ROS Version is not supported.");
    #endif
    if(_il_received&&(_init==false)){
        ARInit();
        ROS_INFO("ARToolkit initialized");
        _init=true;
    }
    _il_received=true;
    //_pub.publish(incoming_img);
    imageRightCallback(incoming_img);
    
}

void ARStereo::imageRightCallback(const sensor_msgs::ImageConstPtr& incoming_img){
    ROS_DEBUG("Image Right received.");
    #if ROS_VERSION_MINIMUM(1, 9, 0)
    try
    {
      _capture_right = cv_bridge::toCvCopy (incoming_img, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    #else
    ROS_ERROR("Your ROS Version is not supported.");
    #endif
    _ir_received=true;
    
    
    //_pub.publish(incoming_img);
    
}

void ARStereo::cameraInfoLeftCallback(const sensor_msgs::CameraInfoConstPtr & cam_info){
    ROS_INFO("CameraInfo Left received. Unsubscribing from Topic..");
    
    _cam_info_left_ros = (*cam_info);

    _cam_param_left_art.xsize = _cam_info_left_ros.width;
    _cam_param_left_art.ysize = _cam_info_left_ros.height;

    _cam_param_left_art.mat[0][0] = _cam_info_left_ros.K[0];
    _cam_param_left_art.mat[1][0] = _cam_info_left_ros.K[3];
    _cam_param_left_art.mat[2][0] = _cam_info_left_ros.K[6];
    _cam_param_left_art.mat[0][1] = _cam_info_left_ros.K[1];
    _cam_param_left_art.mat[1][1] = _cam_info_left_ros.K[4];
    _cam_param_left_art.mat[2][1] = _cam_info_left_ros.K[7];
    _cam_param_left_art.mat[0][2] = _cam_info_left_ros.K[2];
    _cam_param_left_art.mat[1][2] = _cam_info_left_ros.K[5];
    _cam_param_left_art.mat[2][2] = _cam_info_left_ros.K[8];
    _cam_param_left_art.mat[0][3] = 0;
    _cam_param_left_art.mat[1][3] = 0;
    _cam_param_left_art.mat[2][3] = 0;

        _cam_param_left_art.dist_factor[6] = _cam_info_left_ros.K[2];       // x0 = cX from openCV calibration
    _cam_param_left_art.dist_factor[7] = _cam_info_left_ros.K[5];       // y0 = cY from openCV calibration
    if ( _cam_info_left_ros.distortion_model == "plumb_bob" && _cam_info_left_ros.D.size() == 5){
      //_cam_param_left_art.dist_factor[2] = -100*_cam_info_left_ros.D[0];// f = -100*k1 from CV. Note, we had to do mm^2 to m^2, hence 10^8->10^2
      _cam_param_left_art.dist_factor[0]= _cam_info_left_ros.D[0];  //k0
      _cam_param_left_art.dist_factor[1]= _cam_info_left_ros.D[1];  //k1
      _cam_param_left_art.dist_factor[2]= _cam_info_left_ros.D[2];  //p0
      _cam_param_left_art.dist_factor[3]= _cam_info_left_ros.D[3];  //p1
      _cam_param_left_art.dist_factor[4]= _cam_info_left_ros.K[0];  //fx
      _cam_param_left_art.dist_factor[5]= _cam_info_left_ros.K[4];  //fy
    }
    else{
      //_cam_param_left_art.dist_factor[2] = 0;                  // We don't know the right value, so ignore it
        ROS_ERROR("Camera left: Distortion Parameters from ROS MSG mismatch ARToolkit Model");
    }
    _cam_param_left_art.dist_factor[8] = 1.0;                  // scale factor
    
    _cam_param_left_art.dist_function_version=4;
    
    ROS_INFO("Fake camera_info msg for left camera");
    cameraInfoRightCallback(cam_info);
    _cil_received=true;
    //unsibscribe from topic
    _sub_cil.shutdown();  
}

void ARStereo::cameraInfoRightCallback(const sensor_msgs::CameraInfoConstPtr & cam_info){
    ROS_INFO("CameraInfo Right received. Unsubscribing from Topic..");
    
    _cam_info_right_ros = (*cam_info);

    _cam_param_right_art.xsize = _cam_info_right_ros.width;
    _cam_param_right_art.ysize = _cam_info_right_ros.height;

    _cam_param_right_art.mat[0][0] = _cam_info_right_ros.K[0];
    _cam_param_right_art.mat[1][0] = _cam_info_right_ros.K[3];
    _cam_param_right_art.mat[2][0] = _cam_info_right_ros.K[6];
    _cam_param_right_art.mat[0][1] = _cam_info_right_ros.K[1];
    _cam_param_right_art.mat[1][1] = _cam_info_right_ros.K[4];
    _cam_param_right_art.mat[2][1] = _cam_info_right_ros.K[7];
    _cam_param_right_art.mat[0][2] = _cam_info_right_ros.K[2];
    _cam_param_right_art.mat[1][2] = _cam_info_right_ros.K[5];
    _cam_param_right_art.mat[2][2] = _cam_info_right_ros.K[8];
    _cam_param_right_art.mat[0][3] = 0;
    _cam_param_right_art.mat[1][3] = 0;
    _cam_param_right_art.mat[2][3] = 0;

        _cam_param_right_art.dist_factor[6] = _cam_info_right_ros.K[2];       // x0 = cX from openCV calibration
    _cam_param_right_art.dist_factor[7] = _cam_info_right_ros.K[5];       // y0 = cY from openCV calibration
    if ( _cam_info_right_ros.distortion_model == "plumb_bob" && _cam_info_right_ros.D.size() == 5){
      //_cam_param_right_art.dist_factor[2] = -100*_cam_info_right_ros.D[0];// f = -100*k1 from CV. Note, we had to do mm^2 to m^2, hence 10^8->10^2
      _cam_param_right_art.dist_factor[0]= _cam_info_right_ros.D[0];  //k0
      _cam_param_right_art.dist_factor[1]= _cam_info_right_ros.D[1];  //k1
      _cam_param_right_art.dist_factor[2]= _cam_info_right_ros.D[2];  //p0
      _cam_param_right_art.dist_factor[3]= _cam_info_right_ros.D[3];  //p1
      _cam_param_right_art.dist_factor[4]= _cam_info_right_ros.K[0];  //fx
      _cam_param_right_art.dist_factor[5]= _cam_info_right_ros.K[4];  //fy
    }
    else{
      //_cam_param_right_art.dist_factor[2] = 0;                  // We don't know the right value, so ignore it
        ROS_ERROR("Camera Right: Distortion Parameters from ROS MSG mismatch ARToolkit Model");
    }
    _cam_param_right_art.dist_factor[8] = 1.0;                  // scale factor
    
    _cam_param_right_art.dist_function_version=4;
    
    _cil_received=true;
    //unsibscribe from topic
    _sub_cir.shutdown();  
}

void ARStereo::mainLoop(void)
{
    
    if(_init){
        ROS_DEBUG("Start Tracking");

        AR2VideoBufferT *buffL, *buffR;
        ARMarkerInfo* markerInfoL;
        ARMarkerInfo* markerInfoR;
        int markerNumL;
        int markerNumR;
            ARdouble err;
        ARdouble transR[3][4];
        ARPose poseR;
        int             i, j, kL, kR;


            // Grab a video frame.
            //convert from opencv 
            gARTImageL = (ARUint8 *) ((IplImage) _capture_left->image).imageData;


            gARTImageR = (ARUint8 *) ((IplImage) _capture_left->image).imageData;

        if (gARTImageL && gARTImageR) {
                    gCallCountMarkerDetect++; // Increment ARToolKit FPS counter.

                    // Detect the markers in the video frame.
                    if (arDetectMarker(gARHandleL, gARTImageL) < 0 ||
                arDetectMarker(gARHandleR, gARTImageR) < 0) {
                            exit(-1);
                    }
                    
                    // Get detected markers
                    markerInfoL = arGetMarker(gARHandleL);
                    markerInfoR = arGetMarker(gARHandleR);
                    markerNumL = arGetMarkerNum(gARHandleL);
                    markerNumR = arGetMarkerNum(gARHandleR);

                    // Update markers.
                    for (i = 0; i < markersSquareCount; i++) {
                            markersSquare[i].validPrev = markersSquare[i].valid;
                markersSquare[i].valid = FALSE;

                            // Check through the marker_info array for highest confidence
                            // visible marker matching our preferred pattern.
                            kL = kR = -1;
                            if (markersSquare[i].patt_type == AR_PATTERN_TYPE_TEMPLATE) {
                                    for (j = 0; j < markerNumL; j++) {
                                            if (markersSquare[i].patt_id == markerInfoL[j].idPatt) {
                                                    if (kL == -1) {
                                                            if (markerInfoL[j].cfPatt >= markersSquare[i].matchingThreshold) kL = j; // First marker detected.
                                                    } else if (markerInfoL[j].cfPatt > markerInfoL[kL].cfPatt) kL = j; // Higher confidence marker detected.
                                            }
                                    }
                                    if (kL != -1) {
                                            ROS_INFO("Marker Detected");
                                            markerInfoL[kL].id = markerInfoL[kL].idPatt;
                                            markerInfoL[kL].cf = markerInfoL[kL].cfPatt;
                                            markerInfoL[kL].dir = markerInfoL[kL].dirPatt;
                                    }
                                    for (j = 0; j < markerNumR; j++) {
                                            if (markersSquare[i].patt_id == markerInfoR[j].idPatt) {
                                                    if (kR == -1) {
                                                            if (markerInfoR[j].cfPatt >= markersSquare[i].matchingThreshold) kR = j; // First marker detected.
                                                    } else if (markerInfoR[j].cfPatt > markerInfoR[kR].cfPatt) kR = j; // Higher confidence marker detected.
                                            }
                                    }
                                    if (kR != -1) {
                                        ROS_INFO("Marker Detected");
                                            markerInfoR[kR].id = markerInfoR[kR].idPatt;
                                            markerInfoR[kR].cf = markerInfoR[kR].cfPatt;
                                            markerInfoR[kR].dir = markerInfoR[kR].dirPatt;
                                    }
                            } else {
                                    for (j = 0; j < markerNumL; j++) {
                                            if (markersSquare[i].patt_id == markerInfoL[j].idMatrix) {
                                                    if (kL == -1) {
                                                            if (markerInfoL[j].cfMatrix >= markersSquare[i].matchingThreshold) kL = j; // First marker detected.
                                                    } else if (markerInfoL[j].cfMatrix > markerInfoL[kL].cfMatrix) kL = j; // Higher confidence marker detected.
                                            }
                                    }
                                    if (kL != -1) {
                                            markerInfoL[kL].id = markerInfoL[kL].idMatrix;
                                            markerInfoL[kL].cf = markerInfoL[kL].cfMatrix;
                                            markerInfoL[kL].dir = markerInfoL[kL].dirMatrix;
                                    }
                                    for (j = 0; j < markerNumR; j++) {
                                            if (markersSquare[i].patt_id == markerInfoR[j].idMatrix) {
                                                    if (kR == -1) {
                                                            if (markerInfoR[j].cfMatrix >= markersSquare[i].matchingThreshold) kR = j; // First marker detected.
                                                    } else if (markerInfoR[j].cfMatrix > markerInfoR[kR].cfMatrix) kR = j; // Higher confidence marker detected.
                                            }
                                    }
                                    if (kR != -1) {
                                            markerInfoR[kR].id = markerInfoR[kR].idMatrix;
                                            markerInfoR[kR].cf = markerInfoR[kR].cfMatrix;
                                            markerInfoR[kR].dir = markerInfoR[kR].dirMatrix;
                                    }
                            }

                            if (kL != -1 || kR != -1) {

                                if (kL != -1 && kR != -1) {
                                    err = arGetStereoMatchingErrorSquare(gAR3DStereoHandle, &markerInfoL[kL], &markerInfoR[kR]);
                                    //ARLOG("stereo err = %f\n", err);
                                    if (err > 16.0) {
                                        //ARLOG("Stereo matching error: %d %d.\n", markerInfoL[kL].area, markerInfoR[kR].area);
                                        if (markerInfoL[kL].area > markerInfoR[kR].area ) kR = -1;
                                        else                                              kL = -1;
                                    }
                                }

                                err = arGetTransMatSquareStereo(gAR3DStereoHandle, (kL == -1 ? NULL : &markerInfoL[kL]), (kR == -1 ?  NULL : &markerInfoR[kR]), markersSquare[i].marker_width, markersSquare[i].trans);

                                if (err < 10.0) markersSquare[i].valid = TRUE;

                                if (kL == -1)      ROS_INFO("[%2d] right:      err = %f\n", i, err);
                                else if (kR == -1) ROS_INFO("[%2d] left:       err = %f\n", i, err);
                                else               ROS_INFO("[%2d] left+right: err = %f\n", i, err);

                            }

                            if (markersSquare[i].valid) {

                                    // Filter the pose estimate.
                                    if (markersSquare[i].ftmi) {
                                            if (arFilterTransMat(markersSquare[i].ftmi, markersSquare[i].trans, !markersSquare[i].validPrev) < 0) {
                                                    ARLOGe("arFilterTransMat error with marker %d.\n", i);
                                            }
                                    }

                                    if (!markersSquare[i].validPrev) {
                                            // Marker has become visible, tell any dependent objects.
                                    }
                                    // We have a new pose, so set that.
                                    tf::Transform tf;
                                    ARdouble ARquat[4];
                                    ARdouble ARpos[3];
                                    tf::Quaternion ROSquat;
                                    std::string frame_id="Patt.ID=";
                                    frame_id.append(boost::lexical_cast<std::string>(markersSquare[i].patt_id));
                                    
                                    arUtilMat2QuatPos(markersSquare[i].trans,ARquat,ARpos);
                                    ROSquat.setValue(-ARquat[0],-ARquat[1],-ARquat[2],ARquat[3]);
                                    tf.setOrigin(tf::Vector3(ARpos[0]*UnitAR2ROS,ARpos[1]*UnitAR2ROS,ARpos[2]*UnitAR2ROS));
                                    tf.setRotation(ROSquat);
                                    _tf_br.sendTransform(tf::StampedTransform(tf, ros::Time::now(), "camera" ,frame_id));
                                    
                                    ROS_INFO("%f  %f  %f  %f", markersSquare[i].trans[0][0],markersSquare[i].trans[0][1], markersSquare[i].trans[0][2],markersSquare[i].trans[0][3] );
                                    ROS_INFO("%f  %f  %f  %f", markersSquare[i].trans[1][0],markersSquare[i].trans[1][1], markersSquare[i].trans[1][2],markersSquare[i].trans[1][3] );
                                    ROS_INFO("%f  %f  %f  %f", markersSquare[i].trans[2][0],markersSquare[i].trans[2][1], markersSquare[i].trans[2][2],markersSquare[i].trans[2][3] );
                                    arglCameraViewRH((const ARdouble (*)[4])markersSquare[i].trans, markersSquare[i].pose.T, 1.0f /*VIEW_SCALEFACTOR*/);
                                    arUtilMatMul((const ARdouble (*)[4])transL2R, (const ARdouble (*)[4])markersSquare[i].trans, transR);
                                    arglCameraViewRH((const ARdouble (*)[4])transR, poseR.T, 1.0f /*VIEW_SCALEFACTOR*/);
                                    // Tell any dependent objects about the update.

                            } else {
                                    if (markersSquare[i].validPrev) {
                                            // Marker has ceased to be visible, tell any dependent objects.
                                    }
                            }
            }
                    // Tell GLUT the display has changed.
            } else {

            }
    }

}