#include <ARStereo.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

ARStereo::ARStereo(ros::NodeHandle & nh):
        _nh(nh),
        _it(_nh),
        _il_received(false),
        _cil_received(false),
        _ciln_received(true)
{

    //read param
    _nh.param<std::string>("camera",_cameraImageLeftTopic,"/pylon_camera_left_node");
    _nh.param<std::string>("roi",_cameraInfoLeftNewTopic,"/ar_pose/roi/camera_info");
    //init all parameters and so on and so forth
    //_pub = _it.advertise("test", 1);
    ROS_INFO("Subscribing to camera topic: %s", _cameraImageLeftTopic.data());
    _sub_il = _it.subscribe(_cameraImageLeftTopic+"/image_raw", 1, &ARStereo::imageLeftCallback, this);
    _sub_cil = _nh.subscribe(_cameraImageLeftTopic +"/camera_info",1,&ARStereo::cameraInfoLeftCallback ,this);
    //_sub_ciln = _nh.subscribe(_cameraInfoLeftNewTopic,1,&ARStereo::updateCameraInfo ,this);
    ROS_INFO("Advertising ROI topic: %s", _cameraInfoLeftNewTopic.data());
    _pub_ciln = _nh.advertise<sensor_msgs::CameraInfo>(_cameraInfoLeftNewTopic,1);
    ROS_INFO("AR_LABELING_WORK_SIZE: %d",AR_LABELING_WORK_SIZE);



}
    
void ARStereo::ARInit(){
    
    ROS_INFO_ONCE("Initializing ARToolkit..");

    if ((gCparamLTL = arParamLTCreate(&_cam_param_left_art, AR_PARAM_LT_DEFAULT_OFFSET)) == NULL) {
        ROS_ERROR("setupCamera(): Error: arParamLTCreate.\n");
    }
    
    // Init AR.
    gARPattHandle = arPattCreateHandle();
	if (!gARPattHandle) {
		ARLOGe("Error creating pattern handle.\n");
		exit(-1);
	}
    
    gARHandleL = arCreateHandle(gCparamLTL);
    if (!gARHandleL) {
        ROS_ERROR("Error creating AR handle.\n");
		exit(-1);
    }
    arPattAttach(gARHandleL, gARPattHandle);

    if (arSetPixelFormat(gARHandleL, AR_PIXEL_FORMAT_MONO) < 0) {
        ROS_ERROR("Error setting pixel format.\n");
		exit(-1);
    }
    
    gAR3DHandleL = ar3DCreateHandle(&gCparamLTL->param);
    if (!gAR3DHandleL ) {
        ROS_ERROR("Error creating 3D handle.\n");
		exit(-1);
    }

    arUtilMatInv((const ARdouble (*)[4])transL2R, transR2L);
    ROS_INFO_ONCE("transL2R:");
    ROS_INFO_ONCE("",arParamDispExt(transL2R));
    ROS_INFO_ONCE("transR2L:");
    ROS_INFO_ONCE("",arParamDispExt(transR2L));
    ROS_INFO_ONCE("Left Camera Calibration:",arParamDisp((const ARParam*) &_cam_param_left_art));
    ROS_INFO_ONCE("",arParamDisp((const ARParam*) &_cam_param_left_art));


    //
    // Markers setup.
    //
    
    // Load marker(s).
    newMarkers(markerConfigDataFilename.c_str(), gARPattHandle, &markersSquare, &markersSquareCount, &gARPattDetectionMode);
    ROS_INFO_ONCE("Marker count = %d\n", markersSquareCount);
    
    // 
    // Other ARToolKit setup.
    //

    arSetMarkerExtractionMode(gARHandleL, AR_USE_TRACKING_HISTORY_V2);
    //arSetMarkerExtractionMode(gARHandleL, AR_NOUSE_TRACKING_HISTORY);
    //arSetMarkerExtractionMode(gARHandleR, AR_NOUSE_TRACKING_HISTORY);
    //set automatic thresholding   AR_LABELING_THRESH_MODE_AUTO_OTSU
    arSetLabelingThreshMode(gARHandleL,AR_LABELING_THRESH_MODE_AUTO_ADAPTIVE);
    //recompute treshold every 6 frames
    arSetLabelingThreshModeAutoInterval(gARHandleL,1);


    //arSetLabelingThreshMode(gARHandleL,AR_LABELING_THRESH_MODE_AUTO_OTSU);

    arSetDebugMode(gARHandleL,AR_DEBUG_ENABLE);
    //arSetLabelingThreshMode(gARHandleL,AR_LABELING_THRESH_MODE_AUTO_BRACKETING);
    //gARHandleL->arLabelingThreshAutoBracketOver=30;
    //gARHandleL->arLabelingThreshAutoBracketOver=30;
    //arSetLabelingThresh(gARHandleL,100);
    
    //arSetMarkerExtractionMode(gARHandleL, AR_NOUSE_TRACKING_HISTORY);
    //arSetMarkerExtractionMode(gARHandleR, AR_NOUSE_TRACKING_HISTORY);
    //arSetLabelingThresh(gARHandleL,100);
    //arSetLabelingThreshMode(gARHandleL, AR_LABELING_THRESH_MODE_MANUAL); // Uncomment to force manual thresholding.
    //arSetLabelingThreshMode(gARHandleR, AR_LABELING_THRESH_MODE_MANUAL); // Uncomment to force manual thresholding.
    
    // Set the pattern detection mode (template (pictorial) vs. matrix (barcode) based on
    // the marker types as defined in the marker config. file.
    arSetPatternDetectionMode(gARHandleL, AR_TEMPLATE_MATCHING_MONO); // Default = AR_TEMPLATE_MATCHING_COLOR

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
    arPattDetach(gARHandleL);
    arPattDeleteHandle(gARPattHandle);
    ar3DDeleteHandle(&gAR3DHandleL);
    arDeleteHandle(gARHandleL);
    arParamLTFree(&gCparamLTL);
}

void ARStereo::arParamUpdate(ARHandle *handle,ARParam *param) {
    if(arPattDetach(gARHandleL)<0) ROS_ERROR("arPattDetach: no success");
    if(arDeleteHandle(gARHandleL)<0) ROS_ERROR("arDeleteHandle: no success");
    //if(arPattDeleteHandle(gARPattHandle)<0) ROS_ERROR("arPattDeleteHandle: no success");
    if(ar3DDeleteHandle(&gAR3DHandleL)<0)ROS_ERROR("ar3DDeleteHandle: no success");
    if(arParamLTFree(&gCparamLTL)<0) ROS_ERROR("arParamLTFree: no success");


    if ((gCparamLTL = arParamLTCreate(&_cam_param_left_art, AR_PARAM_LT_DEFAULT_OFFSET)) == NULL) {
        ROS_ERROR("setupCamera(): Error: arParamLTCreate.\n");
    }


    gARHandleL = arCreateHandle(gCparamLTL);
    if (!gARHandleL) {
        ROS_ERROR("Error creating AR handle.\n");
    }

        arPattAttach(gARHandleL, gARPattHandle);

    if (arSetPixelFormat(gARHandleL, AR_PIXEL_FORMAT_MONO) < 0) {
        ROS_ERROR("Error setting pixel format.\n");
        exit(-1);
    }

    gAR3DHandleL = ar3DCreateHandle(&gCparamLTL->param);
    if (!gAR3DHandleL ) {
        ROS_ERROR("Error creating 3D handle.\n");
        exit(-1);
    }
    arSetMarkerExtractionMode(gARHandleL, AR_NOUSE_TRACKING_HISTORY);
    arSetLabelingThreshMode(gARHandleL,AR_LABELING_THRESH_MODE_AUTO_ADAPTIVE);
    arSetLabelingThreshModeAutoInterval(gARHandleL,1);
    arSetDebugMode(gARHandleL,AR_DEBUG_ENABLE);
    arSetPatternDetectionMode(gARHandleL, AR_TEMPLATE_MATCHING_MONO); // Default = AR_TEMPLATE_MATCHING_COLOR

}


void ARStereo::imageLeftCallback(const sensor_msgs::ImageConstPtr& incoming_img){
    //check if camera info was received
    if(!_cil_received){
        ROS_DEBUG("imageLeftCallback:\twaiting for camera_info");
        return;
    }

    ROS_INFO("Image Left received.");

    try
    {
        if(_cam_info_left_ros.roi.height>0 && _cam_info_left_ros.roi.width>0){
            _capture_left = cv_bridge::toCvCopy (incoming_img, sensor_msgs::image_encodings::MONO8);
            _capture_left->image=_capture_left->image(_cam_model_left.rawRoi()).clone();
            ROS_INFO("Mat Size: %d  %d",_capture_left->image.rows ,_capture_left->image.cols);
        }else{
            _capture_left = cv_bridge::toCvCopy (incoming_img, sensor_msgs::image_encodings::MONO8);

        }
        if(image_scale<1){
            //resize image
            cv::Size size_new(_cam_param_left_art.xsize,_cam_param_left_art.ysize);
            cv::resize(_capture_left->image,_capture_left->image,size_new,cv::INTER_AREA);
            std::cout<<"New Size: "<<size_new<<std::endl;
        }

    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    //set capture time
    _capture_time_left=incoming_img->header.stamp;

    _il_received=true;

    //_pub.publish(incoming_img);
    //imageRightCallback(incoming_img);
    
}



void ARStereo::cameraInfoLeftCallback(const sensor_msgs::CameraInfoConstPtr & cam_info){
    ROS_DEBUG("CameraInfo Left received.");
    image_geometry::PinholeCameraModel cam_model_tmp;
    cam_model_tmp.fromCameraInfo(cam_info);

    if(_cam_model_left.rawRoi()==cam_model_tmp.rawRoi() && _cam_model_left.initialized() && _cam_model_left.intrinsicMatrix()==cam_model_tmp.intrinsicMatrix() && _cil_received==true){
        ROS_DEBUG("cameraInfoLeftCallback: camera_info received but not updated since it has not changed");
        return;
    }else{
        ROS_DEBUG("cameraInfoLeftCallback:\tcamera info has changed.");
        //std::cout<<cam_model_tmp.cameraInfo()<<std::endl;
    }
    //copy new camera info and create pinhole model
    _cam_info_left_ros = (*cam_info);
    _cam_model_left=cam_model_tmp;


    //if roi active adjust the intrinsics and update

    memcpy(_cam_param_left_art.mat,&_cam_model_left.intrinsicMatrix()(0,0), 3*sizeof(double));
    memcpy(&_cam_param_left_art.mat[1][0],&_cam_model_left.intrinsicMatrix()(1,0), 3*sizeof(double));
    memcpy(&_cam_param_left_art.mat[2][0],&_cam_model_left.intrinsicMatrix()(2,0), 3*sizeof(double));

    //adjusting the scale
    _cam_param_left_art.mat[0][0] *= image_scale; //fx*scale
    _cam_param_left_art.mat[0][2] *= image_scale; //cx*scale
    _cam_param_left_art.mat[1][1] *= image_scale; //fy*scale
    _cam_param_left_art.mat[1][2] *= image_scale; //cy*scale


    _cam_param_left_art.xsize = (int)round(_cam_model_left.reducedResolution().width*image_scale);
    _cam_param_left_art.ysize = (int)round(_cam_model_left.reducedResolution().height*image_scale);


    //set 0 to the last column
    _cam_param_left_art.mat[0][3] = 0;
    _cam_param_left_art.mat[1][3] = 0;
    _cam_param_left_art.mat[2][3] = 0;


    _cam_param_left_art.dist_factor[6] = (int)(_cam_model_left.cx()*image_scale);//_cam_info_left_ros.K[2];       // x0 = cX from openCV calibration
    _cam_param_left_art.dist_factor[7] = (int)(_cam_model_left.cy()*image_scale);//_cam_info_left_ros.K[5];       // y0 = cY from openCV calibration
    if ( _cam_info_left_ros.distortion_model == "plumb_bob" && _cam_info_left_ros.D.size() == 5){
      _cam_param_left_art.dist_factor[0]= _cam_info_left_ros.D[0];  //k0
      _cam_param_left_art.dist_factor[1]= _cam_info_left_ros.D[1];  //k1
      _cam_param_left_art.dist_factor[2]= _cam_info_left_ros.D[2];  //p0
      _cam_param_left_art.dist_factor[3]= _cam_info_left_ros.D[3];  //p1
      _cam_param_left_art.dist_factor[4]= _cam_model_left.fx()*image_scale; // _cam_info_left_ros.K[0];  //fx
      _cam_param_left_art.dist_factor[5]= _cam_model_left.fy()*image_scale;//_cam_info_left_ros.K[4];  //fy
    }
    else{
      //_cam_param_left_art.dist_factor[2] = 0;                  // We don't know the right value, so ignore it
        ROS_ERROR("Camera left: Distortion Parameters from ROS MSG mismatch ARToolkit Model");
    }
    _cam_param_left_art.dist_factor[8] = 1.0;                  // scale factor
    
    _cam_param_left_art.dist_function_version=4;

    ROS_INFO_ONCE("Continue with AR initialization..");
    ARInit();
    ROS_INFO_ONCE("ARToolkit initialized");
    _init=true;


    //ROS_INFO("Fake camera_info msg for left camera");
    //cameraInfoRightCallback(cam_info);

    _cil_received=true;
    //unsibscribe from topic
    ROS_INFO("Camera Info received. Unsubscribing from topic..");
    _sub_cil.shutdown();
}

void ARStereo::updateCameraInfo(const sensor_msgs::CameraInfo &cam_info){
    ROS_DEBUG("CameraInfo LeftNew received.");
    image_geometry::PinholeCameraModel cam_model_tmp;
    cam_model_tmp.fromCameraInfo(cam_info);

    if(_cam_model_left.rawRoi()==cam_model_tmp.rawRoi() && _cam_model_left.initialized() && _cam_model_left.intrinsicMatrix()==cam_model_tmp.intrinsicMatrix() && _ciln_received){
        ROS_DEBUG("updateCameraInfo: camera_info received but not updated since it has not changed");
        return;
    }else{
        ROS_INFO("updateCameraInfo:\tcamera info has changed.");
        //std::cout<<cam_model_tmp.cameraInfo()<<std::endl;
    }
    //copy new camera info and create pinhole model
    _cam_info_left_ros = (cam_info);
    _cam_model_left=cam_model_tmp;

    //if roi active adjust the intrinsics and update

    memcpy(_cam_param_left_art.mat,&_cam_model_left.intrinsicMatrix()(0,0), 3*sizeof(double));
    memcpy(&_cam_param_left_art.mat[1][0],&_cam_model_left.intrinsicMatrix()(1,0), 3*sizeof(double));
    memcpy(&_cam_param_left_art.mat[2][0],&_cam_model_left.intrinsicMatrix()(2,0), 3*sizeof(double));

    //adjusting the scale
    _cam_param_left_art.mat[0][0] *= image_scale;
    _cam_param_left_art.mat[0][2] *= image_scale;
    _cam_param_left_art.mat[1][1] *= image_scale;
    _cam_param_left_art.mat[1][2] *= image_scale;

    //compute new image size
    _cam_param_left_art.xsize = (int)round(_cam_model_left.reducedResolution().width*image_scale);
    _cam_param_left_art.ysize = (int)round(_cam_model_left.reducedResolution().height*image_scale);


    //set 0 to the last column
    _cam_param_left_art.mat[0][3] = 0;
    _cam_param_left_art.mat[1][3] = 0;
    _cam_param_left_art.mat[2][3] = 0;

    _cam_param_left_art.dist_factor[6] = (int)(_cam_model_left.cx()*image_scale);//_cam_info_left_ros.K[2];       // x0 = cX from openCV calibration
    _cam_param_left_art.dist_factor[7] = (int)(_cam_model_left.cy()*image_scale);//_cam_info_left_ros.K[5];       // y0 = cY from openCV calibration

    if ( _cam_info_left_ros.distortion_model == "plumb_bob" && _cam_info_left_ros.D.size() == 5){
        _cam_param_left_art.dist_factor[0]= _cam_info_left_ros.D[0];  //k0
        _cam_param_left_art.dist_factor[1]= _cam_info_left_ros.D[1];  //k1
        _cam_param_left_art.dist_factor[2]= _cam_info_left_ros.D[2];  //p0
        _cam_param_left_art.dist_factor[3]= _cam_info_left_ros.D[3];  //p1
        _cam_param_left_art.dist_factor[4]= _cam_model_left.fx()*image_scale; // _cam_info_left_ros.K[0];  //fx
        _cam_param_left_art.dist_factor[5]= _cam_model_left.fy()*image_scale;//_cam_info_left_ros.K[4];  //fy
    }
    else{
        //_cam_param_left_art.dist_factor[2] = 0;                  // We don't know the right value, so ignore it
        ROS_ERROR("Camera left: Distortion Parameters from ROS MSG mismatch ARToolkit Model");
    }
    _cam_param_left_art.dist_factor[8] = 1.0;                  // scale factor

    _cam_param_left_art.dist_function_version=4;

    arParamUpdate(gARHandleL,&_cam_param_left_art);
    //ARInit();


    _ciln_received=true;
    //unsibscribe from topic
    //_sub_cil.shutdown();
}



void ARStereo::mainLoop(void)
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
        if(!_il_received){
            ROS_DEBUG("Waiting for image..");
            return;
        }else{
            _il_received=false;
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
         gARTImageL = (ARUint8 *) ((IplImage) _capture_left->image).imageData;
        //update parameters
        if (gARTImageL) {
            //cv::imshow("test ",_capture_left->image);
            //cv::waitKey(0);
            gCallCountMarkerDetect++; // Increment ARToolKit FPS counter.

            // Detect the markers in the video frame.
            if (arDetectMarker(gARHandleL, gARTImageL) < 0) {

                exit(-1);
            }



            // Get detected markers
            markerInfoL = arGetMarker(gARHandleL);
            markerNumL = arGetMarkerNum(gARHandleL);

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
                       // ROS_INFO("Marker Left Cam Detected");
                        markerInfoL[kL].id = markerInfoL[kL].idPatt;
                        markerInfoL[kL].cf = markerInfoL[kL].cfPatt;
                        markerInfoL[kL].dir = markerInfoL[kL].dirPatt;
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

                }

                if (kL != -1 ) {

                    err = arGetTransMatSquare(gAR3DHandleL,&markerInfoL[kL],markersSquare[i].marker_width, markersSquare[i].trans);

                    if (err < 10.0) markersSquare[i].valid = TRUE;


                }

                if (markersSquare[i].valid) {
                    // Filter the pose estimate.
                    if (markersSquare[i].ftmi) {
                        if (arFilterTransMat(markersSquare[i].ftmi, markersSquare[i].trans, !markersSquare[i].validPrev) < 0) {
                            ARLOGe("arFilterTransMat error with marker %d.\n", i);
                        }
                    }

                    // We have a new pose, so set that.
                    tf::Transform tf;
                    ARdouble ARquat[4];
                    ARdouble ARpos[3];
                    tf::Quaternion ROSquat;
                    std::string frame_id="Patt.ID=";
                    frame_id.append(boost::lexical_cast<std::string>(markersSquare[i].patt_id));
                    arUtilMat2QuatPos(markersSquare[i].trans,ARquat,ARpos);
                    //ROSquat.setValue(-ARquat[0],-ARquat[1],-ARquat[2],ARquat[3]);
                    ROSquat.setValue(ARquat[0],ARquat[1],ARquat[2],ARquat[3]);
                    tf.setOrigin(tf::Vector3(ARpos[0]*UnitAR2ROS,ARpos[1]*UnitAR2ROS,ARpos[2]*UnitAR2ROS));
                    tf.setRotation(ROSquat);
                    _tf_br.sendTransform(tf::StampedTransform(tf, _capture_time_left, "pylon_camera_left_node" ,frame_id));
                    //ROS_INFO("%f  %f  %f  %f", markersSquare[i].trans[0][0],markersSquare[i].trans[0][1], markersSquare[i].trans[0][2],markersSquare[i].trans[0][3] );
                    //ROS_INFO("%f  %f  %f  %f", markersSquare[i].trans[1][0],markersSquare[i].trans[1][1], markersSquare[i].trans[1][2],markersSquare[i].trans[1][3] );
                    //ROS_INFO("%f  %f  %f  %f", markersSquare[i].trans[2][0],markersSquare[i].trans[2][1], markersSquare[i].trans[2][2],markersSquare[i].trans[2][3] );
                    arglCameraViewRH((const ARdouble (*)[4])markersSquare[i].trans, markersSquare[i].pose.T, 1.0f /*VIEW_SCALEFACTOR*/);
                    arUtilMatMul((const ARdouble (*)[4])transL2R, (const ARdouble (*)[4])markersSquare[i].trans, transR);
                    arglCameraViewRH((const ARdouble (*)[4])transR, poseR.T, 1.0f /*VIEW_SCALEFACTOR*/);
                    // Tell any dependent objects about the update.
                    time_elapsed=(ros::Time::now()-time_prev);
                    framerate=(1.0/(float)time_elapsed.toSec());
                    time_prev=ros::Time::now();
                    ROS_INFO("[%2d] Result:\terr: %.2f\tposXYZ: [%.2f,\t%.2f,\t%.2f]\t%.2f fps\n", i, err,tf.getOrigin().x(),tf.getOrigin().y(),tf.getOrigin().z(),framerate);
                    //marker has been detected, now reduce scale of the image and
                    //if (markersSquare[i].validPrev && i== selected_marker) {
                    if ( i== selected_marker) {

                        ///compute new scale
                        //get vertex and compute average length of pixel of the marker in the image
                        double x=markersSquare[i].marker_height/1000.0;
                        double y=0;
                        double z=tf.getOrigin().length();
                        cv::Point3d p3d(x,0,z);
                        cv::Point2d p2d= _cam_model_left.project3dToPixel(p3d);
                        //ROS_INFO("P2D roi [%.1f,%.1f]",p2d.x,p2d.y);
                        //p2d = _cam_model_left.unrectifyPoint(p2d);
                        //ROS_INFO("P2D roi unrectified [%.1f,%.1f]",p2d.x,p2d.y);

                        p2d=_cam_model_left.toFullResolution(p2d);
                        //ROS_INFO("P2D full [%.1f,%.1f]",p2d.x,p2d.y);

                        p2d.x-=_cam_model_left.fullIntrinsicMatrix()(0,2);
                        p2d.y-=_cam_model_left.fullIntrinsicMatrix()(1,2);
                        //ROS_INFO("P2D final [%.1f,%.1f]",p2d.x,p2d.y);
                        int l= abs(p2d.x);
                        if(l>marker_size_max){
                            image_scale=(float)(marker_size_max)/(float)l;
                            ROS_INFO("New scale= %0.2f , l= %d",image_scale,l);
                        }else{
                            image_scale=SCALE_DEFAULT;
                        }
                        //set roi around marker
                        p3d=cv::Point3d(tf.getOrigin().x(),
                                        tf.getOrigin().y(),
                                        tf.getOrigin().z());
                        //predict new position
                        ros::Duration dt=_capture_time_left-_capture_time_prev;
                        tf_prev.setOrigin(tf.getOrigin()-tf_prev.getOrigin());
                        //tf_prev.getOrigin().normalize();
                        tf::Vector3 grad=tf_prev.getOrigin();
                        ROS_INFO("POS 3d=  [%.2f, %.2f, %.2f]",p3d.x,p3d.y,p3d.z);
                        if(dt.toSec()<0.3){
                            p3d=p3d+cv::Point3d(grad.x(),
                                                grad.y(),
                                                grad.z());
                            ROS_INFO("POS pre= [%.2f, %.2f, %.2f]",p3d.x,p3d.y,p3d.z);
                        }
                        tf_prev=tf;
                        _capture_time_prev=_capture_time_left;
                        p2d=_cam_model_left.project3dToPixel(p3d);
                        p2d=_cam_model_left.toFullResolution(p2d);
                        p2d=_cam_model_left.unrectifyPoint(p2d);
                        ROS_DEBUG("Setting new ROI");
                        //std::cout<<"p2d: "<<p2d<<"\np3d: "<<p3d<<std::endl;
                        _cam_info_left_ros.roi.x_offset=(uint)cv::max(0,(int)p2d.x-marker_roi_x-l);
                        _cam_info_left_ros.roi.y_offset=(uint)cv::max(0,(int)p2d.y-marker_roi_y-l);
                        _cam_info_left_ros.roi.height=cv::min((int)(_cam_model_left.fullResolution().height-_cam_info_left_ros.roi.y_offset),2*(marker_roi_x+l));
                        _cam_info_left_ros.roi.width=cv::min((int)(_cam_model_left.fullResolution().width-_cam_info_left_ros.roi.x_offset),2*(marker_roi_x+l));
                        _pub_ciln.publish(_cam_info_left_ros);
                        updateCameraInfo(_cam_info_left_ros);
                        return;

                    }
                }
                else {

                     if (!markersSquare[i].validPrev && i== selected_marker){
                         //reset scale
                         static int cnt_scale=0;
                         if(cnt_scale>=SCALE_WAIT_RESET){
                             ROS_INFO("Reset scale");
                             image_scale=SCALE_DEFAULT;
                             cnt_scale=0;
                             _ciln_received=false;
                         }else{
                             cnt_scale++;
                         }

                         //reset roi
                         static int cnt_roi=0;
                         if(cnt_roi>=ROI_WAIT_RESET){

                             //check if roi is set
                            if(_cam_info_left_ros.roi.width>0 && _cam_info_left_ros.roi.height>0) {

                                //compute new size of roi
                                ROS_INFO("Reset ROI");
                                int height, width, x, y, x_size, y_size;
                                height = (int)((_cam_info_left_ros.roi.height +  (2 * ROI_REGION_GROW_Y)) );
                                width = (int)((_cam_info_left_ros.roi.width +  (2 * ROI_REGION_GROW_X )) );
                                x = (int)((_cam_info_left_ros.roi.x_offset -  (ROI_REGION_GROW_X)) );
                                y = (int)((_cam_info_left_ros.roi.y_offset - (ROI_REGION_GROW_Y)) );
                                x_size =  _cam_model_left.fullResolution().width ;
                                y_size =  _cam_model_left.fullResolution().height ;

                                //check if new region is within the width of the image and adapt the size
                                _cam_info_left_ros.roi.x_offset=x =     cv::max(0,x);
                                _cam_info_left_ros.roi.y_offset=y =     cv::max(0,y);
                                _cam_info_left_ros.roi.width =width=    cv::min(x_size - x, width);
                                _cam_info_left_ros.roi.height =height=  cv::min(y_size-y,height);


                                //if roi is the whole image reset all values to 0
                                if ((_cam_info_left_ros.roi.height == y_size &&
                                     _cam_info_left_ros.roi.width == x_size) || _cam_info_left_ros.roi.width == 0 ||
                                    _cam_info_left_ros.roi.height == 0) {
                                    _cam_info_left_ros.roi.width = 0;
                                    _cam_info_left_ros.roi.height = 0;
                                    _cam_info_left_ros.roi.x_offset = 0;
                                    _cam_info_left_ros.roi.y_offset = 0;
                                }

                                ROS_INFO("NEW ROI: x:%d y:%d w:%d h:%d", _cam_info_left_ros.roi.x_offset,
                                         _cam_info_left_ros.roi.y_offset,
                                         _cam_info_left_ros.roi.width, _cam_info_left_ros.roi.height);
                                _pub_ciln.publish(_cam_info_left_ros);
                                _ciln_received=false;
                            }

                             cnt_roi=0;
                         }else{
                             cnt_roi++;
                         }

                         //if something was changed update the parameters
                         if(_ciln_received==false){
                             updateCameraInfo(_cam_info_left_ros);
                             return;
                         }

                        //std::cout<<_cam_model_left.rawRoi()<<std::endl;


                    }
                }
            }
                // Tell GLUT the display has changed.
            } else {

        }
    }

}


void ARStereo::safeMarker(ARMarkerInfo *target)
{
    char   name1[256], name2[256];
    printf("Enter filename: ");
    //if( fgets(name1, 256, stdin) == NULL ) return;
    //if( sscanf(name1, "%s", name2) != 1 ) return;
    if( arPattSave(gARTImageL, gARHandleL->xsize, gARHandleL->ysize, gARHandleL->arPixelFormat, &(gARHandleL->arParamLT->paramLTf),
                   gARHandleL->arImageProcMode, target, 0.5, 16, "/home/tman/ros_ws/src/ar_pose_stereo/Data/furbot.patt") < 0 ) {
        ARLOGe("ERROR!!\n");
    }
    else {
        ARLOG("  Saved\n");
    }

}
