//
// Created by tman on 5/23/17.
//

#include <RoiTracker.h>
#include <ros/ros.h>

RoiTracker::RoiTracker() {

}

void RoiTracker::fromCameraInfo(sensor_msgs::CameraInfo camera_info) {
    cam_model.fromCameraInfo(camera_info);
}

sensor_msgs::CameraInfo RoiTracker::getCameraInfo() {
    return cam_model.cameraInfo();
}

image_geometry::PinholeCameraModel RoiTracker::getCameraModel() {
    return cam_model;
}

bool
RoiTracker::updateRoi(tf::Vector3 pos3d, int roi_width, int roi_height,double marker_size, ros::Time now, bool predict, float dt_max) {
    ros::Duration dt=now-time_call_last;
    cv::Point3d pos3d_pred(pos3d.x(),pos3d.y(),pos3d.z());

    //create 3d point with abs length to the camera in z direction and x with the marker size in [m]
    cv::Point3d p3d_length(marker_size,0,pos3d.length());
    //project to 2d
    cv::Point2d p2d_length= cam_model.project3dToPixel(p3d_length);
    p2d_length=cam_model.toFullResolution(p2d_length);
    //ROS_INFO("P2D full [%.1f,%.1f]",p2d.x,p2d.y);
    p2d_length.x-=cam_model.fullIntrinsicMatrix()(0,2);
    p2d_length.y-=cam_model.fullIntrinsicMatrix()(1,2);
    //ROS_INFO("P2D final [%.1f,%.1f]",p2d.x,p2d.y);
    int l= abs(p2d_length.x);

    if(predict){
        tf::Vector3 grad=pos3d-tf_prev;
        //ROS_INFO("POS 3d=  [%.2f, %.2f, %.2f]",pos3d.x,pos3d.y,pos3d.z);
        if(dt.toSec()<dt_max){
            pos3d_pred=pos3d_pred+cv::Point3d(grad.x(),
                                              grad.y(),
                                              grad.z());
            //ROS_INFO("POS pre= [%.2f, %.2f, %.2f]",pos3d.x,pos3d.y,pos3d.z);
        }
    }

    tf_prev=pos3d;
    time_call_last=now;
    cv::Point2d p2d=cam_model.project3dToPixel(pos3d_pred);
    p2d=cam_model.toFullResolution(p2d);
    p2d=cam_model.unrectifyPoint(p2d);
    //ROS_DEBUG("Setting new ROI");
    sensor_msgs::CameraInfo cam_info=getCameraInfo();
    cam_info.roi.x_offset=(uint)cv::max(0,(int)(p2d.x-roi_width-l/2.0));
    cam_info.roi.y_offset=(uint)cv::max(0,(int)(p2d.y-roi_height-l/2.0));
    cam_info.roi.height=cv::min((int)(cam_model.fullResolution().height-cam_info.roi.y_offset),(int)(2*(roi_width+l)));
    cam_info.roi.width=cv::min((int)(cam_model.fullResolution().width-cam_info.roi.x_offset),(int)(2*(roi_height+l)));



    cam_model.fromCameraInfo(cam_info);

    return true;
}

bool RoiTracker::enlargeRoi(int step_x, int step_y, int every_n_iter) {

    static int cnt_roi=0;
    if(cnt_roi>=every_n_iter){
        sensor_msgs::CameraInfo cam_info=cam_model.cameraInfo();
        //check if roi is set
        if(cam_info.roi.width>0 && cam_info.roi.height>0) {

            //compute new size of roi
            ROS_DEBUG("Grow Roi");
            int height, width, x, y, x_size, y_size;
            height = (int)((cam_info.roi.height +  (2 * step_y)) );
            width = (int)((cam_info.roi.width +  (2 * step_x )) );
            x = (int)((cam_info.roi.x_offset -  (step_x)) );
            y = (int)((cam_info.roi.y_offset - (step_y)) );
            x_size =  cam_model.fullResolution().width ;
            y_size =  cam_model.fullResolution().height ;

            //check if new region is within the width of the image and adapt the size
            cam_info.roi.x_offset=x =     cv::max(0,x);
            cam_info.roi.y_offset=y =     cv::max(0,y);
            cam_info.roi.width =width=    cv::min(x_size - x, width);
            cam_info.roi.height =height=  cv::min(y_size-y,height);


            //if roi is the whole image reset all values to 0
            if ((cam_info.roi.height == y_size &&
                    cam_info.roi.width == x_size) || cam_info.roi.width == 0 ||
                    cam_info.roi.height == 0) {
                cam_info.roi.width = 0;
                cam_info.roi.height = 0;
                cam_info.roi.x_offset = 0;
                cam_info.roi.y_offset = 0;
            }

            ROS_DEBUG("NEW ROI: x:%d y:%d w:%d h:%d", cam_info.roi.x_offset,
                     cam_info.roi.y_offset,
                     cam_info.roi.width, cam_info.roi.height);
            cam_model.fromCameraInfo(cam_info);
            return true;
        }
        cnt_roi=0;
        return false;
    }else{
        cnt_roi++;
        return false;
    }


}
