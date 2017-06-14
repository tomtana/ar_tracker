//
// Created by tman on 6/14/17.
//

#ifndef AR_TRACKER_SAMPLEEXTRACTOR_H
#define AR_TRACKER_SAMPLEEXTRACTOR_H


#include <tf/tf.h>
#include <opencv2/core/core.hpp>
#include <string>
#include <image_geometry/pinhole_camera_model.h>
#include <boost/filesystem.hpp>


namespace fs = ::boost::filesystem;

class SampleExtractor {

public:
    SampleExtractor();
    SampleExtractor(cv::Size2d obj_size, std::string path_root, tf::Vector3 off_mar_obj,
                        sensor_msgs::CameraInfo cam_info);
    bool update(tf::Transform cam_marker, cv::Mat img, bool rectify=false, bool verifyRectSize=false);

    void extractPositivePatch(std::string path,std::string file_type=".jpg", bool overwrite =false);
    void extractNegativePatch(std::string path,std::string file_type=".jpg", bool overwrite =false);


private:
    long _cnt_pos;
    long _cnt_neg;
    bool _init=false;
    tf::Vector3 _off_mar_obj;
    tf::Transform _cam_marker;
    cv::Size2d _obj_size;
    cv::Mat _img;
    cv::Rect _bb;
    image_geometry::PinholeCameraModel _cam_model;
    std::string _path_root;
    bool openDir(std::string path, std::vector<fs::path> & file_list, std::string file_format);

    bool getBoundingRect(cv::Rect & bb);
    bool getScale(cv::Rect bb, double & scale, cv::Rect bb_scaled,bool validateSize);

};


#endif //AR_TRACKER_SAMPLEEXTRACTOR_H
