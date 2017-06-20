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
#include <opencv2/objdetect/objdetect.hpp>

namespace fs = ::boost::filesystem;

class SampleExtractor {

public:
    SampleExtractor();
    SampleExtractor(cv::Size rect_size,cv::Size2d obj_size,cv::Size2d boarder_size_percent, std::string path_root, tf::Transform marker2obj,
                    sensor_msgs::CameraInfo cam_info);
    bool update(tf::Transform cam_marker, cv::Mat img, bool rectify=false, bool verifyRectSize=false);

    void extractPositivePatch(std::string path,std::string file_type=".jpg", bool overwrite =false);
    void extractNegativePatch(std::string path,std::string file_type=".jpg", bool overwrite =false);
    bool initDetection(std::string path_hog);
    void extractFalsePositives(std::string path,std::string file_type=".jpg", bool overwrite =false);


private:
    long _cnt_pos;
    long _cnt_neg;
    long _cnt_false_positive;
    bool _init=false;
    bool _init_detection=false;
    tf::Transform _marker2obj;
    tf::Transform _cam2marker;
    cv::Size _rect_size;
    cv::Size2d _boarder_size_percent;
    cv::Size2d _obj_size;
    cv::Mat _img;
    cv::Rect _bb_scaled;
    cv::HOGDescriptor _hog;
    image_geometry::PinholeCameraModel _cam_model;
    std::string _path_root;
    bool openDir(std::string path, std::vector<fs::path> & file_list, std::string file_format);

    bool getBoundingRect(cv::Rect & bb);
    bool getScale(cv::Rect bb, double & scale, cv::Rect& bb_scaled,bool validateSize);

};


#endif //AR_TRACKER_SAMPLEEXTRACTOR_H
