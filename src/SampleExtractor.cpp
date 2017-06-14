//
// Created by tman on 6/14/17.
//

#include "../include/ar_tracker/SampleExtractor.h"

#include <regex>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>


SampleExtractor::SampleExtractor() {

}

SampleExtractor::SampleExtractor(cv::Size2d obj_size, std::string path_root, tf::Vector3 off_mar_obj,
                                 sensor_msgs::CameraInfo cam_info) {
    _cam_model.fromCameraInfo(cam_info);

    if(_path_root.back()!='/'){
        _path_root.append("/");
    }
    _path_root=path_root;
    _obj_size=obj_size;
    _off_mar_obj=off_mar_obj;
    _cnt_neg=0;
    _cnt_pos=0;
}

bool SampleExtractor::update(tf::Transform cam_marker, cv::Mat img, bool rectify, bool verifyRectSize) {

    _init=false;

    if(!_cam_model.initialized()){
        return false;
    }

    _cam_marker=cam_marker;

    if (rectify){
        _cam_model.rectifyImage(img,_img);
    }else{
        _img=img.clone();
    }

    cv::Rect bb;
    cv::Rect bb_scaled;
    double scale;

    //rectangle enclosing object
    if(!getBoundingRect(bb)){
        return false;
    }

    //get scale and scaled rectangle enclosing object
    if(!getScale(bb,scale,bb_scaled,verifyRectSize)){
        return false;
    }


    //now resize image
    cv::resize(_img,_img,cv::Size(0,0),scale,scale,cv::INTER_AREA);

    _bb=bb;

    _init=true;
    return true;

}

bool SampleExtractor::openDir(std::string path, std::vector<fs::path> & file_list, std::string file_format) {
    fs::path root(path);
    std::vector<fs::path> ret;

    if(!fs::exists(root) || !fs::is_directory(root)) {
        ROS_ERROR("Dir doesnt exist");
        return false;
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

    file_list=ret;

    return true;
}


bool SampleExtractor::getBoundingRect(cv::Rect &bb) {
    //set the object boundaries
    tf::Transform obj_left;
    tf::Transform obj_right;
    obj_left.setIdentity();
    obj_right.setIdentity();

    double x_off=_off_mar_obj.x(); //x offsetz to the origin of the pallet frame
    double y_off=_off_mar_obj.y(); // y offset to the origin of the pallet frame
    double z_off=_off_mar_obj.z();
    double object_width=_obj_size.width;
    double object_height=_obj_size.height;
    int boarder_x=10;
    int boarder_y=5;



    obj_left.setOrigin(tf::Vector3(y_off,-x_off,0)); //notice here the x and y values are exchanged because of this bug
    obj_right=obj_left;
    tf::Vector3 orig_right=obj_right.getOrigin();
    orig_right.setY(orig_right.y()+ object_width);
    obj_right.setOrigin(orig_right);
    obj_left  = _cam_marker*obj_left;
    obj_right = _cam_marker*obj_right;

    //ROS_INFO("[Pallet Left XYZ: [%.2f,\t%.2f,\t%.2f]\n",obj_left.getOrigin().x(),obj_left.getOrigin().y(),obj_left.getOrigin().z());
    //ROS_INFO("[Pallet Right XYZ: [%.2f,\t%.2f,\t%.2f]\n",obj_right.getOrigin().x(),obj_right.getOrigin().y(),obj_right.getOrigin().z());
    double y,p,r;
    _cam_marker.getBasis().getEulerYPR(y,p,r);
    //ROS_INFO("Rot: [yaw,pitch,roll] = [%.2f,\t%.2f,\t%.2f]",y,p,r);

    //compute corner points of roi
    cv::Point bl=_cam_model.project3dToPixel(cv::Point3d(obj_left.getOrigin().x(),obj_left.getOrigin().y(),obj_left.getOrigin().z()+_off_mar_obj.z()));
    cv::Point ul=_cam_model.project3dToPixel(cv::Point3d(obj_left.getOrigin().x(),obj_left.getOrigin().y()-object_height,obj_left.getOrigin().z()+_off_mar_obj.z()));

    cv::Point br=_cam_model.project3dToPixel(cv::Point3d(obj_right.getOrigin().x(),obj_right.getOrigin().y(),obj_right.getOrigin().z()+_off_mar_obj.z()));
    cv::Point ur=_cam_model.project3dToPixel(cv::Point3d(obj_right.getOrigin().x(),obj_right.getOrigin().y()-object_height,obj_right.getOrigin().z()+_off_mar_obj.z()));


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

    //check if points lie in the image
    bb=cv::Rect(cv::Point(), _cam_model.fullResolution());

    //if object not in image return
    if(!(bb.contains(bl) && bb.contains(ul) && bb.contains(br)  && bb.contains(ur))){
        ROS_ERROR("Object lies outside image");
        return false;
    }

    return true;


}

bool SampleExtractor::getScale(cv::Rect bb, double &scale,cv::Rect bb_scaled, bool validateSize) {
    //determine required scale
    double s=1;
    if(bb.width>_obj_size.width){
        s=(float)_obj_size.width/(float)(bb.width);
    }else{
        ROS_ERROR("Image smaller than chosen scale");
        return false;
    }

    scale=s;

    ROS_DEBUG("Scale: \t %.2f",scale);
    //resize roi
    bb_scaled = cv::Rect((int)round(bb.x*scale),(int)round(bb.y*scale),(int)round(bb.width*scale),(int)round(bb.height*scale));
    ROS_DEBUG("Bounding Rect scaled:");
    ROS_DEBUG_STREAM(bb_scaled);

    //check if the height is smaller the required size
    if(bb_scaled.height>_obj_size.height){
        bb_scaled.height=_obj_size.height;
        if(validateSize){
            ROS_ERROR("After scaling image doesnt fit the roi..");
            ROS_ERROR("Cutting the rest..");
            return false;
        } else{
            ROS_WARN("After scaling image doesnt fit the roi..");
            ROS_WARN("Cutting the rest..");
        }

    }

    //resize rectancle to a precise height of the scaled object size
    int y_corr=_obj_size.height-bb_scaled.height;
    if(y_corr>0){
        //check if rect would fit image
        if((bb_scaled.y+bb_scaled.height+y_corr)>round(_cam_model.fullResolution().height*scale)){
            bb_scaled.y-= y_corr;
            bb_scaled.width += y_corr;
            if(validateSize){
                ROS_ERROR("Enlarging rectangle failed..");
                ROS_ERROR("Subtracting from the origin instead of adding");
                return false;
            }else{
                ROS_WARN("Enlarging rectangle failed..");
                ROS_WARN("Subtracting from the origin instead of adding");
            }
            return false;
        }
        bb_scaled.height+=y_corr;
    }

    return true;

}

void SampleExtractor::extractPositivePatch(std::string path, std::string file_type,bool overwrite) {

    static bool isOpen=false;
    if(!_init){
        ROS_DEBUG("Not initialized!");
        return;
    }

    if(!isOpen){
        if(path.at(0)=='/'){
            path.erase(0,1);
        }
        if(path.back()!='/'){
            path.append("/");
        }
        std::vector<fs::path> file_list;
        if(!openDir(_path_root+path,file_list,file_type)){
            ROS_ERROR("Dir not valid!");
            exit(1);
        }

        std::string last =file_list.back().string();
        std::regex r("[1-9][[:digit:]]+");
        std::smatch match;

        if(!overwrite){
            //check if number was matched and set the cnt variable accoringly
            if(std::regex_search(last,match,r)){
                _cnt_pos=std::stoi(match.str())+1;
                ROS_INFO("Highest number of image name: %s",match.str().c_str());
                ROS_INFO("Starting naming at: %u", _cnt_pos);

            }
        }

        isOpen=true;
    }

    cv::Mat out = _img(_bb);

    std::string img_name;
    char img_name_c[10];
    std::sprintf(img_name_c,"%.7u",_cnt_pos);
    cv::imshow("Positive patch", out);
    cv::imwrite(_path_root+path+img_name_c+file_type,out);

    _cnt_pos++;

}

void SampleExtractor::extractNegativePatch(std::string path, std::string file_type, bool overwrite) {

    static bool isOpen=false;

    if(!_init){
        ROS_DEBUG("Not initialized!");
        return;
    }
    if(!isOpen){
        if(path.at(0)=='/'){
            path.erase(0,1);
        }
        if(path.back()!='/'){
            path.append("/");
        }
        std::vector<fs::path> file_list;
        if(!openDir(_path_root+path,file_list,file_type)){
            ROS_ERROR("Dir not valid!");
            exit(1);
        }

        std::string last =file_list.back().string();
        std::regex r("[1-9][[:digit:]]+");
        std::smatch match;

        if(!overwrite){
            //check if number was matched and set the cnt variable accoringly
            if(std::regex_search(last,match,r)){
                _cnt_neg=std::stoi(match.str())+1;
                ROS_INFO("Highest number of image name: %s",match.str().c_str());
                ROS_INFO("Starting naming at: %u", _cnt_neg);

            }
        }

        isOpen=true;
    }


    //generate negative sample
    int x_ran=0;
    int y_ran=0;
    do{
        x_ran = std::rand() % (_img.cols-_bb.width);
        y_ran = std::rand() % (_img.rows-_bb.height);
    }
    while(_bb.contains(cv::Point(x_ran,y_ran))
          ||    _bb.contains(cv::Point(x_ran+_bb.width,y_ran+_bb.height))
          ||    _bb.contains(cv::Point(x_ran,y_ran+_bb.height))
          ||    _bb.contains(cv::Point(x_ran+_bb.width,y_ran)));
    cv::Rect obj_neg_rect_scaled=cv::Rect(x_ran,y_ran,_bb.width,_bb.height);
    //get roi
    cv::Mat out = _img(obj_neg_rect_scaled);

    std::string img_name;
    char img_name_c[10];
    std::sprintf(img_name_c,"%.7u",_cnt_neg);
    cv::imshow("Positive patch", out);
    cv::imwrite(_path_root+path+img_name_c+file_type,out);

    _cnt_neg++;
}


