//
// Created by tman on 6/14/17.
//

#include "../include/ar_tracker/SampleExtractor.h"

#include <regex>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>


SampleExtractor::SampleExtractor() {

}

SampleExtractor::SampleExtractor(cv::Size rect_size,cv::Size2d obj_size, cv::Size2d boarder_size_percent, std::string path_root, tf::Transform marker2obj,
                                 sensor_msgs::CameraInfo cam_info) {
    _cam_model.fromCameraInfo(cam_info);

    if(path_root.back()!='/'){
        path_root.append("/");
        ROS_DEBUG("SampleExtractor: Path corrected to : %s", path_root.c_str());
    }
    _path_root=path_root;
    _rect_size=rect_size;
    _marker2obj=marker2obj;
    _boarder_size_percent=boarder_size_percent;
    _cnt_neg=0;
    _cnt_pos=0;
    _obj_size=obj_size;
}

bool SampleExtractor::update(tf::Transform cam_marker, cv::Mat img, bool rectify, bool verifyRectSize) {

    _init=false;

    if(!_cam_model.initialized()){
        return false;
    }

    _cam2marker=cam_marker;

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

    _bb_scaled=bb_scaled;
    _init=true;
    return true;

}

bool SampleExtractor::openDir(std::string path, std::vector<fs::path> & file_list, std::string file_format) {
    fs::path root(path);
    std::vector<fs::path> ret;

    if(!fs::exists(root) || !fs::is_directory(root)) {
        ROS_ERROR("Dir doesnt exist: %s", root.string().c_str());
        return false;
    }

    fs::recursive_directory_iterator it(root);
    fs::recursive_directory_iterator endit;

    while(it != endit)
    {
        if(fs::is_regular_file(*it) && it->path().extension() == ".jpg") ret.push_back(it->path().filename());
        ++it;

    }
    std::sort(ret.begin(),ret.end(), [] (fs::path & one, fs::path & two){
        return one.string()< two.string();
    } );

    file_list=ret;

    return true;
}


bool SampleExtractor::getBoundingRect(cv::Rect &bb) {
    //set the object boundaries
    tf::Transform obj_left_b;
    tf::Transform obj_left_u;
    tf::Transform obj_right_b;
    tf::Transform obj_right_u;
    obj_left_b.setIdentity();
    obj_right_b.setIdentity();

    double object_width=_obj_size.width;
    double object_height=_obj_size.height;
    double boarder_percent_x=_boarder_size_percent.width;
    double boarder_percent_y=_boarder_size_percent.height;



    obj_left_b.setOrigin(tf::Vector3(0,0,0)); //origin of object is bottom left point
    ///set up left point
    obj_left_u=obj_left_b;
    obj_left_u.getOrigin().setY(obj_left_u.getOrigin().y()+object_height);
    ///set bottom right
    obj_right_b=obj_left_b;
    obj_right_b.getOrigin().setX(obj_right_b.getOrigin().x()+object_width);
    ///set up right
    obj_right_u=obj_right_b;
    obj_right_u.getOrigin().setY(obj_right_u.getOrigin().y()+object_height);


    //ROS_INFO("Transmat:\n %f %f %f\n %f %f %f\n %f %f %f\n",_marker2obj.getBasis().getRow(0).x(),_marker2obj.getBasis().getRow(0).y(),_marker2obj.getBasis().getRow(0).z(),_marker2obj.getBasis().getRow(1).x(),
    //        _marker2obj.getBasis().getRow(1).y(),_marker2obj.getBasis().getRow(1).z(),_marker2obj.getBasis().getRow(2).x(),_marker2obj.getBasis().getRow(2).y(),_marker2obj.getBasis().getRow(2).z());

    //convert into camera frame
    obj_left_b.mult(_marker2obj,obj_left_b);
    obj_left_u.mult(_marker2obj,obj_left_u);
    obj_right_b.mult(_marker2obj,obj_right_b);
    obj_right_u.mult(_marker2obj,obj_right_u);



    obj_left_b  = (_cam2marker*obj_left_b);
    obj_left_u = (_cam2marker*obj_left_u);
    obj_right_b = (_cam2marker*obj_right_b);
    obj_right_u = (_cam2marker*obj_right_u);
    //ROS_INFO("BL: [%.2f,\t%.2f,\t%.2f]", obj_left_b.getOrigin().x(),obj_left_b.getOrigin().y(),obj_left_b.getOrigin().z() );
    //ROS_INFO("UL: [%.2f,\t%.2f,\t%.2f]", obj_left_u.getOrigin().x(),obj_left_u.getOrigin().y(),obj_left_u.getOrigin().z() );
    //ROS_INFO("BR: [%.2f,\t%.2f,\t%.2f]", obj_right_b.getOrigin().x(),obj_right_b.getOrigin().y(),obj_right_b.getOrigin().z() );
    //ROS_INFO("UR: [%.2f,\t%.2f,\t%.2f]", obj_right_u.getOrigin().x(),obj_right_u.getOrigin().y(),obj_right_u.getOrigin().z() );

    double y,p,r;
    _cam2marker.getBasis().getEulerYPR(y,p,r);
    //ROS_INFO("Rot: [yaw z, pitch y, roll x] = [%.2f,\t%.2f,\t%.2f]",y,p,r);

    //compute corner points of roi
    cv::Point bl=_cam_model.project3dToPixel(cv::Point3d(obj_left_b.getOrigin().x(),obj_left_b.getOrigin().y(),obj_left_b.getOrigin().z()));
    cv::Point ul=_cam_model.project3dToPixel(cv::Point3d(obj_left_u.getOrigin().x(),obj_left_u.getOrigin().y(),obj_left_u.getOrigin().z()));

    cv::Point br=_cam_model.project3dToPixel(cv::Point3d(obj_right_b.getOrigin().x(),obj_right_b.getOrigin().y(),obj_right_b.getOrigin().z()));
    cv::Point ur=_cam_model.project3dToPixel(cv::Point3d(obj_right_u.getOrigin().x(),obj_right_u.getOrigin().y(),obj_right_u.getOrigin().z()));

    //get bounding box of object
    std::vector<cv::Point> pts {bl,br,ul,ur};
    cv::Rect obj_rect=cv::boundingRect(pts);

    //compute boarder
    int border_frame_x= (int)round((obj_rect.width*boarder_percent_x/100)/2);
    int border_frame_y= (int)round((obj_rect.height*boarder_percent_y/100)/2);
    obj_rect.x -= border_frame_x;
    obj_rect.width += 2*border_frame_x;
    obj_rect.y -= border_frame_y;
    obj_rect.height += 2* border_frame_y;

    ROS_INFO("Bounding Rect:");
    ROS_INFO_STREAM(obj_rect);

    //check if points lie in the image
    cv::Rect rect_full=cv::Rect(cv::Point(), _cam_model.fullResolution());

    //if object not in image return
    if(!(rect_full.contains(obj_rect.br()) && rect_full.contains(obj_rect.tl()) )){
        ROS_ERROR("Object lies outside image");
        return false;
    }

    bb=obj_rect;
    return true;


}

bool SampleExtractor::getScale(cv::Rect bb, double &scale,cv::Rect &bb_scaled, bool validateSize) {
    //determine required scale
    double s=1;
    if(bb.width>_rect_size.width){
        s=(float)_rect_size.width/(float)(bb.width);
    }else{
        ROS_ERROR("Image smaller than chosen scale");
        ROS_INFO("Bounding Rect");
        ROS_INFO_STREAM(bb);
        ROS_INFO("ROI Size");
        ROS_INFO_STREAM(_rect_size);
        return false;
    }

    scale=s;

    ROS_DEBUG("Scale: \t %.2f",scale);
    //resize roi
    bb_scaled = cv::Rect((int)round(bb.x*scale),(int)round(bb.y*scale),(int)round(bb.width*scale),(int)round(bb.height*scale));
    ROS_DEBUG("Bounding Rect scaled:");
    ROS_DEBUG_STREAM(bb_scaled);
    ROS_DEBUG("Bounding Rect:");
    ROS_DEBUG_STREAM(bb);

    //check if the height is smaller the required size
    if(bb_scaled.height>_rect_size.height){
        bb_scaled.height=_rect_size.height;
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
    int y_corr=_rect_size.height-bb_scaled.height;
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
    if(path.at(0)=='/'){
        path.erase(0,1);
    };
    if(path.back()!='/'){
        path.append("/");
    };

    if(!isOpen){

        ROS_INFO("Path: %s",path.c_str());
        std::vector<fs::path> file_list;
        if(!openDir(_path_root+path,file_list,file_type)){
            ROS_ERROR("Dir not valid!");
            exit(1);
        }
        if(!file_list.empty()&& !overwrite){
            std::string last =file_list.back().string();
            std::regex r("[1-9][[:digit:]]+");
            std::smatch match;
            //check if number was matched and set the cnt variable accoringly
            if(std::regex_search(last,match,r)){
                _cnt_pos=std::stoi(match.str())+1;
                ROS_INFO("Highest number of image name: %s",match.str().c_str());
                ROS_INFO("Starting naming at: %u", _cnt_pos);

            }

        }


        isOpen=true;
    }

    cv::Mat out = _img(_bb_scaled);



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
    if(path.at(0)=='/'){
        path.erase(0,1);
    };
    if(path.back()!='/'){
        path.append("/");
    };

    if(!isOpen){

        ROS_INFO("Path: %s",path.c_str());
        std::vector<fs::path> file_list;
        if(!openDir(_path_root+path,file_list,file_type)){
            ROS_ERROR("Dir not valid!");
            exit(1);
        }
        if(!file_list.empty()&& !overwrite){
            std::string last =file_list.back().string();
            std::regex r("[1-9][[:digit:]]+");
            std::smatch match;
            //check if number was matched and set the cnt variable accoringly
            if(std::regex_search(last,match,r)){
                _cnt_pos=std::stoi(match.str())+1;
                ROS_INFO("Highest number of image name: %s",match.str().c_str());
                ROS_INFO("Starting naming at: %u", _cnt_pos);

            }

        }


        isOpen=true;
    }


    //generate negative sample
    int x_ran=0;
    int y_ran=0;
    do{
        x_ran = std::rand() % (_img.cols-_bb_scaled.width);
        y_ran = std::rand() % (_img.rows-_bb_scaled.height);
    }
    while(_bb_scaled.contains(cv::Point(x_ran,y_ran))
          ||    _bb_scaled.contains(cv::Point(x_ran+_bb_scaled.width,y_ran+_bb_scaled.height))
          ||    _bb_scaled.contains(cv::Point(x_ran,y_ran+_bb_scaled.height))
          ||    _bb_scaled.contains(cv::Point(x_ran+_bb_scaled.width,y_ran)));
    cv::Rect obj_neg_rect_scaled=cv::Rect(x_ran,y_ran,_bb_scaled.width,_bb_scaled.height);
    //get roi
    cv::Mat out = _img(obj_neg_rect_scaled);

    std::string img_name;
    char img_name_c[10];
    std::sprintf(img_name_c,"%.7u",_cnt_neg);
    cv::imshow("Positive patch", out);
    cv::imwrite(_path_root+path+img_name_c+file_type,out);

    _cnt_neg++;
}

bool SampleExtractor::initDetection(std::string path_hog) {
    if(!fs::exists(path_hog) || !fs::is_regular_file(path_hog)) {
        ROS_ERROR("Path to HogDescriptor file is wrong: %s", path_hog.c_str());
        return false;
    }
    if(!_hog.load(path_hog)){
        ROS_ERROR("Could not load HogDescriptor: %s", path_hog.c_str());
        return false;
    }
    ROS_INFO("HogDescriptor initialized..");
    _init_detection=true;
    return true;

}

void SampleExtractor::extractFalsePositives(std::string path, std::string file_type, bool overwrite) {

    if(!_init){
        ROS_DEBUG("Not initialized!");
        return;
    }
    if(!_init_detection){
        ROS_DEBUG("Detection not initialized!");
        return;
    }

    std::vector<cv::Point> locations;
    std::vector<double> weights;
    if(_img.type()!=CV_8UC1){
        ROS_ERROR("Image has to be of type CV_8UC1");
        return;
    }
    double min,max;
    cv::minMaxLoc(_img,&min,&max);
    if(max<=1){
        ROS_ERROR("Image has wrong scale max= %f",max);
    }
    _hog.detect(_img,locations,weights,-10);
    //if sth was detected, draw the image with the detection and order the
    if(!locations.empty()){
        //std::sort(weights.begin(),weights.end());
        long idx=std::distance(weights.begin(),std::min_element(weights.begin(),weights.end()));
        for(auto i :weights){
            ROS_INFO("%f",i);
        }

        cv::Rect bb=_bb_scaled;
        bb.x =locations[idx].x;
        bb.y =locations[idx].y;
        cv::rectangle(_img,bb,cv::Scalar(0,0,0),5);
        cv::imshow("Detection", _img);
    }


}

