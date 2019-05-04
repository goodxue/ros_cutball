#include "detect_falling.hpp"

detect_falling::detect_falling():it_(nh_) {
    sub_rectarray_red = nh_.subscribe("/detect_color/detect_red/Detect_Color/rect/red", 1,&detect_falling::update_rect_red, this);
    sub_rectarray_yellow = nh_.subscribe("/detect_color/detect_yellow/Detect_Color/rect/yellow", 1,&detect_falling::update_rect_yellow, this);
    sub_height = nh_.subscribe("xdd_z",1,&detect_falling::update_height,this);
    depth_sub_ = it_.subscribe("/zed/depth/depth_registered", 1,&detect_falling::depthCallback,this);
    srv_tf = nh_.serviceClient<ros_cutball::TF>("ball_falling");

    //初始化一些用于保存坐标点的vector
    nh_.getParam("red_balloon",red_num);
    nh_.getParam("yellow_balloon",yellow_num);
    if (red_num > 0) {
        vec_rect_red.resize(red_num);
    }
    if (yellow_num > 0) {
        vec_rect_yellow.resize(yellow_num);
    }
}

void detect_falling::update_rect_red(const ros_cutball::rectArray::ConstPtr& msg) {

}