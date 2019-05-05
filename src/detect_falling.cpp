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
    nh_.getParam("weight_value",weight_value);
    nh_.getParam("weight_point_num",weight_point_num);
    if (red_num > 0) {
        vec_rect_red.resize(red_num);
    }
    if (yellow_num > 0) {
        vec_rect_yellow.resize(yellow_num);
    }
}

void detect_falling::update_rect_red(const ros_cutball::rectArray::ConstPtr& msg) {
    vec_rect_red.resize(msg->rectarray.size());               //预分配大小，使用stl的vector一定要小心越界
    for (int i=0; i < msg->rectarray.size(); ++i)
    {
        vec_rect_red[i].x = msg->rectarray[i].x;
        vec_rect_red[i].y = msg->rectarray[i].y;
        vec_rect_red[i].width = msg->rectarray[i].width;
        vec_rect_red[i].height = msg->rectarray[i].height;
        //vec_rect_red[i] = rect_temp;
    }
}


void detect_falling::update_rect_red(const ros_cutball::rectArray::ConstPtr& msg) {
    vec_rect_yellow.resize(msg->rectarray.size());               //预分配大小，使用stl的vector一定要小心越界
    for (int i=0; i < msg->rectarray.size(); ++i)
    {
        vec_rect_yellow[i].x = msg->rectarray[i].x;
        vec_rect_yellow[i].y = msg->rectarray[i].y;
        vec_rect_yellow[i].width = msg->rectarray[i].width;
        vec_rect_yellow[i].height = msg->rectarray[i].height;
        //vec_rect_yellow[i] = rect_temp;
    }
}

//飞机发布的高度信息，回调函数更改内部的height值。
void detect_falling::update_height(const std_msgs::Float64 Heights) {
    height = Heights.data;
}

//气球下落，向客户端发送请求的函数封装，当请求发出后，程序即暂停在这里等待服务端回应：
//当飞机先跑到提前设定的象限的定点后，然后根据黑圆坐标判断返回原点后再向客户端发送成功
//信息，然后检测下落的程序才开始继续。
bool detect_falling::call_service(int tf_num) {
    ros_cutball::TF tf;
    tf.request.tf = tf_num;
    ROS_INFO("calling service on \"ball_falling\" of tf %d ...",tf_num);
    if (srv_tf.call(tf)){
        if (tf.response.success) {
            ROS_INFO("UAV has return to original place, ready to resume the falling_detection.");

        }
        else {
            ROS_INFO("fatel error occurs: server returned a false state for %s",tf.response.message);
            ROS_INFO("shutting down the detect_falling node");
            ros::shutdown();
        }
    }
    else {
        ROS_INFO("fatel error occurs: Failed to call the server.");
        return false;
    }
    return true;
}


void detect_falling::weighted_average_to_key_pixel() {
    key_pixel.resize(red_num+yellow_num);
    for (int i = 0;i < vec_rect_red.size();i++) {

    }
}