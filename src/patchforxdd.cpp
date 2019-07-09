/*
* @file patchforxdd.cpp
* @brief This code is building for a programming problem that xdd can't make a service that controling part to call 'detect_falling
    triger'. So I made this to receive his topic and call the 'triger' when the time is appropriate.    UPDATE:NO LONGER USED.
* @author Weipeng.Xue  <goodxue@gmail.com>
* @version 1.0.0
*
************************************************
*
* Copyright (c) 2019 Weipeng.Xue.
*
* All rights reserved.
*
************************************************
*/

//******************************UNUSED******************************
#include <opencv2/opencv.hpp>
#include <sl_zed/Camera.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <ros_cutball/rect.h>                   //自定义消息类型的头文件
#include <ros_cutball/rectArray.h>
#include <ros_cutball/TF.h>                     //自定义服务
#include <std_srvs/Trigger.h>                   //服务的类型：扳机Trigger
#include <std_msgs/Bool.h>

class patch{
public:
    patch(){
        trigger = nh.serviceClient<std_srvs::Trigger>("trigger");
        sub_xdd = nh.subscribe("xdd_call",1,&patch::call_back,this);
        srvice_tf = nh.advertiseService("/balloon_falling",&patch::TF_callback,this);
        Return = false;
    }
private:
    ros::NodeHandle nh;
    ros::ServiceClient trigger;
    ros::Subscriber sub_xdd;
    ros::ServiceServer srvice_tf;   //接收detect_falling 发出的坐标象限
    bool Return;
    
    void call_back(const std_msgs::Bool msg) {

    }
    bool TF_callback(ros_cutball::TF::Request  &req,ros_cutball::TF::Response &res){

    }
};

int main(int argc, char** argv)  {
    ros::init(argc, argv, "patch_node");
    ros::NodeHandle nh;
    // ros::ServiceClient call_update_howmany_red = nh.serviceClient<std_srvs::Trigger>("/detect_color/detect_red/red_balloon_fell");
    // ros::ServiceClient call_update_howmany_yellow = nh.serviceClient<std_srvs::Trigger>("/detect_color/detect_red/yellow_balloon_fell");

    ros::Rate loop_rate(15);
    while (nh.ok()) {
        ros::spinOnce(); 
        loop_rate.sleep();
    }
    return 0;
}