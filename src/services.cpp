/*
* @file services.cpp
* @brief TUDO
* @author Weipeng.Xue  <xuedplay@gmail.com>
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
#include <opencv2/opencv.hpp>
#include <sl_zed/Camera.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <ros_cutball/rect.h>                   //自定义消息类型的头文件
#include <ros_cutball/rectArray.h>
#include <std_msgs/Float64.h>                   //高度话题消息类型
#include <ros_cutball/TF.h>                     //自定义服务
#include <std_srvs/Trigger.h>                   //服务的类型：扳机Trigger
bool TF_callback(ros_cutball::TF::Request  &req,
         ros_cutball::TF::Response &res)
{
  ROS_INFO_STREAM("TF recieved: ["<<req.tf<<"],holding... press 'g' to continue...");
  for(int i=0;i < 4;i++) {
    ros::Duration(1).sleep();
    ROS_INFO("pause %d s to restart...",4-i);
  }
  ROS_INFO("continue... return to detect_falling...");
  return true;
}


int main(int argc, char** argv)  {
    ros::init(argc, argv, "Service_node");
    ros::NodeHandle nh;
    // ros::ServiceClient call_update_howmany_red = nh.serviceClient<std_srvs::Trigger>("/detect_color/detect_red/red_balloon_fell");
    // ros::ServiceClient call_update_howmany_yellow = nh.serviceClient<std_srvs::Trigger>("/detect_color/detect_red/yellow_balloon_fell");
    ros::ServiceServer srvice_tf = nh.advertiseService("/balloon_falling",TF_callback);

    ros::Rate loop_rate(15);
    while (nh.ok()) {
        ros::spinOnce(); 
        loop_rate.sleep();
    }
    return 0;
}