/*
* @file msg_bridge.cpp
* @brief A bridge of transporting msg form from Point2i to geometry.
* * * *  Because my partner confused the interface definition and I have noting to do but compromise.
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
#include <opencv2/opencv.hpp>
#include <sl_zed/Camera.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <ros_cutball/rect.h>                   //自定义消息类型的头文件
#include <ros_cutball/rectArray.h>
#include <ros_cutball/point2i.h>
#include <geometry_msgs/Pose2D.h>
class msg_bridge{
public:
    msg_bridge();
private:
    void msg_callback(const ros_cutball::point2i& msg);
    ros::NodeHandle nh;
    ros::Subscriber msg_sub;
    ros::Publisher msg_pub;
};

msg_bridge::msg_bridge() {
    msg_sub = nh.subscribe(std::string("/detect_black/Detect_Color/black_point"),1,&msg_bridge::msg_callback,this);
    msg_pub = nh.advertise<geometry_msgs::Pose2D>(std::string("/msg_bridge/black_point"),1);
}

void msg_bridge::msg_callback(const ros_cutball::point2i& msg) {
    geometry_msgs::Pose2D _point;
    _point.x = msg.x;
    _point.y = msg.y;
    msg_pub.publish(_point);
    //ROS_INFO_STREAM("x: "<<_point.x<<"  y: "<<_point.y);
}


int main(int argc, char** argv)  {
    ros::init(argc, argv, "Msg_bridge");
    msg_bridge mb;

    ros::Rate loop_rate(15);
    while (ros::ok()) {
        ros::spinOnce(); 
        loop_rate.sleep();
    }
    return 0;
}