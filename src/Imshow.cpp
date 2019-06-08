/*
* @file Imshow.cpp
* @brief TUDO
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
#include <ros/ros.h>  
#include <image_transport/image_transport.h>  
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>  
#include <ros_cutball/rect.h>                   //自定义消息类型的头文件
#include <ros_cutball/rectArray.h>
#include <ros_cutball/point2i.h>
#include <vector>

class imshow{
private:

    ros::NodeHandle nh; 
    image_transport::ImageTransport it;
    image_transport::Publisher image_pub_;
    image_transport::Subscriber image_sub_;
    ros::Subscriber sub_red;
    ros::Subscriber sub_yellow;
    ros::Subscriber sub_black;


    std::vector<cv::Rect> vec_rect_red;
    std::vector<cv::Rect> vec_rect_yellow;
    cv::Point point_black;
    cv_bridge::CvImagePtr cv_ptr;
    image_transport::Publisher convert_pub_;
    cv::Rect rect_temp;
    //cv::Mat image_raw;    
    sensor_msgs::ImagePtr ros_image;      
    int red_num,yellow_num;

    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void rect_array_yellow_callback(const ros_cutball::rectArray::ConstPtr& msg);
    void rect_array_red_callback(const ros_cutball::rectArray::ConstPtr& msg);
    void point_black_callback(const ros_cutball::point2i::ConstPtr& msg);

public:
    imshow():it(nh) {
        image_pub_ = it.advertise(std::string("/image_convert/Imshow"),1);
        image_sub_ = it.subscribe("/zed/rgb/image_raw_color", 1,&imshow::imageCallback,this);
        sub_yellow = nh.subscribe("/detect_color/detect_yellow/Detect_Color/rect/yellow", 1, &imshow::rect_array_yellow_callback,this);
        sub_red = nh.subscribe("/detect_color/detect_red/Detect_Color/rect/red", 1, &imshow::rect_array_red_callback,this);
        sub_black = nh.subscribe("/detect_black/Detect_Color/black_point", 1 , &imshow::point_black_callback,this);

        nh.getParam("red_balloon",red_num);
        nh.getParam("yellow_balloon",yellow_num);
        if (red_num > 0) {
            vec_rect_red.resize(red_num);
        }
        if (yellow_num > 0) {
            vec_rect_yellow.resize(yellow_num);
        }
    }

    void start() {
        if (!cv_ptr) return;

        // if (vec_rect_red.empty())
        // {
        //     ROS_INFO("!!!!!!!");
        // }
        // ROS_INFO("size: %d", vec_rect_red.size());
        for (int i = 0;i < vec_rect_red.size();i++) {
            cv::rectangle(cv_ptr->image,vec_rect_red.at(i),cv::Scalar(255,255,0),10);
            //cv::circle(cv_ptr->image,cv::Point2i(cv_ptr->image.cols/2,cv_ptr->image.rows/2),10,cv::Scalar(0,0,255),-1);
        }
        for (int i = 0;i < vec_rect_yellow.size();i++) {
            cv::rectangle(cv_ptr->image,vec_rect_yellow.at(i),cv::Scalar(255,0,0),10);
        }
        if ( (point_black.x != -1) || (point_black.y != -1)) {
            cv::circle(cv_ptr->image,point_black,5,cv::Scalar(0,255,0),2);
            //ROS_INFO("x:%d y:%d",point_black.x,point_black.y);
        }
        
        //ROS_INFO("red:  x: %d, y: %d, width: %d, height: %d",vec_rect_red.at(0).x,vec_rect_red[0].y,vec_rect_red[0].width,vec_rect_red[0].height);
        
        image_pub_.publish(cv_ptr->toImageMsg());
    }
};


void imshow::imageCallback(const sensor_msgs::ImageConstPtr& msg)  
{  
    try  
    {  
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);  
    }  
    catch (cv_bridge::Exception& e)  
    {  
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());  
    }  
    //image_raw = cv_ptr->image;
}  


void imshow::rect_array_yellow_callback(const ros_cutball::rectArray::ConstPtr& msg) {
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


void imshow::rect_array_red_callback(const ros_cutball::rectArray::ConstPtr& msg) {
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
//TUDO将msg的时间戳加上，同步信息。

//point_black在类中定义,将msg中的point的值赋值给point_black，最终在发布时绘制到图像上
void imshow::point_black_callback(const ros_cutball::point2i::ConstPtr& msg) {
    point_black.x = msg->x;
    point_black.y = msg->y;
}




int main(int argc, char **argv)  
{  
    ros::init(argc, argv, "Imshow"); 
    imshow is;
    ros::Rate loop_rate(15);
    while(ros::ok()) {
        is.start();
        ros::spinOnce();  
        loop_rate.sleep();
    }
 
}  