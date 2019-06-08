/*
* @file Detect_Color.cpp
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
#include "Detect_Color.hpp"

//TUDO后面再修改，辅助函数【全局】
bool comp_area(cv::Rect& Rect1,cv::Rect& Rect2)
{
    return Rect1.area() > Rect2.area();
}


bool comp_rect_y(cv::Rect& rect1,cv::Rect& rect2)
{
    return rect1.y > rect2.y;
}


//节点内subscribe的回调函数，用来将原先ros中的图像类型转变为opencv中的图像类型,此处共享图片数据
void Detect_Color::updateImage(const sensor_msgs::ImageConstPtr& msg) {
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    cvmat_raw = cv_ptr->image;
}



Detect_Color::Detect_Color(std::string Scolor,int Num_balloons):it_(nh_),scolor(Scolor){

    image_sub_ = it_.subscribe("/zed/rgb/image_raw_color", 1,&Detect_Color::updateImage, this);

    //image_pub_ = it_.advertise("/image_converter/output_raw", 1);
    threshold_pub_ = it_.advertise(std::string("/image_convert/")+scolor,1);
    counter_pub = nh_.advertise<ros_cutball::rectArray>(std::string("Detect_Color/rect/")+scolor,1);
    std::string scolor_temp = scolor + std::string("_balloon_fell");
    update_howmany = nh_.advertiseService(scolor_temp,&Detect_Color::Update_howmany,this);
    //TUDO后期添加两个服务的客户端，用于告诉这个服务气球下落减少检测的气球数
    //dynamic_reconfigure::Server<ros_cutball::hsv_Config> server;
    
    //server.setCallback(f);
    
    nh_.getParam(scolor+std::string("_H1"),range_hsv[0]);
    nh_.getParam(scolor+std::string("_S1"),range_hsv[1]);
    nh_.getParam(scolor+std::string("_V1"),range_hsv[2]);
    nh_.getParam(scolor+std::string("_H2"),range_hsv[3]);
    nh_.getParam(scolor+std::string("_S2"),range_hsv[4]);
    nh_.getParam(scolor+std::string("_V2"),range_hsv[5]);

    howmany = Num_balloons;

    //cv::namedWindow("test");
}



void Detect_Color::start() {
    if (cvmat_raw.empty()) return;                  //初始时可能zed还没发布图像，所以订阅不到，直接返回
    if (howmany < 1) return;                        //即只剩一个的气球也下落了
    cv::cvtColor(cvmat_raw,cvmat_hsv,cv::COLOR_BGR2HSV);
    cv::inRange(cvmat_hsv,cv::Scalar(range_hsv[0],range_hsv[1],range_hsv[2]),cv::Scalar(range_hsv[3],range_hsv[4],range_hsv[5]),cvmat_hsv);
    //ROS_INFO("{H1:%d S1:%d V1:%d,H2:%d S2:%d V2:%d}",range_hsv[0],range_hsv[1],range_hsv[2],range_hsv[3],range_hsv[4],range_hsv[5]);
    //cv::imshow("test",cvmat_hsv);
    //cv::waitKey(1);
    Proc_Img(cvmat_hsv);
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(cvmat_hsv,contours,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);
    std::vector<std::vector<cv::Point> > convexHulls(contours.size());
    
    for(unsigned int i = 0; i < contours.size(); i++)
    {
        cv::convexHull(contours[i], convexHulls[i]);
    }
    std::vector<cv::Rect> rectangles;
    for(int i = 0;i < contours.size();i++)
    {
        rectangles.push_back(cv::boundingRect(convexHulls[i]));//框选住的矩形框，在使用气球大小时最好不使用。TODO
    }
    std::sort(rectangles.begin(),rectangles.end(),comp_area); //需要有一定顺序，否则给气球对象分配会乱。按面积排序
//#ifdef DETECT_FALLING
    rectangles.resize(howmany); //howmany的大小和size的大小谁大？
    //                                           [comp_area] before
    if (howmany > 1)                                            //按照y值排序
        std::sort(rectangles.begin(),rectangles.end(),comp_rect_y);
    
//#endif
    if (rectangles.size() < howmany) {                         //将返回的矩形框数组大小调整为需要检测的气球数量
        vec_rect_color.assign(rectangles.begin(),rectangles.begin() + rectangles.size());
        while (vec_rect_color.size() < howmany) {
            vec_rect_color.push_back(cv::Rect());
        }
    } else {
        vec_rect_color.assign(rectangles.begin(),rectangles.begin() + howmany);
    }
    // if (scolor == std::string("red"))
    //     ROS_INFO("x: %d, y: %d",vec_rect_color[0].x,vec_rect_color[0].y);
    //ROS_INFO("ahead howmany");
    // if (howmany > 1) {
    //     // if (find_ball_num > 5) {
    //     //     for (int i = 0;i < rectangles.size();i++) {
    //     //         cv::Point aim_point = rectangles[i].br();
    //     //         int _temp_ = match_up(aim_point,first_rect_pos);
    //     //         now_has_point[_temp_] = true;
    //     //     }
    //     //     for (int i = 0;i < howmany;i ++) {
    //     //         if (!now_has_point[i]) {
    //     //             vec_rect_color.insert(vec_rect_color.begin()+i,cv::Rect());
    //     //         }
    //     //     }
    //     // }
    //     for (int i = 0;i < howmany;i++) {
    //         first_rect_pos[i] = vec_rect_color[i].br();
    //     }


    //     for (int i = 0; i < howmany; i++) {
    //         now_has_point[i] = false;
    //     }

    // }
    //ROS_INFO("ahead pub");
    image_pub();
    //ROS_INFO("end image_pub");
    data_pub();
}





void Detect_Color::Proc_Img(cv::Mat& thresh)
{
    cv::Mat erodeElement = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3,3));
    cv::Mat dilateElement = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(5,5));

    int erodenumber = 1;
    int dilatenumber = 1;

    for(int i=0;i<erodenumber;i++)
    {
        cv::erode(thresh,thresh,erodeElement);
    }

    for(int i=0;i<dilatenumber;i++)
    {
        cv::dilate(thresh,thresh,dilateElement);
    }
}


void Detect_Color::data_pub() {
    ros_cutball::rect Rect;
    ros_cutball::rectArray Rectmsg;
    for(int i = 0;i < vec_rect_color.size();++i) {
        Rect.x = vec_rect_color[i].x;
        Rect.y = vec_rect_color[i].y;
        Rect.width = vec_rect_color[i].width;
        Rect.height = vec_rect_color[i].height;
        Rectmsg.rectarray.push_back(Rect);
        // if (scolor == std::string("red"))
        //     ROS_INFO("publisher rectmsg of red's size is %d",Rectmsg.rectarray.size());
    }
    counter_pub.publish(Rectmsg);
}


void Detect_Color::image_pub() {
    // for (int i = 0;i < howmany;i++)
    //     cv::rectangle(cvmat_raw,vec_rect_color[i],cv::Scalar(255,0,0),2);
    //ros_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cvmat_raw).toImageMsg();
    //image_pub_.publish(ros_image);
    ros_threshold = cv_bridge::CvImage(std_msgs::Header(), "mono8", cvmat_hsv).toImageMsg();
    threshold_pub_.publish(ros_threshold);
    
}


void Detect_Color::hsv_update(int H1,int S1,int V1,int H2,int S2,int V2) {
    range_hsv[0] = H1;
    range_hsv[1] = S1;
    range_hsv[2] = V1;
    range_hsv[3] = H2;
    range_hsv[4] = S2;
    range_hsv[5] = V2;
}


bool Detect_Color::Update_howmany(std_srvs::Trigger::Request &req,std_srvs::Trigger::Response &res) {
    if (howmany >= 1){
        howmany--;
        nh_.setParam(scolor+std::string("_balloon"),howmany);           //更改参数服务器上的气球数量
        ROS_INFO("now there has %d %s_balloons left",howmany,scolor.data());
        res.success = true;
        return true;
    }
    else {
        ROS_INFO("the nnumber of %s_balloon can't smaller than 0",scolor.data());
        return false;
    }
}