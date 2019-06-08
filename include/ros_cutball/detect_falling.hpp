/*
* @file detect_falling.hpp
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
#include <opencv2/opencv.hpp>
#include <sl_zed/Camera.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <ros_cutball/rect.h>                   //自定义消息类型的头文件
#include <ros_cutball/rectArray.h>
#include <std_msgs/Float64.h>                   //高度话题消息类型
#include <ros_cutball/TF.h>                     //自定义服务
#include <std_srvs/Trigger.h>                   //服务的类型：扳机Trigger
//#include <sensor_msgs/Image.h>                  //深度图像订阅类型
#include <vector>
#include <queue>
#include <map>
#include <cmath>

class detect_falling {
public:
    //Detect_falling();
    detect_falling(std::string parameter = std::string(""));
    bool start();

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber depth_sub_;         //订阅主图像
    ros::Subscriber sub_rectarray_red;      //红球boundingbox
    ros::Subscriber sub_rectarray_yellow;   //黄球boundingbox
    ros::Subscriber sub_height;             //飞机当前高度
    
    ros::ServiceClient srv_tf;          //ros服务，客户端，当有气球下落则暂停继续监测，等待飞机返回原点后继续,这是一个#client#
    ros::ServiceServer srv_check;       //ros服务，服务端，客户端发出一个扳机，开启或者关闭深度的检测，用于飞机飞的平稳时发布一个信号，开始检测
    ros::ServiceClient call_update_howmany_red; //ros服务，客户端，气球下落后呼叫detect_color节点更新待检测气球数量。
    ros::ServiceClient call_update_howmany_yellow;  //黄色的客户端

    void update_rect_red(const ros_cutball::rectArray::ConstPtr& msg);
    void update_rect_yellow(const ros_cutball::rectArray::ConstPtr& msg);       //更新红黄的boundingbox(用于确定气球上检测深度的几个点)
    void update_height(const std_msgs::Float64 Heights);                        //更新飞机当前高度
    void depthCallback(const sensor_msgs::Image::ConstPtr& msg);                //深度图的订阅

    bool reinitialize();                                
    //sl::Mat slmat_depth;                                //zed深度图（弃置不用，因为这个节点不需要保存深度图，只需要几个位置的深度信息即可）
    bool start_checking;        //用于控制何时开始检测下落，使用一个service扳机控制
    int time_decay;                                     //延迟等待一段时间,让所有成员都获得初值
    int red_num,yellow_num;                             //查看红色黄色球数量
    int queue_num;                                     //队列最大存储深度的个数,即判断下落时取出首位
    //double height;
    double falling_threshold;
    double falling_threshold_m;                           //一定时间段内气球相对地面移动的距离阈值，超过即掉落
    double falling_threshold_mm;                           //一定时间段内气球相对地面移动的距离阈值，超过即掉落
    int openni_depth_mode;         //深度数值的单位。0是米为单位，1是毫米为单位
    bool test;
    std::vector<cv::Rect> vec_rect_red;                 //红色boundingbox
    std::vector<cv::Rect> vec_rect_yellow;              //黄色boundingbox


    std::vector<std::vector<cv::Point2i>> red_key_pixel_2d;      //二维数组，每个元素是一个要检测的气球需要监测深度的点  
    std::vector<std::vector<cv::Point2i>> yellow_key_pixel_2d;   //为什么

    int weight_value;               //从中心点向外扩展的像素点值
    int weight_point_num;           //取点的个数，默认为(weight_point_num+1)^2 默认为2
                                    // ·----·----·
                                    // |     ↙中心|
                                    // ·    o    ·
                                    // |         |
                                    // ·----·----·
                                    // ——————   ←---
                                    //    8
                                    //weight_value-↑

    void weighted_average_to_key_pixel(std::vector<cv::Rect>& rect_vec,std::vector<std::vector<cv::Point2i>>& vec_vec_point);                 //返回加权平均值，表示要检测深度的点。数据保存在key_pixel
    bool call_service(int tf_num);                      //call了service之后，前面的数据需要全部清除并重新初始化，因为气球个数少了
                                                        //TUDO应该需要由节点来执行这个操作，比如说重新建个detect_falling对象
    bool checking_trigger(std_srvs::Trigger::Request& req,std_srvs::Trigger::Response& res);
    std::vector<std::queue<float> > red_depth_queue_vec;       //队列的顺序由标号决定，这就要求了一次下落之间时候，颜色检测时的气球相对顺序不要变
    std::vector<std::queue<float> > yellow_depth_queue_vec;
    std::queue<double> height_queue;
    void depth_push_into_queue(std::queue<float>& depth_queue,float depth);
    bool check_falling();
    int calculate_tf(cv::Rect object);

    std::map<int,bool> tf_call;     //这个东西是用来判断某个象限的气球是否掉落用的，黄色掉落也一样如实记录，只是判断函数的输出输出相反象限。

};

    bool comp_rect_y(cv::Rect& rect1,cv::Rect& rect2);
    bool comp_rect_x(cv::Rect& rect1,cv::Rect& rect2);