#include <opencv2/opencv.hpp>
#include <sl_zed/Camera.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <ros_cutball/red_hsv_Config.h>
#include <ros_cutball/yellow_hsv_Config.h>
#include <std_srvs/Trigger.h>
#include <ros_cutball/rect.h>                   //自定义消息类型的头文件
#include <ros_cutball/rectArray.h>
#include <string.h>

class Detect_Color{
    public:
        Detect_Color(std::string Scolor,int hmany = 1); //第一个参数是检测的颜色，例如黄球、红球，因为每个对象的颜色阈值不一样，所以需要创建多个对象
        //virtual ~Detect_Color();

        void start();
        void hsv_update(int H1,int S1,int V1,int H2,int S2,int V2); //滑动条回调时更改颜色阈值
    private:
        std::string scolor;                             //颜色，名称字符串
        //ros参数
        ros::NodeHandle nh_;
        image_transport::ImageTransport it_;
        image_transport::Subscriber image_sub_;         //订阅主图像
        //image_transport::Publisher image_pub_;        //发布框选后的图像[更新：不发布了]
        image_transport::Publisher threshold_pub_;    //发布二值化图像
        sensor_msgs::ImagePtr ros_image;              //cv_bridge转化ros图像指针
        sensor_msgs::ImagePtr ros_threshold;
        cv_bridge::CvImagePtr cv_ptr;               //  图像指针
        ros::ServiceServer update_howmany;          //ros服务，当有气球下落则减少需要检测的气球数
        ros::Publisher counter_pub;                 //发布检测到的矩形框
        //boost::shared_ptr<dynamic_reconfigure::Server<ros_cutball::hsv_Config>> mDynRecServer; //动态参数配置

        //回调函数
        //void hsv_update(ros_cutball::hsv_Config &config, uint32_t level);
        void updateImage(const sensor_msgs::ImageConstPtr& msg);
        bool Update_howmany(std_srvs::Trigger::Request &req,std_srvs::Trigger::Response &res);
        cv::Mat cvmat_raw;
        cv::Mat cvmat_hsv;
        //std::vector<cv::Rect>& vec_rect_color;
        int range_hsv[6];                               //颜色阈值
        std::vector<cv::Point> first_rect_pos;
        std::vector<cv::Rect> vec_rect_color;
        std::vector<bool> now_has_point;
        unsigned int find_ball_num;
        int howmany;                                    // 检测多少个气球，接收一个service，当切掉一个则减少
        bool find_ball;
        //int match_up(cv::Point& point,std::vector<cv::Point>& map);

        //中间变量
        
        

        void data_pub();
        void image_pub();
        //辅助函数
        void Proc_Img(cv::Mat& thresh);
        //bool comp_area(cv::Rect& Rect1,cv::Rect& Rect2);
        //bool comp_rect_y(cv::Rect& rect1,cv::Rect& rect2);
};