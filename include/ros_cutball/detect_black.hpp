#include <opencv2/opencv.hpp>
#include <sl_zed/Camera.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <ros_cutball/black_threshold_Config.h>
#include <ros_cutball/point2i.h>                   //自定义消息类型的头文件
#include <string.h>

class detect_black{
    public:
        detect_black(); 
        //virtual ~Detect_Color();

        void start();
        void threshold_update(int Threshold); //滑动条回调时更改颜色阈值
    private:
        //ros参数
        ros::NodeHandle nh_;
        image_transport::ImageTransport it_;
        image_transport::Subscriber image_sub_;         //订阅主图像
        //image_transport::Publisher image_pub_;        //发布框选后的图像[更新：不发布了]
        image_transport::Publisher threshold_pub_;    //发布二值化图像
        sensor_msgs::ImagePtr ros_image;              //cv_bridge转化ros图像指针
        sensor_msgs::ImagePtr ros_threshold;
        cv_bridge::CvImagePtr cv_ptr;               //  图像指针
        ros::Publisher point_pub;
        //boost::shared_ptr<dynamic_reconfigure::Server<ros_cutball::hsv_Config>> mDynRecServer; //动态参数配置

        //回调函数
        //void hsv_update(ros_cutball::hsv_Config &config, uint32_t level);
        void updateImage(const sensor_msgs::ImageConstPtr& msg);
        cv::Point2d outpoint;
        cv::Mat cvmat_raw;
        cv::Mat cvmat_thr;
        //std::vector<cv::Rect>& vec_rect_color;
        int threshold;                               //颜色阈值
        //int match_up(cv::Point& point,std::vector<cv::Point>& map);

        //中间变量
        
        void data_pub();
        void image_pub();
        //辅助函数
        void Proc_Img(cv::Mat& thresh);
        //bool comp_area(cv::Rect& Rect1,cv::Rect& Rect2);
        //bool comp_rect_y(cv::Rect& rect1,cv::Rect& rect2);
};