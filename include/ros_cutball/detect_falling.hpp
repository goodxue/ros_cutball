#include <opencv2/opencv.hpp>
#include <sl_zed/Camera.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <ros_cutball/rect.h>                   //自定义消息类型的头文件
#include <ros_cutball/rectArray.h>
#include <std_msgs/Float64.h>                   //高度话题消息类型
#include <ros_cutball/TF.h>                     //自定义服务
#include <vector>

class detect_falling {
public:
    //Detect_falling();
    detect_falling();
    bool start();

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber depth_sub_;         //订阅主图像
    ros::Subscriber sub_rectarray_red;      //红球boundingbox
    ros::Subscriber sub_rectarray_yellow;   //黄球boundingbox
    ros::Subscriber sub_height;             //飞机当前高度
    
    ros::ServiceClient srv_tf;          //ros服务，当有气球下落则暂停继续监测，等待飞机返回原点后继续,这是一个#client#

    void update_rect_red(const ros_cutball::rectArray::ConstPtr& msg);
    void update_rect_yellow(const ros_cutball::rectArray::ConstPtr& msg);       //更新红黄的boundingbox(用于确定气球上检测深度的几个点)
    void update_height(const std_msgs::Float64 Heights);                        //更新飞机当前高度
    void depthCallback(const sensor_msgs::Image::ConstPtr& msg);                //深度图的订阅

                                     
    //sl::Mat slmat_depth;                                //zed深度图（弃置不用，因为这个节点不需要保存深度图，只需要几个位置的深度信息即可）
    int red_num,yellow_num;                             //查看红色黄色球数量
    std::vector<cv::Rect> vec_rect_red;                 //红色boundingbox
    std::vector<cv::Rect> vec_rect_yellow;              //黄色boundingbox

    std::vector<std::vector<cv::Point2d>> key_pixel;      //二维数组，每个元素是一个要检测的气球需要监测深度的点             
    void weighted_average_to_key_pixel();                 //返回加权平均值，表示要检测深度的点。数据保存在key_pixel

};