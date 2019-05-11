#include "detect_black.hpp"

class detect_black_node {
    private:
        void threshold_update_black(ros_cutball::black_threshold_Config &config, uint32_t level);
        //void hsv_update_black(ros_cutball::hsv_Config &config, uint32_t level);
        ros::NodeHandle nh;
        std::string syellow;
        std::string sred;
        detect_black db;                  //需要使用参数服务器去调取构造函数初始化的参数，所以定义指针方便一些
        boost::shared_ptr<dynamic_reconfigure::Server<ros_cutball::black_threshold_Config>> black_DynRecServer; //动态参数配置

        //void detect_color_init();
    public:
        detect_black_node():db(){
            black_DynRecServer = boost::make_shared<dynamic_reconfigure::Server<ros_cutball::black_threshold_Config>>();
            dynamic_reconfigure::Server<ros_cutball::black_threshold_Config>::CallbackType fb;
            fb = boost::bind(&detect_black_node::threshold_update_black,this, _1, _2);
            black_DynRecServer->setCallback(fb);
        }
        void start() {
            db.start();
        }
    };

int main(int argc, char** argv){
    ros::init(argc, argv, "Detect_Color");
    //动态滑动条改变阈值参数
    ros::NodeHandle nh;
    //获得需要检测的红黄气球数
    detect_black_node dbn;
    ros::Rate loop_rate(15);
    while(nh.ok()) {
        dbn.start();
        ros::spinOnce(); 
        loop_rate.sleep();
    }                
}


void detect_black_node::threshold_update_black(ros_cutball::black_threshold_Config &config, uint32_t level) {
    db.threshold_update(config.threshold);
}