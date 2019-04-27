#include "Detect_Color.hpp"

class Detect_Color_node {
    private:
        void hsv_update_red(ros_cutball::red_hsv_Config &config, uint32_t level);
        void hsv_update_yellow(ros_cutball::yellow_hsv_Config &config, uint32_t level);
        //void hsv_update_black(ros_cutball::hsv_Config &config, uint32_t level);
        ros::NodeHandle nh;
        std::string syellow;
        std::string sred;
        Detect_Color dc;                  //需要使用参数服务器去调取构造函数初始化的参数，所以定义指针方便一些
        boost::shared_ptr<dynamic_reconfigure::Server<ros_cutball::red_hsv_Config>> red_DynRecServer; //动态参数配置
        boost::shared_ptr<dynamic_reconfigure::Server<ros_cutball::yellow_hsv_Config>> yellow_DynRecServer; //动态参数配置

        //void detect_color_init();
    public:
        Detect_Color_node(std::string scolor,int num,int __flag__=0):dc(scolor,num){
            switch(__flag__){
                case 0: {
                    yellow_DynRecServer = boost::make_shared<dynamic_reconfigure::Server<ros_cutball::yellow_hsv_Config>>();
                    dynamic_reconfigure::Server<ros_cutball::yellow_hsv_Config>::CallbackType fy;
                    fy = boost::bind(&Detect_Color_node::hsv_update_yellow,this, _1, _2);
                    yellow_DynRecServer->setCallback(fy);
                    break;
                }
                case 1: {
                    red_DynRecServer = boost::make_shared<dynamic_reconfigure::Server<ros_cutball::red_hsv_Config>>();
                    dynamic_reconfigure::Server<ros_cutball::red_hsv_Config>::CallbackType fr;
                    fr = boost::bind(&Detect_Color_node::hsv_update_red,this, _1, _2);
                    red_DynRecServer->setCallback(fr);
                    break;
                }
                case 2: {
                    //f = boost::bind(&Detect_Color_node::hsv_update_black,this, _1, _2);
                    //mDynRecServer->setCallback(f);
                    //break;
                }
                default: {
                    ROS_INFO("bad init of arg of the detect_color,check the parameter!");
                    ros::shutdown();
                }
            }
            //回调函数调用每一个类的改变阈值方法

        }
        void start() {
            dc.start();
        }
};

class test{
public:
    Detect_Color ccc;
    std::string aaa;
    test():ccc("yellow",1){ROS_INFO("object created successfully");}
    void start(){ccc.start();}
};

int __FLAG__;
int main(int argc, char** argv){
    ros::init(argc, argv, "Detect_Color");
    //动态滑动条改变阈值参数
    ros::NodeHandle nh;
    //获得需要检测的红黄气球数
    int num;
                 
    std::string argv1(argv[1]);
    if (argv[1] != nullptr) {
        if (argv1 == std::string("yellow")) {
            __FLAG__ = 0;
            nh.getParam("yellow_balloon",num);  //从参数服务器上获取定义检测的气球数量
        }
        else if (argv1 == std::string("red")) {
            __FLAG__ = 1; 
            nh.getParam("red_balloon",num); //从参数服务器上获取定义检测的气球数量
        }
        else if (argv1 == std::string("black")) {
            __FLAG__ = 2;
        }
        else{
            ROS_INFO("detect_color arg neither 'yellow' 'red' nor 'black',bad node init!");
            ros::shutdown();
        }
    }

    //我想把这些放到node的类中，但是这样就不能列表初始化detect类对象，所以需要动态指针初始化
    //detect_color_init();                          //但是就会出现大量问题，难于解决，后面的可以尝试一下
    Detect_Color_node dcn(argv1,num,__FLAG__);
    if (!num){
        ROS_INFO("one of the red or yellow param set to 0");
    }
    
    //std::string ss("yellow");
    //Detect_Color dcn(ss,1);
    //test dcn;
    ros::Rate loop_rate(15);
    while(nh.ok()) {
        dcn.start();
        ros::spinOnce(); 
        loop_rate.sleep();
    }
}
//********************通过argv参数判断开启的这个节点是用来干什么的，然后更新哪一个阈值********************************
//滑动条回调函数
void Detect_Color_node::hsv_update_yellow(ros_cutball::yellow_hsv_Config &config, uint32_t level) {
    dc.hsv_update(config.yellow_H1,config.yellow_S1,config.yellow_V1,config.yellow_H2,config.yellow_S2,config.yellow_V2);
    //red.hsv_update(config.red_H1,config.red_S1,config.red_V1,config.red_H2,config.red_S2,config.red_V2);
    //ROS_INFO("{H1:%d S1:%d V1:%d,H2:%d S2:%d V2:%d}",range_hsv[0],range_hsv[1],range_hsv[2],range_hsv[3],range_hsv[4],range_hsv[5]);
}

void Detect_Color_node::hsv_update_red(ros_cutball::red_hsv_Config &config, uint32_t level) {
    //dc.hsv_update(config.yellow_H1,config.yellow_S1,config.yellow_V1,config.yellow_H2,config.yellow_S2,config.yellow_V2);
    dc.hsv_update(config.red_H1,config.red_S1,config.red_V1,config.red_H2,config.red_S2,config.red_V2);
    //ROS_INFO("{H1:%d S1:%d V1:%d,H2:%d S2:%d V2:%d}",range_hsv[0],range_hsv[1],range_hsv[2],range_hsv[3],range_hsv[4],range_hsv[5]);
}

// void Detect_Color_node::hsv_update_black(ros_cutball::hsv_Config &config, uint32_t level) {
//     dc.hsv_update(config.black_threshold);
//     //red.hsv_update(config.red_H1,config.red_S1,config.red_V1,config.red_H2,config.red_S2,config.red_V2);
//     //ROS_INFO("{H1:%d S1:%d V1:%d,H2:%d S2:%d V2:%d}",range_hsv[0],range_hsv[1],range_hsv[2],range_hsv[3],range_hsv[4],range_hsv[5]);
// }
// void Detect_Color_node::detect_color_init(){
//     ylo = new Detect_Color("yellow",yellow_num);
//     red = new Detect_Color("red",red_num);
    
//     std::cout<<"object created successful"<<std::endl;
// }
