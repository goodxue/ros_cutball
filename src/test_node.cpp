#include <opencv2/opencv.hpp>
#include <sl_zed/Camera.hpp>
#include <ros/ros.h>
#include <ros_cutball/TF.h>
#include <string>

char key;

bool srv_callback(ros_cutball::TF::Request& req,ros_cutball::TF::Response& res) {
    while(1) {
        key = cv::waitKey(1);
        if (key == 'q') {
            res.success = false;
            res.message = std::string("test");
            return false;
        }
        else if (key == 'g') {
            res.success = true;
            return true;
        }
    }
    return false;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;
    nh.advertiseService("balloon_falling",srv_callback);
    key = ' ';

    ros::Rate loop_rate(15);
    while(nh.ok()) {
        ros::spinOnce(); 
        loop_rate.sleep();
    }
    return 0;
}