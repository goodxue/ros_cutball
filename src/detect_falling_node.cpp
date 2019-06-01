#include "detect_falling.hpp"

int main(int argc, char** argv)  {
    ros::init(argc, argv, "Detect_falling");
    ros::NodeHandle nh;
    detect_falling DF(" ");


    ros::Rate loop_rate(15);
    while(nh.ok()) {
        DF.start();
        ros::spinOnce(); 
        loop_rate.sleep();
    }
    return 0;
}