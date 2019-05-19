#include "detect_black.hpp"
bool comp_area(cv::Rect& Rect1,cv::Rect& Rect2)
{
    return Rect1.area() > Rect2.area();
}
detect_black::detect_black():it_(nh_){

    image_sub_ = it_.subscribe("/zed/rgb/image_raw_color", 1,&detect_black::updateImage, this);

    //image_pub_ = it_.advertise("/image_converter/output_raw", 1);
    threshold_pub_ = it_.advertise(std::string("/image_convert/black"),1);
    //TUDO后期添加两个服务的客户端，用于告诉这个服务气球下落减少检测的气球数
    //dynamic_reconfigure::Server<ros_cutball::hsv_Config> server;
    
    //server.setCallback(f);
    point_pub = nh_.advertise<ros_cutball::point2i>(std::string("Detect_Color/black_point"),10);
    nh_.getParam(std::string("threshold"),threshold);
    //cv::namedWindow("test");
}


void detect_black::updateImage(const sensor_msgs::ImageConstPtr& msg) {
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


void detect_black::start() {
    if (cvmat_raw.empty()) return;                  //初始时可能zed还没发布图像，所以订阅不到，直接返回
    cv::cvtColor(cvmat_raw,cvmat_thr,CV_BGR2GRAY);
    cv::threshold(cvmat_thr,cvmat_thr,threshold,255,CV_THRESH_BINARY_INV);
    //ROS_INFO("{H1:%d S1:%d V1:%d,H2:%d S2:%d V2:%d}",range_hsv[0],range_hsv[1],range_hsv[2],range_hsv[3],range_hsv[4],range_hsv[5]);
    //cv::imshow("test",cvmat_hsv);
    //cv::waitKey(1);
    Proc_Img(cvmat_thr);
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(cvmat_thr,contours,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);
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
    std::sort(rectangles.begin(),rectangles.end(),comp_area);
    if (rectangles.empty()) {
        outpoint.x = -1;
        outpoint.y = -1;
    }
    else {
        outpoint.x = rectangles[0].x + rectangles[0].width / 2;
        outpoint.y = rectangles[0].y + rectangles[0].height / 2;
    }
    image_pub();
    //ROS_INFO("end image_pub");
    data_pub();
}




void detect_black::Proc_Img(cv::Mat& thresh)
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


void detect_black::data_pub() {
    ros_cutball::point2i point;
    point.x = outpoint.x;
    point.y = outpoint.y;
    point_pub.publish(point);
}


void detect_black::image_pub() {
    // for (int i = 0;i < howmany;i++)
    //     cv::rectangle(cvmat_raw,vec_rect_color[i],cv::Scalar(255,0,0),2);
    //ros_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cvmat_raw).toImageMsg();
    //image_pub_.publish(ros_image);
    ros_threshold = cv_bridge::CvImage(std_msgs::Header(), "mono8", cvmat_thr).toImageMsg();
    threshold_pub_.publish(ros_threshold);
}


void detect_black::threshold_update(int Threshold) {
    threshold = Threshold;
}