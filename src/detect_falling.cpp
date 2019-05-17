#include "detect_falling.hpp"

detect_falling::detect_falling(std::string parameter):it_(nh_) {
    sub_rectarray_red = nh_.subscribe("/detect_color/detect_red/Detect_Color/rect/red", 1,&detect_falling::update_rect_red, this);
    sub_rectarray_yellow = nh_.subscribe("/detect_color/detect_yellow/Detect_Color/rect/yellow", 1,&detect_falling::update_rect_yellow, this);
    sub_height = nh_.subscribe("xdd_z",1,&detect_falling::update_height,this);
    depth_sub_ = it_.subscribe("/zed/depth/depth_registered", 1,&detect_falling::depthCallback,this);
    srv_tf = nh_.serviceClient<ros_cutball::TF>("ball_falling");

    //初始化一些用于保存坐标点的vector
    //参数服务器获取超参数
    nh_.getParam("red_balloon",red_num);
    nh_.getParam("yellow_balloon",yellow_num);
    nh_.getParam("weight_value",weight_value);
    nh_.getParam("weight_point_num",weight_point_num);
    nh_.getParam("queue_num",queue_num);

    if (parameter == std::string("test")) {
        test = true;
        if (red_num > 0) {
            vec_rect_red.resize(red_num);
        }
        if (yellow_num > 0) {
            vec_rect_yellow.resize(yellow_num);
        }
    } else {
        test = false;
    }
    red_depth_queue_vec.resize(red_num+yellow_num);
    yellow_depth_queue_vec.resize(red_num+yellow_num);
}

//直接订阅深度图像，然后利用它的ptr直接读数据就行（因为不需要修改，所以不使用cv_bridge)
//读取的目标点来自key_pixel_2d
void detect_falling::depthCallback(const sensor_msgs::Image::ConstPtr& msg) {

    if (test) {
        // Get a pointer to the depth values casting the data
        // pointer to floating point
        float* depths = (float*)(&msg->data[0]);

        // Image coordinates of the center pixel
        int u = msg->width / 2;
        int v = msg->height / 2;

        // Linear index of the center pixel
        int centerIdx = u + msg->width * v;

        // Output the measure
        ROS_INFO("Center distance : %g m", depths[centerIdx]);
    }
    else {
        float* depths = (float*)(&msg->data[0]);                    //图像变为一位的全连接
        //红球深度装入队列
        for (int i = 0;i < red_key_pixel_2d.size();i++) {
            float all_sum = 0; //深度之和
            for (int j = 0;j < red_key_pixel_2d[i].size();j++) {
                int u = red_key_pixel_2d[i][j].x;                   //横坐标
                int v = msg->width * red_key_pixel_2d[i][j].y;      //纵坐标的所在的行
                all_sum += depths[u+v];
            }
            depth_push_into_queue(red_depth_queue_vec[i],all_sum/red_key_pixel_2d[i].size());     //all_sum除以这个向量point的个数，即平均值
        }
        //黄球深度装入队列
        for (int i = 0;i < yellow_key_pixel_2d.size();i++) {
            float all_sum = 0; //深度之和
            for (int j = 0;j < yellow_key_pixel_2d[i].size();j++) {
                int u = yellow_key_pixel_2d[i][j].x;                   //横坐标
                int v = msg->width * yellow_key_pixel_2d[i][j].y;      //纵坐标的所在的行
                all_sum += depths[u+v];
            }
            depth_push_into_queue(yellow_depth_queue_vec[i],all_sum/yellow_key_pixel_2d[i].size());     //all_sum除以这个向量point的个数，即平均值
        }

    }
}

void detect_falling::update_rect_red(const ros_cutball::rectArray::ConstPtr& msg) {
    vec_rect_red.resize(msg->rectarray.size());               //预分配大小，使用stl的vector一定要小心越界
    for (int i=0; i < msg->rectarray.size(); ++i)
    {
        vec_rect_red[i].x = msg->rectarray[i].x;
        vec_rect_red[i].y = msg->rectarray[i].y;
        vec_rect_red[i].width = msg->rectarray[i].width;
        vec_rect_red[i].height = msg->rectarray[i].height;
        //vec_rect_red[i] = rect_temp;
    }
}


void detect_falling::update_rect_yellow(const ros_cutball::rectArray::ConstPtr& msg) {
    vec_rect_yellow.resize(msg->rectarray.size());               //预分配大小，使用stl的vector一定要小心越界
    for (int i=0; i < msg->rectarray.size(); ++i)
    {
        vec_rect_yellow[i].x = msg->rectarray[i].x;
        vec_rect_yellow[i].y = msg->rectarray[i].y;
        vec_rect_yellow[i].width = msg->rectarray[i].width;
        vec_rect_yellow[i].height = msg->rectarray[i].height;
        //vec_rect_yellow[i] = rect_temp;
    }
}

//飞机发布的高度信息，回调函数更改内部的height值。
void detect_falling::update_height(const std_msgs::Float64 Heights) {
    height = Heights.data;
}



void detect_falling::update_rect_red(const ros_cutball::rectArray::ConstPtr& msg) {
    vec_rect_yellow.resize(msg->rectarray.size());               //预分配大小，使用stl的vector一定要小心越界
    for (int i=0; i < msg->rectarray.size(); ++i)
    {
        vec_rect_yellow[i].x = msg->rectarray[i].x;
        vec_rect_yellow[i].y = msg->rectarray[i].y;
        vec_rect_yellow[i].width = msg->rectarray[i].width;
        vec_rect_yellow[i].height = msg->rectarray[i].height;
        //vec_rect_yellow[i] = rect_temp;
    }
}

//飞机发布的高度信息，回调函数更改内部的height值。
void detect_falling::update_height(const std_msgs::Float64 Heights) {
    height = Heights.data;
}


//气球下落，向客户端发送请求的函数封装，当请求发出后，程序即暂停在这里等待服务端回应：
//当飞机先跑到提前设定的象限的定点后，然后根据黑圆坐标判断返回原点后再向客户端发送成功
//信息，然后检测下落的程序才开始继续。
bool detect_falling::call_service(int tf_num) {
    ros_cutball::TF tf;
    tf.request.tf = tf_num;
    ROS_INFO("calling service on \"ball_falling\" of tf %d ...",tf_num);
    if (srv_tf.call(tf)){
        if (tf.response.success) {
            ROS_INFO("UAV has return to original place, ready to resume the falling_detection.");

        }
        else {
            ROS_INFO("fatel error occurs: server returned a false state for %s",tf.response.message);
            ROS_INFO("shutting down the detect_falling node");
            ros::shutdown();
        }
    }
    else {
        ROS_INFO("fatel error occurs: Failed to call the server.");
        return false;
    }
    return true;
}


void detect_falling::weighted_average_to_key_pixel(std::vector<cv::Rect>& rect_vec,std::vector<std::vector<cv::Point2d>>& vec_vec_point) {
    vec_vec_point.resize(red_num+yellow_num);           //将二维数组先分配空间
    for (int i = 0;i < rect_vec.size();i++) {
        int tempx = rect_vec[i].x;          //左上角坐标点
        int tempy = rect_vec[i].y;
        vec_vec_point[i].resize((weight_point_num+1)*(weight_point_num+1));
        for (int j = 0;j < weight_point_num;j++) {
            for (int k = 0;k < weight_point_num;k++) {
                vec_vec_point[i][j] = cv::Point2d(tempx+j*weight_value+k*weight_value,tempy+j*weight_value+k*weight_value);      //TUDO 将待检测八个点放进二维数组中，判断下红黄球顺序这种问题
            }
        }
    }
}


void detect_falling::depth_push_into_queue(std::queue<float>& depth_queue,float depth) {
    if (depth_queue.size()>queue_num) {
        depth_queue.pop();
        depth_queue.push(depth);
    }
    else {
        depth_queue.push(depth);
    }
}