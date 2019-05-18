#include "detect_falling.hpp"

detect_falling::detect_falling(std::string parameter):it_(nh_) {
    sub_rectarray_red = nh_.subscribe("/detect_color/detect_red/Detect_Color/rect/red", 1,&detect_falling::update_rect_red, this);
    sub_rectarray_yellow = nh_.subscribe("/detect_color/detect_yellow/Detect_Color/rect/yellow", 1,&detect_falling::update_rect_yellow, this);
    sub_height = nh_.subscribe("xdd_z",1,&detect_falling::update_height,this);
    depth_sub_ = it_.subscribe("/zed/depth/depth_registered", 1,&detect_falling::depthCallback,this);
    srv_tf = nh_.serviceClient<ros_cutball::TF>("balloon_falling");

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

    tf_call[1] = false;
    tf_call[2] = false;
    tf_call[3] = false;
    tf_call[4] = false;
}


bool detect_falling::start() {
    
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
    if (height_queue.size() >= queue_num) {
        height_queue.pop();
        height_queue.push(Heights.data);
    }
    else {
        height_queue.push(Heights.data);
    }
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


//  暂定象限为        1
//            2           3
//                  4
//注意检测颜色时发布的气球顺序是y值由大到小，即y最大的在第四象限
bool detect_falling::check_falling() {
    for (int i = 0;i < red_depth_queue_vec.size();i++) {
        double difference;
        //气球绝对高度差的变化，气球相对相机的距离变化，减去飞机相对地面的距离变化
        difference = abs(red_depth_queue_vec[i].front() - red_depth_queue_vec[i].back())-abs(height_queue.front()-height_queue.back());
        if (difference > falling_threshold) {
            //计算出红球的象限，然后调用服务
            ros_cutball::TF tf;
            tf.request.tf = calculate_tf(vec_rect_red[i]);
            if(srv_tf.call(tf)) {
                reinitialize();                     //重新开启检测下落节点
            }
            else {
                ROS_ERROR("in detect_falling::check_falling: service returned false, fatel error occured");
                return false;
            }
        }
    }
    //接着写黄球的掉落检测
    for (int i = 0;i < yellow_depth_queue_vec.size();i++) {
        double difference;
        //气球绝对高度差的变化，气球相对相机的距离变化，减去飞机相对地面的距离变化
        difference = abs(yellow_depth_queue_vec[i].front() - yellow_depth_queue_vec[i].back())-abs(height_queue.front()-height_queue.back());
        if (difference > falling_threshold) {
            //计算出红球的象限，然后调用服务
            ros_cutball::TF tf;
            int tf_temp = calculate_tf(vec_rect_yellow[i]);
            switch(tf_temp) {               //象限反向
                case 1: tf.request.tf = 4;
                        break;
                case 2: tf.request.tf = 3;
                        break;
                case 3: tf.request.tf = 2;
                        break;
                case 4: tf.request.tf = 1;
                        break; 
            }
            if(srv_tf.call(tf)) {
                reinitialize();                     //重新开启检测下落节点
            }
            else {
                ROS_ERROR("in detect_falling::check_falling: service returned false, fatel error occured");
                return false;
            }
        }
    }
}

//输入一个点的坐标，先判断是不是最上或最下，再判断是不是最左或最右
int detect_falling::calculate_tf(cv::Rect object) {
    
    std::vector<cv::Rect> four_rect;
    for (int i=0;i < vec_rect_red.size();i++) {
        four_rect.push_back(vec_rect_red[i]);
    }
    for (int i=0;i < vec_rect_yellow.size();i++) {
        four_rect.push_back(vec_rect_yellow[i]);
    }
    while (four_rect.size() < 4) {          //确保里面有四个，这样才便于划分象限
        four_rect.push_back(cv::Rect());
    }

    std::sort(four_rect.begin(),four_rect.end(),comp_rect_x);
    if ((*four_rect.begin()) == object && (!tf_call[3])) {
        tf_call[3] = true;
        return 3;
    }
    else if ((*four_rect.end()) == object && (!tf_call[2])) {
        tf_call[2] = true;
        return 2;
    }
    else {
        std::sort(four_rect.begin(),four_rect.end(),comp_rect_y);
        if ((*four_rect.begin()) == object && (!tf_call[4])) {
            tf_call[4] = true;
            return 4;
        }
        else if ((*four_rect.end()) == object && (!tf_call[1])) {
            tf_call[1] = true;
            return 1;
        }
        else {
            ROS_INFO("an error occurs in calculate_tf(), where object has no tf number!");
        }
    }
    
}


bool comp_rect_y(cv::Rect& rect1,cv::Rect& rect2) {
    return rect1.y > rect2.y;
}

bool comp_rect_x(cv::Rect& rect1,cv::Rect& rect2) {
    return rect1.x > rect2.x;
}

//很大可能出问题,先马上
bool detect_falling::reinitialize() {
    nh_.getParam("red_balloon",red_num);
    nh_.getParam("yellow_balloon",yellow_num);
    nh_.getParam("weight_value",weight_value);
    nh_.getParam("weight_point_num",weight_point_num);
    nh_.getParam("queue_num",queue_num);
    red_depth_queue_vec.clear();
    yellow_depth_queue_vec.clear();
    red_depth_queue_vec.resize(red_num+yellow_num);
    yellow_depth_queue_vec.resize(red_num+yellow_num);
}