#include "detect_falling.hpp"

detect_falling::detect_falling(std::string parameter):it_(nh_) {
    sub_rectarray_red = nh_.subscribe("/detect_color/detect_red/Detect_Color/rect/red", 1,&detect_falling::update_rect_red, this);
    sub_rectarray_yellow = nh_.subscribe("/detect_color/detect_yellow/Detect_Color/rect/yellow", 1,&detect_falling::update_rect_yellow, this);
    sub_height = nh_.subscribe("xdd_z",1,&detect_falling::update_height,this);
    depth_sub_ = it_.subscribe("/zed/depth/depth_registered", 1,&detect_falling::depthCallback,this);
    srv_tf = nh_.serviceClient<ros_cutball::TF>("/balloon_falling");
    srv_check = nh_.advertiseService("trigger",&detect_falling::checking_trigger,this);
    call_update_howmany_red = nh_.serviceClient<std_srvs::Trigger>("/detect_color/detect_red/red_balloon_fell");
    call_update_howmany_yellow = nh_.serviceClient<std_srvs::Trigger>("/detect_color/detect_red/yellow_balloon_fell");
    //初始化一些用于保存坐标点的vector
    //参数服务器获取超参数
    nh_.getParam("/detect_color/detect_red/red_balloon",red_num);
    ROS_INFO("red_balloon_num: %d",red_num);
    nh_.getParam("/detect_color/detect_yellow/yellow_balloon",yellow_num);
    ROS_INFO("yellow_balloon_num: %d",yellow_num);
    nh_.getParam("weight_value",weight_value);              //从中心点向外扩展的像素点值
    nh_.getParam("weight_point_num",weight_point_num);      //取点的个数，默认为(weight_point_num+1)^2 默认为2
    nh_.getParam("queue_num",queue_num);                    //队列最大存储深度的个数,即判断下落时取出首位
    nh_.getParam("falling_threshold_m",falling_threshold_m);    //下落的阈值
    nh_.getParam("falling_threshold_mm",falling_threshold_mm);    //下落的阈值
    nh_.getParam("/zed/zed_wrapper_node/openni_depth_mode",openni_depth_mode);  //深度数值的单位。0是米为单位，1是毫米为单位
    falling_threshold = openni_depth_mode? falling_threshold_mm:falling_threshold_m;

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
    if (red_num > 0) {
        vec_rect_red.resize(red_num);
    }
    if (yellow_num > 0) {
        vec_rect_yellow.resize(yellow_num);
    }
    red_depth_queue_vec.resize(red_num);
    yellow_depth_queue_vec.resize(yellow_num);

    tf_call[1] = false;
    tf_call[2] = false;
    tf_call[3] = false;
    tf_call[4] = false;
    time_decay = 0;
    start_checking = false;
    height_queue.push(0);
}


bool detect_falling::start() {
    if (start_checking) {
        if (check_falling()) {
            return true;
        }
    }
    else {
        return false;
    }
}


//直接订阅深度图像，然后利用它的ptr直接读数据就行（因为不需要修改，所以不使用cv_bridge)
//读取的目标点来自key_pixel_2d
void detect_falling::depthCallback(const sensor_msgs::Image::ConstPtr& msg) {
    //ROS_INFO("depth_callback! in...");
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
        //将关键点放进数组
        weighted_average_to_key_pixel(vec_rect_red,red_key_pixel_2d);

        float* depths = (float*)(&msg->data[0]);                    //图像变为一位的全连接
        //红球深度装入队列
        for (int i = 0;i < red_key_pixel_2d.size();i++) {
            double all_sum = 0; //深度之和
            for (int j = 0;j < red_key_pixel_2d[i].size();j++) {
                int u = red_key_pixel_2d[i][j].x;                   //横坐标
                int v = msg->width * red_key_pixel_2d[i][j].y;      //纵坐标的所在的行
                all_sum += depths[u+v];
            }
            if (red_key_pixel_2d[i].size()) {
                //ROS_INFO("red[%d]:size:%d,  value:%f ,all_sum:%lf",i,red_key_pixel_2d[i].size(),all_sum/red_key_pixel_2d[i].size(),all_sum);
                depth_push_into_queue(red_depth_queue_vec[i],all_sum/red_key_pixel_2d[i].size());     //all_sum除以这个向量point的个数，即平均值
            }
        }
        weighted_average_to_key_pixel(vec_rect_yellow,yellow_key_pixel_2d);
        //ROS_INFO("Finish weight_average_to_key_pixel in depth_callback! out...");
        //黄球深度装入队列
        for (int i = 0;i < yellow_key_pixel_2d.size();i++) {
            double all_sum = 0; //深度之和
            for (int j = 0;j < yellow_key_pixel_2d[i].size();j++) {
                int u = yellow_key_pixel_2d[i][j].x;                   //横坐标
                int v = msg->width * yellow_key_pixel_2d[i][j].y;      //纵坐标的所在的行
                all_sum += depths[u+v];
            }
            if(yellow_key_pixel_2d[i].size()) {
                //ROS_INFO("yellow:i : %d;  yellow_key_pixel_2d[i].size(): %d",i,yellow_key_pixel_2d[i].size());
                //ROS_INFO("yellow[%d]:size:%d,  value:%f ,all_sum:%lf,yellow_depth_queue_vec[i].size():%d",i,yellow_key_pixel_2d[i].size(),all_sum/yellow_key_pixel_2d[i].size(),all_sum,yellow_depth_queue_vec[i].size());
                depth_push_into_queue(yellow_depth_queue_vec[i],all_sum/yellow_key_pixel_2d[i].size());     //all_sum除以这个向量point的个数，即平均值
            }
        }
    }
    //ROS_INFO("depth_callback! out...");
}

void detect_falling::update_rect_red(const ros_cutball::rectArray::ConstPtr& msg) {
    //ROS_INFO("update_rect_red! in...");
    vec_rect_red.resize(msg->rectarray.size());               //预分配大小，使用stl的vector一定要小心越界
    for (int i=0; i < msg->rectarray.size(); ++i)
    {
        vec_rect_red[i].x = msg->rectarray[i].x;
        vec_rect_red[i].y = msg->rectarray[i].y;
        vec_rect_red[i].width = msg->rectarray[i].width;
        vec_rect_red[i].height = msg->rectarray[i].height;
        //vec_rect_red[i] = rect_temp;
    }
    //ROS_INFO("update_rect_red! out...");
}


void detect_falling::update_rect_yellow(const ros_cutball::rectArray::ConstPtr& msg) {
    //ROS_INFO("update_rect_yellow! in...");
    vec_rect_yellow.resize(msg->rectarray.size());               //预分配大小，使用stl的vector一定要小心越界
    for (int i=0; i < msg->rectarray.size(); ++i)
    {
        vec_rect_yellow[i].x = msg->rectarray[i].x;
        vec_rect_yellow[i].y = msg->rectarray[i].y;
        vec_rect_yellow[i].width = msg->rectarray[i].width;
        vec_rect_yellow[i].height = msg->rectarray[i].height;
        //vec_rect_yellow[i] = rect_temp;
    }
    //ROS_INFO("update_rect_yellow! out...");
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
            ROS_FATAL("fatel error occurs: server returned a false state for: %s",tf.response.message.data());
            ROS_FATAL("shutting down the detect_falling node");
            ros::shutdown();
        }
    }
    else {
        ROS_INFO("fatel error occurs: Failed to call the server.");
        return false;
    }
    return true;
}

//先不使用复杂的取点了，先直接取框选的中心坐标方便调试
void detect_falling::weighted_average_to_key_pixel(std::vector<cv::Rect>& rect_vec,std::vector<std::vector<cv::Point2i>>& vec_vec_point) {
    // //ROS_INFO("weight_average_to_key_pixel! in...");
    // vec_vec_point.resize(rect_vec.size());           //将二维数组先分配空间
    // for (int i = 0;i < rect_vec.size();i++) {
    //     int tempx = rect_vec[i].x;          //左上角坐标点
    //     int tempy = rect_vec[i].y;
    //     //ROS_INFO("weight_average_to_key_pixel! first loop...i= %d",i);
    //     vec_vec_point[i].resize((weight_point_num+1)*(weight_point_num+1));
    //     for (int j = 0;j < weight_point_num;j++) {
    //         //ROS_INFO("weight_average_to_key_pixel! second loop...j= %d",j);
    //         for (int k = 0;k < weight_point_num;k++) {
    //             //ROS_INFO("weight_average_to_key_pixel! third loop...k= %d",k);
    //             vec_vec_point[i][j] = cv::Point2i(tempx+j*weight_value+k*weight_value,tempy+j*weight_value+k*weight_value);      //TUDO 将待检测八个点放进二维数组中，判断下红黄球顺序这种问题
    //         }
    //     }
    // }
    // //ROS_INFO("weight_average_to_key_pixel! out...");
    vec_vec_point.resize(rect_vec.size());
    for (int i = 0;i < rect_vec.size();i++) {
        vec_vec_point[i].resize(1);
        vec_vec_point[i][0] = cv::Point2i(rect_vec[i].x+rect_vec[i].width/2,rect_vec[i].y+rect_vec[i].height/2);
    }
}


void detect_falling::depth_push_into_queue(std::queue<float>& depth_queue,float depth) {
    //ROS_INFO("depth_push_into_queue! in... empty:%d",depth_queue.empty());
    if (std::isnan(depth)) return;
    //ROS_INFO("size de wen ti");
    if (depth_queue.size()>queue_num) {
        depth_queue.pop();
        depth_queue.push(depth);
    }
    else {
        depth_queue.push(depth);
    }
    //ROS_INFO("depth_push_into_queue! out...");
}


//  暂定象限为        1
//            2           3
//                  4
//注意检测颜色时发布的气球顺序是y值由大到小，即y最大的在第四象限
bool detect_falling::check_falling() {
    //ROS_INFO("check_falling! in...");
    for (int i = 0;i < red_depth_queue_vec.size();i++) {
        float difference;
        //气球绝对高度差的变化，气球相对相机的距离变化，减去飞机相对地面的距离变化
        difference = abs(abs(red_depth_queue_vec[i].front() - red_depth_queue_vec[i].back())-abs(height_queue.front()-height_queue.back()));
        ROS_INFO(" red[%d] difference: %f ;front:[%f],back:[%f]",i,difference,red_depth_queue_vec[i].front(),red_depth_queue_vec[i].back());
        if (difference > falling_threshold) {
            //计算出红球的象限，然后调用服务
            ros_cutball::TF tf;
            tf.request.tf = calculate_tf(vec_rect_red[i]);
            ROS_INFO_STREAM("red_falling tf:"<<tf.request.tf);
            if(srv_tf.call(tf)) {
                std_srvs::Trigger tri;
                if(call_update_howmany_red.call(tri)){
                    ROS_INFO("update_homany_red successfully...");
                }
                else {
                    ROS_INFO_STREAM("unsuccessfully call update_homany_red with a message :"<<tri.response.message);
                }
                reinitialize();                     //重新开启检测下落节点
            }
            else {
                ROS_ERROR("in detect_falling::check_falling: service returned false, fatel error occured");
                return false;
            }
        }
    }
    //ROS_INFO("depth_push_into_queue! after red_tf...");
    //接着写黄球的掉落检测
    for (int i = 0;i < yellow_depth_queue_vec.size();i++) {
        float difference;
        //气球绝对高度差的变化，气球相对相机的距离变化，减去飞机相对地面的距离变化
        difference = abs(abs(yellow_depth_queue_vec[i].front() - yellow_depth_queue_vec[i].back())-abs(height_queue.front()-height_queue.back()));
        ROS_INFO("yellow[%d] difference: %f ;fro:[%f],back:[%f]",i,difference,yellow_depth_queue_vec[i].front(),yellow_depth_queue_vec[i].back());
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
            ROS_INFO_STREAM("yellow_falling tf:"<<tf.request.tf);
            if(srv_tf.call(tf)) {
                std_srvs::Trigger tri;
                if(call_update_howmany_yellow.call(tri)) {
                    ROS_INFO("update_homany_yellow successfully...");
                }
                else {
                    ROS_INFO_STREAM("unsuccessfully call update_homany_yellow with a message :"<<tri.response.message);
                }
                reinitialize();                     //重新开启检测下落节点
            }
            else {
                ROS_ERROR("in detect_falling::check_falling: service returned false, fatel error occured");
                return false;
            }
        }
    }
    //ROS_INFO("depth_push_into_queue! after yellow_difference... out...");
}

//输入一个点的坐标，先判断是不是最上或最下，再判断是不是最左或最右
int detect_falling::calculate_tf(cv::Rect object) {
    ROS_INFO("calculate_tf! in...");
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
            ROS_WARN("an error occurs in calculate_tf(), where object has no tf number!");
        }
    }
    ROS_INFO("calculate_tf! out...");
}


bool comp_rect_y(cv::Rect& rect1,cv::Rect& rect2) {
    return rect1.y > rect2.y;
}

bool comp_rect_x(cv::Rect& rect1,cv::Rect& rect2) {
    return rect1.x > rect2.x;
}

//很大可能出问题,先马上
bool detect_falling::reinitialize() {
    nh_.getParam("/detect_color/detect_red/red_balloon",red_num);
    nh_.getParam("/detect_color/detect_yellow/yellow_balloon",yellow_num);
    nh_.getParam("weight_value",weight_value);
    nh_.getParam("weight_point_num",weight_point_num);
    nh_.getParam("queue_num",queue_num);
    red_depth_queue_vec.clear();
    yellow_depth_queue_vec.clear();
    red_depth_queue_vec.resize(red_num);
    yellow_depth_queue_vec.resize(yellow_num);
    start_checking = false;                             //注意要将开始下落检测设置为false
    time_decay = 0;
}

//服务扳机的回调函数，将布尔值取反
bool detect_falling::checking_trigger(std_srvs::Trigger::Request& req,std_srvs::Trigger::Response& res) {
    start_checking = !start_checking;
    res.success = true;
    return true;
}