#include <nav_vel_plan/Follow.h>

Follow::Follow(ros::NodeHandle &n)
:private_nh_(n)
{
    //Parameter initialization
    InitParameters();
    InitRosParameters();
}

Follow::~Follow()
{
}

void Follow::InitRosParameters()
{
    // Initialize the subscribers and publishers
    vel_pub_ = private_nh_.advertise<geometry_msgs::Twist>("robot2/cmd_vel", 1);
    comm_pub_ = private_nh_.advertise<std_msgs::String>("comm", 1);
    
    scan_sub_ = private_nh_.subscribe("/robot2/scan", 1,&Follow::scanCallback, this );
    
}
void Follow::InitParameters()
{
    scan_ok_ = false;
    too_far_ = false;
    robot1_find_me_ = false;
    tf_flag_ = false;
    tf_Mode_ = true;
    image_show_ = false;
    commStr_.data = "start";
	send_flag_ = false;
	cancel_flag_ = false;
    far_to_near_ = false;

    front_len_ = 10; left_len_ = 20; right_len_ =20; rear_len_ = 10;
    check_len_ = 24;

    map_frame_id = "/map";
    odom_frame_id = "robot2_tf/odom";

    laser_x_ = 0; laser_y_ = 0;
}
void Follow::setMoveBase(MoveBaseClient *ac)
{
    ac_ = ac;
}

void Follow::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{   
    scan_ok_ = true;
    scanData_.header = msg->header;
    scanData_.angle_min = msg->angle_min; scanData_.angle_max = msg->angle_max;
    scanData_.angle_increment = msg->angle_increment; scanData_.time_increment = msg->time_increment;
    scanData_.scan_time = msg->scan_time; 
    scanData_.range_min = msg->range_min; scanData_.range_max = msg->range_max;
    scanData_.ranges = msg->ranges; scanData_.intensities = msg->intensities;

    std::vector<double> temp_f(front_len_);
    std::vector<double> temp_l(left_len_);
    std::vector<double> temp_r(right_len_);
    std::vector<double> temp_rear(rear_len_);
    
    //Clear the lidar jump point
    leap_index_.clear();
    for(int i = 0; i<360; i++)
    {
        //front
        if(i < front_len_/2 || i >= 360 - front_len_/2)
        {
            if(i < front_len_/2)
            {
                temp_f[i] = scanData_.ranges[i];
            }
            else
            {
                temp_f[front_len_ -(360-i)] = scanData_.ranges[i];
            }
        }

        //On the left, take left_len_ degree
        if(i < (90 + left_len_/2) && i >= (90 - left_len_/2))
        {
            temp_l[(i - 90) + left_len_/2] = scanData_.ranges[i];
        }

        //rear
        if( i < (180 + rear_len_/2) && i >= (180 - rear_len_/2))
        {
            temp_rear[ (i -180) + rear_len_/2] = scanData_.ranges[i];
        }

        //On the right, take right_len_ degree
        if( i < (270 + right_len_/2) && i >= (270 - right_len_/2))
        {
            temp_r[ (i - 270) + right_len_/2 ] = scanData_.ranges[i];
        }

        //Detect jump points
        if(i > 0)
        {
            if(abs(scanData_.ranges[i]-scanData_.ranges[i-1]) > 0.15)
            {
                if(scanData_.ranges[i] < scanData_.ranges[i-1])
                {
                    leap_index_.push_back(i);
                }
            }
        }
    
    }

    sample_front_ = temp_f;
    sample_left_ = temp_l; sample_right_= temp_r; sample_rear_ = temp_rear;

    //The minimum scanning distance of lidar in four directions is obtained
    front_min_ =  *min_element(sample_front_.begin(), sample_front_.end());
    left_min_ = *min_element(sample_left_.begin(), sample_left_.end());
    right_min_ = *min_element(sample_right_.begin(), sample_right_.end());
    rear_min_ = *min_element(sample_rear_.begin(), sample_rear_.end());



}

void Follow::mainloop()
{
    //Call the callback function
    ros::spinOnce();

    checkRobot();
    tfCalcAndPub();
    startFollow();

    //Publish the state on the /comm topic
    this->comm_pub_.publish(commStr_);
}

void Follow::startFollow()
{
    if(tf_flag_ != true)
    {
        return;
    }
    try
    {

        //Look for the coordinate transformation of map->robot1_tf/base_footprint
        tf::StampedTransform transform;
        listener_.waitForTransform("map", "robot1_tf/base_footprint", ros::Time(0), 
        ros::Duration(0.03));
        listener_.lookupTransform("map", "robot1_tf/base_footprint", ros::Time(0), 
        transform);

        //Look for the coordinate transformation of map->robot1_tf/base_footprint
        tf::StampedTransform transform1;
        listener_.waitForTransform("map", "robot1_tf/base_footprint", ros::Time(0), 
        ros::Duration(0.03));
        listener_.lookupTransform("map", "robot1_tf/base_footprint", ros::Time(0), 
        transform1);

        //Look for the coordinate transformation of robot2_tf/odom->robot2_tf/base_footprint
        tf::StampedTransform transform2;
        listener_.waitForTransform("robot2_tf/odom", "robot2_tf/base_footprint", ros::Time(0), 
        ros::Duration(0.03));
        listener_.lookupTransform("robot2_tf/odom", "robot2_tf/base_footprint", ros::Time(0), 
        transform2);

        //Compute the vector coordinates robot2 to robot1
        double t_x = transform1.getOrigin().getX() - transform2.getOrigin().getX();
        double t_y = transform1.getOrigin().getY() - transform2.getOrigin().getY();

        //Compute vector moduli
        double dis = sqrt(pow(t_x, 2) + pow(t_y,2) );

        //Detect whether robot 1 finds robot 2 and change the marker position after it finds it;
        //Continue to check the distance between robot 1 and 2, and the distance is less than 
        //the threshold value, indicating that it reaches the side of robot 1
        if(dis < 0.3 
        && robot1_find_me_ == false)
        {
            // ac_->cancelGoal();
            robot1_find_me_ = true;
            commStr_.data = "hello";
            // return;
        }
        if(dis < 0.3
            && robot1_find_me_ == true && commStr_.data == "waitForMe")
        {
            commStr_.data = "hello";
            too_far_ == false;
            far_to_near_ = true;
            // return;
        }
        //If the position between robot 1 and robot 2 is too large
        //a command will be issued for robot 1 to wait
        if(dis > 0.7
         && robot1_find_me_ == true && commStr_.data == "hello")
        {
            too_far_ = true;
            commStr_.data = "waitForMe";
        }
        if(dis < 0.2
            && robot1_find_me_ == true && commStr_.data == "hello" )
        {
            if(send_flag_ == true && cancel_flag_ == false)
            {
                ac_->cancelGoal();
                cancel_flag_ = true;
            }
            // std::cout<<"cancel_Flag = : "<<cancel_flag_<<"Send Flag = :"<<send_flag_<<"ROBOT STATUS : "<<robot1_find_me_<<std::endl;
            return;
        }


        if(commStr_.data != "hello" && commStr_.data != "waitForMe")
        {
            return;
        }
        tf::Quaternion q = transform.getRotation();
        double roll1, pitch1, yaw1;                    //Define a container for storing r\p\y
        tf::Matrix3x3(q).getRPY(roll1, pitch1, yaw1);
        double yaw_degree1 = yaw1 / 3.1415 * 180; 

        //robot2's navigation stack top coordinate system
        nextGoal.target_pose.header.frame_id = "robot2_tf/odom";
        nextGoal.target_pose.header.stamp = ros::Time::now();
        //Let robot 2 go around robot 1
        nextGoal.target_pose.pose.position.x = transform.getOrigin().getX() - 0.15*cos(yaw1);
        nextGoal.target_pose.pose.position.y = transform.getOrigin().getY();
        nextGoal.target_pose.pose.position.z = 0;
        nextGoal.target_pose.pose.orientation.w = 1;
        nextGoal.target_pose.pose.orientation.x = 0;
        nextGoal.target_pose.pose.orientation.y = 0;
        nextGoal.target_pose.pose.orientation.z = 1;

       
        ROS_INFO("Sending new Action!");
        ac_->sendGoal(nextGoal);
        send_flag_ = true;
        cancel_flag_ = false;
        // ac_->waitForResult(ros::Duration(30));

        if(ac_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Robot2 , Follow the Robot1!");
        }
        else
        {
            // ac_->cancelGoal();
            ROS_INFO("Doing");
        }
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s",ex.what());
        
    }
}

bool Follow::isRobot(int start, int end)
{
    if(image_show_ == true)
    {
        cv::namedWindow("窗口1",CV_WINDOW_NORMAL);
    }
    points_.clear();
    points1_.clear();
    std::vector<double> temp_chec;
    std::vector<double> temp_checx;
    std::vector<double> temp_checy;

    //So if you have a sequence of points between 0 and check_len_ and 360-check_len_ and 360
    if(start > end && start>=(360-check_len_) && end <= check_len_)
    {
        for(int i = start; i<360; i++)
        {
            //Read the distance and calculate the xy coordinates
            temp_chec.push_back(scanData_.ranges[i]);
            temp_checx.push_back(scanData_.ranges[i] * cos( i/180.0 * PI))  ;
            temp_checy.push_back(scanData_.ranges[i] * sin( i/180.0 * PI)) ;
            points_.push_back(cv::Point2f(temp_checx[temp_checx.size()-1], temp_checy[temp_checy.size()-1] ) );

            //Expand the coordinates of the points for easy display
            points1_.push_back(cv::Point2f( temp_checx[temp_checx.size()-1]*100 + 100, 
            temp_checy[temp_checy.size()-1]*100 +100 ));
            if(image_show_ == true)
            {
                cv::circle(image_, cv::Point2f(temp_checx[temp_checx.size()-1]*100 + 100, 
                temp_checy[temp_checy.size()-1]*100 +100), 1, cv::Scalar(0, 0, 255));
            }
            
        }   
        for(int i = 0; i<=end; i++)
        {
            temp_chec.push_back(scanData_.ranges[i]);
            temp_checx.push_back(scanData_.ranges[i] * cos( i/180.0 * PI))  ;
            temp_checy.push_back(scanData_.ranges[i] * sin( i/180.0 * PI)) ;
            points_.push_back(cv::Point2f(temp_checx[temp_checx.size()-1], temp_checy[temp_checy.size()-1] ) );
            
            points1_.push_back(cv::Point2f( temp_checx[temp_checx.size()-1]*100 + 100, 
            temp_checy[temp_checy.size()-1]*100 +100 ));
            if(image_show_ == true)
            {
                cv::circle(image_, cv::Point2f(temp_checx[temp_checx.size()-1]*100 + 100, 
                temp_checy[temp_checy.size()-1]*100 +100), 1, cv::Scalar(0, 0, 255));
            }
        }   
        //Calculated radius of curvature
        cv::Mat A;
        double R = CurvatureCalculation(temp_checx, temp_checy,A);

        //The ellipse fitting
        cv::RotatedRect RRect = cv::fitEllipse(points1_);
        // cv::ellipse(image_, RRect, cv::Scalar(255,0,0), 1, CV_AA);

        RRect = cv::fitEllipse(points_);

        //Determine if it is a robot
        if(points_.size()>10 && RRect.size.width>0.1 
        && abs(RRect.size.height/RRect.size.width )< 4
        && R > 0.2) 
        {
            std::cout<<"FIND    ^_^"<<std::endl;    
        }  
        std::cout<<std::endl;
        if(image_show_ == true)
        {
            cv::imshow("窗口1", image_);
            cv::waitKey(1);
        }
        return true;
    }

    if(start > end)
    {
        for(int i = end; i<=start; i++)
        {
            temp_chec.push_back(scanData_.ranges[i]);
            temp_checx.push_back(scanData_.ranges[i] * cos( i/180.0 * PI))  ;
            temp_checy.push_back(scanData_.ranges[i] * sin( i/180.0 * PI)) ;
            points_.push_back(cv::Point2f(temp_checx[temp_checx.size()-1], temp_checy[temp_checy.size()-1] ) );
            
            points1_.push_back(cv::Point2f( temp_checx[temp_checx.size()-1]*100 + 100, 
            temp_checy[temp_checy.size()-1]*100 +100 ));
            if(image_show_ == true)
            {
                cv::circle(image_, cv::Point2f(temp_checx[temp_checx.size()-1]*100 + 100, 
                temp_checy[temp_checy.size()-1]*100 +100), 1, cv::Scalar(0, 0, 255));
            }
        }

        cv::Mat A;
        double R = CurvatureCalculation(temp_checx, temp_checy,A);

        cv::RotatedRect RRect = cv::fitEllipse(points1_);
        // cv::ellipse(image_, RRect, cv::Scalar(255,0,0), 1, CV_AA);

        RRect = cv::fitEllipse(points_);

        if(points_.size()>10 && RRect.size.width>0.1 
        && abs(RRect.size.height/RRect.size.width )< 4
        && R > 0.2) 
        {
            std::cout<<"FIND    ^_^"<<std::endl;    
        }         
        std::cout<<std::endl;

        if(image_show_ == true)
        {
            cv::imshow("窗口1", image_);
            cv::waitKey(1);
        }
        return true;
    }
    if(start < end)
    {
        for(int i = start; i<=end; i++)
        {
            temp_chec.push_back(scanData_.ranges[i]);
            temp_checx.push_back(scanData_.ranges[i] * cos( i/180.0 * PI))  ;
            temp_checy.push_back(scanData_.ranges[i] * sin( i/180.0 * PI)) ;
            points_.push_back(cv::Point2f(temp_checx[temp_checx.size()-1], temp_checy[temp_checy.size()-1] ) );
            
            points1_.push_back(cv::Point2f( temp_checx[temp_checx.size()-1]*100 + 100, 
            temp_checy[temp_checy.size()-1]*100 +100 ));
            cv::circle(image_, cv::Point2f(temp_checx[temp_checx.size()-1]*100 + 100, 
            temp_checy[temp_checy.size()-1]*100 +100), 1, cv::Scalar(0, 0, 255));
        }

        cv::Mat A;
        double R = CurvatureCalculation(temp_checx, temp_checy, A);
        cv::RotatedRect RRect ;
        RRect = cv::fitEllipse(points1_);
        // cv::ellipse(image_, RRect, cv::Scalar(255,0,0), 1, CV_AA);
        
        RRect = cv::fitEllipse(points_);
        // std::cout<<"The REct width =  "<<RRect.size.width<<" The REct height = "<<RRect.size.height
        // <<"  BiLi = "<<RRect.size.height/RRect.size.width<<" Dis_MAX = "<<R<<std::endl; 

        if(points_.size()>10 && RRect.size.width>0.1 
        && abs(RRect.size.height/RRect.size.width )< 4
        && R > 0.2) 
        {
            std::cout<<"FIND    ^_^"<<std::endl;    
        }  
        std::cout<<std::endl;
        if(image_show_ == true)
        {
            cv::imshow("窗口1", image_);
            cv::waitKey(1);
        }
        return true;
    }


}

bool Follow::checkRobot()
{
    image_ =  cv::Mat(480, 640, CV_8UC3, cv::Scalar(255,255,255));
    int len_limit = check_len_;

    if(scan_ok_ == true)
    {
        int cout =0;

        //Take the data segment from the jump point
        for(int i=0; i<leap_index_.size(); i++)
        {
            if(i < leap_index_.size()-1)
            {
                if(abs(leap_index_[i]-leap_index_[i+1]) < len_limit  && abs(leap_index_[i]-leap_index_[i+1]) > 5)
                {
                    cout++;
                    // std::cout<<"possible: "<<leap_index_.size()<<" len :"<<(leap_index_[i]-leap_index_[i+1])<<"cout: "<<cout<<std::endl;

                    bool ch = this->isRobot(leap_index_[i], leap_index_[i+1]);
                }
            }

            if(i == leap_index_.size()-1)
            {
                if(abs(360-leap_index_[i] + leap_index_[0]) < len_limit && abs(360-leap_index_[i] + leap_index_[0]) > 5)
                {

                    // std::cout<<"possible&&&&&&&&&&"<<std::endl;
                    bool ch = this->isRobot(leap_index_[i], leap_index_[0]);
                }
            } 
        }
     
    }
    else
    {
        return false;
    }
}

void Follow::tfCalcAndPub()
{
    if(commStr_.data != "hello" && commStr_.data != "waitForMe" && tf_Mode_ != true)
    {
        return ;
    }
    try
    {
        if(tf_flag_ == false)
        {
        // Find the robot1 map->robot1_tf /base_footprint coordinate transformation
        listener_.waitForTransform("/map", "robot1_tf/odom", ros::Time(0), 
        ros::Duration(0.03));
        listener_.lookupTransform("/map", "robot1_tf/odom", ros::Time(0), 
        transform_robot1_);

        robot1_location_.x = transform_robot1_.getOrigin().getX();
        robot1_location_.y = transform_robot1_.getOrigin().getY();
        tf::Quaternion q = transform_robot1_.getRotation();
        double roll, pitch, yaw;                   
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        double yaw_degree = yaw / 3.1415 * 180; 
        robot1_location_.theta = yaw_degree;

        tf_flag_ = true;
        }

    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s",ex.what());
        
    }
}

double Follow::CurvatureCalculation(std::vector<double> &x, std::vector<double> &y, cv::Mat &A)
{
    std::vector<cv::Point2f> points;
    std::vector<double> distances;
    int len = x.size();
    if(len != y.size())
    {
        return 0;
    }
    for(int i = 0; i<len; i++)
    {
        points.push_back(cv::Point2f(x[i], y[i]));
    }

    // cv::Mat A;
    bool isOk = polynomial_curve_fit(points, 1, A);

    // std::cout<<A.at<double>(1, 0)<<std::endl;

    for(int i=0; i<points.size(); i++)
    {
        double x0 = points[i].x; double y0 = points[i].y;
        double dis = abs(y0 - A.at<double>(1, 0) * x0  - A.at<double>(0, 0)) / sqrt(A.at<double>(1, 0)*A.at<double>(1, 0)+1) ;
        distances.push_back(dis);
    }
    double max = *max_element(distances.begin(), distances.end());
    return max;

}
bool Follow::polynomial_curve_fit(std::vector<cv::Point2f>& key_point, int n, cv::Mat& A)
{
    //Number of key points
    int N = key_point.size();
 
    //构造矩阵X
    cv::Mat X = cv::Mat::zeros(n + 1, n + 1, CV_64FC1);
    for (int i = 0; i < n + 1; i++)
    {
        for (int j = 0; j < n + 1; j++)
        {
            for (int k = 0; k < N; k++)
            {
                X.at<double>(i, j) = X.at<double>(i, j) +
                    std::pow(key_point[k].x, i + j);
            }
        }
    }
 
    //构造矩阵Y
    cv::Mat Y = cv::Mat::zeros(n + 1, 1, CV_64FC1);
    for (int i = 0; i < n + 1; i++)
    {
        for (int k = 0; k < N; k++)
        {
            Y.at<double>(i, 0) = Y.at<double>(i, 0) +
                std::pow(key_point[k].x, i) * key_point[k].y;
        }
    }
 
    A = cv::Mat::zeros(n + 1, 1, CV_64FC1);
    //求解矩阵A
    cv::solve(X, Y, A, cv::DECOMP_LU);
    return true;
}
void Follow::circleLeastFit(const std::vector<cv::Point2f> points, cv::Point2f &center, double &radius)
{
    radius = 0.0f;
    double sum_x = 0.0f, sum_y = 0.0f;
    double sum_x2 = 0.0f, sum_y2 = 0.0f;
    double sum_x3 = 0.0f, sum_y3 = 0.0f;
    double sum_xy = 0.0f, sum_x1y2 = 0.0f, sum_x2y1 = 0.0f;
    int N = points.size();
    for (int i = 0; i < N; i++)
    {
        double x = points[i].x;
        double y = points[i].y;
        double x2 = x * x;
        double y2 = y * y;
        sum_x += x;
        sum_y += y;
        sum_x2 += x2;
        sum_y2 += y2;
        sum_x3 += x2 * x;
        sum_y3 += y2 * y;
        sum_xy += x * y;
        sum_x1y2 += x * y2;
        sum_x2y1 += x2 * y;
    }
    double C, D, E, G, H;
    double a, b, c;
    C = N * sum_x2 - sum_x * sum_x;
    D = N * sum_xy - sum_x * sum_y;
    E = N * sum_x3 + N * sum_x1y2 - (sum_x2 + sum_y2) * sum_x;
    G = N * sum_y2 - sum_y * sum_y;
    H = N * sum_x2y1 + N * sum_y3 - (sum_x2 + sum_y2) * sum_y;
    a = (H * D - E * G) / (C * G - D * D);
    b = (H * C - E * D) / (D * D - G * C);
    c = -(a * sum_x + b * sum_y + sum_x2 + sum_y2) / N;
    center.x = a / (-2);
    center.y = b / (-2);
    radius = sqrt(a * a + b * b - 4 * c) / 2;

}