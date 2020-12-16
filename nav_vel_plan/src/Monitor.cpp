#include "nav_vel_plan/Monitor.h"

Monitor::Monitor(ros::NodeHandle *n)
:_nh(n)
{

    _subOdom    = _nh->subscribe("robot2/odom", 1, &Monitor::odomCallback,this);
    _subScan2    = _nh->subscribe("robot2/scan", 1, &Monitor::robot2_scanCallback,this);
    _subScan1    = _nh->subscribe("robot1/scan", 1, &Monitor::robot1_scanCallback,this);
    // _pathSub1 = _nh->subscribe("/move_base/GlobalPlanner/plan", 1, &Monitor::path1Cb,this);
    //  _subVel2    = _nh->subscribe("/conv/robot2/cmd_vel", 1, &Monitor::robot2_velCallback,this);
    // _subVel1    = _nh->subscribe("/conv/robot1/cmd_vel", 1, &Monitor::robot1_velCallback,this);
    _findRobot2Sub = _nh->subscribe("robot1/find_robot2", 1, &Monitor::findCb,this);

    _robot1VelPub = _nh->advertise<geometry_msgs::Twist>("robot1/cmd_vel", 1);

    front_len_ = 4; left_len_ = 4; right_len_ =4; rear_len_ = 4;

	_center_flag = false;
	_find_robot2_flag = false;
    _repair_flag = false;

    _start = ros::Time::now();
    _end = ros::Time::now();
    _duration_time = _end - _start;

    //Initializes the position of robot 1
    getRobot1Pose();
    _robot1OldPose = _robot1TempPose;
}

Monitor::~Monitor()
{
}

void Monitor::mainloop()
{
    getRobot1Pose();
    ros::spinOnce();

    //If robot1 completes the task, stop monitoring
    if(_find_robot2_flag == false)
    {
        robot1VelMonitor();
    }
}

void Monitor::robot1VelMonitor()
{
    _duration_time = _end - _start;
    std::cout<<"Time Duration : "<<_duration_time.toSec()<<" RePair: "<<_repair_flag<<std::endl;
    //Reset the timer every 22 seconds
    if(  _duration_time.toSec() > 22)
    {
        _start = ros::Time::now();
        _end = ros::Time::now();
        _robot1OldPose = _robot1TempPose;
        _repair_flag = false;
        return;
    }
    else if(_duration_time.toSec() >= 15 &&  _duration_time.toSec() <= 22)
    {
        double dis = pointDistance(_robot1OldPose, _robot1TempPose);
        if(abs(dis) < 0.1)
        {
            _repair_flag = true;
        }
        std::cout<<"DIS = :" <<dis<<std::endl;
        _end = ros::Time::now();
    }
    else
    {
        double dis = pointDistance(_robot1OldPose, _robot1TempPose);
        if(abs(dis) > 0.1)
        {
            _repair_flag = false;
            _start = ros::Time::now();
            _robot1OldPose = _robot1TempPose;
        }
        _end = ros::Time::now();
    }
    _duration_time = _end - _start;

    //A 5 second repair begins when the flag bit is true
    if(_duration_time.toSec() >= 15 &&  _duration_time.toSec() <= 22 && _repair_flag == true)
    {
        std::cout<<"Start to repair ^_^"<<std::endl;
        if(rear_min_ >= 0.20 && (right_min_<left_min_)  )
        {
            std::cout<<"Mode is 1"<<std::endl;
            _pub_robot1.linear.x = -0.05;
            _pub_robot1.angular.z = -0.8;
        }
        else if(rear_min_ >= 0.20 && (right_min_>left_min_)  )
        {   std::cout<<"Mode is 2"<<std::endl;
            _pub_robot1.linear.x = -0.05;
            _pub_robot1.angular.z = 0.8;
        }
        else
        {   std::cout<<"Mode is 5"<<std::endl;
            _pub_robot1.linear.x = 0;
            _pub_robot1.angular.z = 0.8;
        }

        if(front_min_ >=0.2 && (right_min_<left_min_))
        {   std::cout<<"Mode is 3"<<std::endl;
            _pub_robot1.linear.x = 0.05;
            _pub_robot1.angular.z = 0.8;
        }
        else if(front_min_ >=0.2 && (right_min_<left_min_))
        {   std::cout<<"Mode is 4"<<std::endl;
            _pub_robot1.linear.x = 0.05;
            _pub_robot1.angular.z = -0.8;
        }
        else
        {   std::cout<<"Mode is 5"<<std::endl;
            _pub_robot1.linear.x = 0;
            _pub_robot1.angular.z = 0.8;
        }
        _robot1VelPub.publish(_pub_robot1);
        // _pub_robot1
    }
    else
    {
        return;
    }
}

void Monitor::getRobot1Pose()
{
    try
    {
        
        _listener.waitForTransform("map", "robot1_tf/base_footprint", ros::Time(0), 
        ros::Duration(0.2));
        _listener.lookupTransform("map", "robot1_tf/base_footprint", ros::Time(0), 
        _transform_robot1);

        _robot1TempPose.x = _transform_robot1.getOrigin().getX() ;
        _robot1TempPose.y = _transform_robot1.getOrigin().getY() ;
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s",ex.what());
    }
}

double Monitor::pointDistance(const geometry_msgs::Point& p1, const geometry_msgs::Point &p2)
{
    return sqrt( pow((p1.x-p2.x),2) + pow((p1.y-p2.y), 2));
}

void Monitor::path1Cb(const nav_msgs::Path::ConstPtr& msg)
{
    _robot1_path.poses = msg->poses;
}
void Monitor::findCb(const geometry_msgs::Twist::ConstPtr& msg)
{
    //A sign that a task has been completed
    if(abs(msg->linear.z - 666.0)<0.01)
    {
        _find_robot2_flag = true;
    }
    if(abs(msg->linear.y-666.0)<0.01)
    {
        _center_flag = true;
    }
}
void Monitor::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) 
{
    double tx = msg->pose.pose.position.x;
    double ty = msg->pose.pose.position.y;
    double tz = msg->pose.pose.position.z;
}
void Monitor::robot2_scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) 
{
    _scanData_2.header.stamp = ros::Time::now();
    _scanData_2.header.frame_id = "robot2";
    _scanData_2.angle_min = msg->angle_min; _scanData_2.angle_max = msg->angle_max;
    _scanData_2.angle_increment = msg->angle_increment; _scanData_2.time_increment = msg->time_increment;
    _scanData_2.scan_time = msg->scan_time; 
    _scanData_2.range_min = msg->range_min; _scanData_2.range_max = msg->range_max;
    _scanData_2.ranges = msg->ranges; _scanData_2.intensities = msg->intensities;

}
void Monitor::robot1_scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) 
{
    _scanData_1.header.stamp = ros::Time::now();
    _scanData_1.header.frame_id = "robot1";
    _scanData_1.angle_min = msg->angle_min; _scanData_1.angle_max = msg->angle_max;
    _scanData_1.angle_increment = msg->angle_increment; _scanData_1.time_increment = msg->time_increment;
    _scanData_1.scan_time = msg->scan_time; 
    _scanData_1.range_min = msg->range_min; _scanData_1.range_max = msg->range_max;
    _scanData_1.ranges = msg->ranges; _scanData_1.intensities = msg->intensities;


    std::vector<double> temp_f(front_len_);
    std::vector<double> temp_l(left_len_);
    std::vector<double> temp_r(right_len_);
    std::vector<double> temp_rear(rear_len_);
    
    for(int i = 0; i<360; i++)
    {
        //The lidar data is used to calculate the minimum distance in the front,rear,left and right directions
        if(i < front_len_/2 || i >= 360 - front_len_/2)
        {
            if(i < front_len_/2)
            {
                temp_f[i] = _scanData_1.ranges[i];
            }
            else
            {
                temp_f[front_len_ -(360-i)] = _scanData_1.ranges[i];
            }
        }

        
        if(i < (90 + left_len_/2) && i >= (90 - left_len_/2))
        {
            temp_l[(i - 90) + left_len_/2] = _scanData_1.ranges[i];
        }

        
        if( i < (180 + rear_len_/2) && i >= (180 - rear_len_/2))
        {
            temp_rear[ (i -180) + rear_len_/2] = _scanData_1.ranges[i];
        }

        
        if( i < (270 + right_len_/2) && i >= (270 - right_len_/2))
        {
            temp_r[ (i - 270) + right_len_/2 ] = _scanData_1.ranges[i];
        }

    
    }

    sample_front_ = temp_f;
    sample_left_ = temp_l; sample_right_= temp_r; sample_rear_ = temp_rear;

    front_min_ =  *min_element(sample_front_.begin(), sample_front_.end());
    left_min_ = *min_element(sample_left_.begin(), sample_left_.end());
    right_min_ = *min_element(sample_right_.begin(), sample_right_.end());
    rear_min_ = *min_element(sample_rear_.begin(), sample_rear_.end());

}


void Monitor::robot1_velCallback(const geometry_msgs::Twist::ConstPtr& msg) 
{
    _robot1.linear = msg->linear;
    _robot1.angular = msg->angular;
}
void Monitor::robot2_velCallback(const geometry_msgs::Twist::ConstPtr& msg) 
{
    _robot2.linear = msg->linear;
    _robot2.angular = msg->angular;
}