#include <nav_vel_plan/NavVelPlan.h>

NavVelPlan::NavVelPlan(costmap_2d::Costmap2DROS *costmap2d_ros, ros::NodeHandle *n)
    :costmap2d_ros_(costmap2d_ros)
{
    //Initializes the costMap2D's parameters
    // costmap2d_ros_->start();

    private_nh = n;
    costmap2d_ = costmap2d_ros_->getCostmap();
    layered_costmap_ = costmap2d_ros_->getLayeredCostmap();

    //Get the initial position of the robot
    bool isok = costmap2d_ros_->getRobotPose(initPose_);
    if(isok == true)
    {
        std::cout<<"Robot1's position get!"<<std::endl;
    }
    else
    {
        ROS_INFO("Failed to get robot1 location! Please check where goes wrong!");
    }
    unsigned int mx, my;
    mx=253; my=512-229;
    double w_x, w_y;
    costmap2d_->mapToWorld(mx,my,w_x, w_y);
    std::cout<<"THe Coordinate is : "<< "(" <<w_x<<","<<w_y<<")"<<std::endl;


    scan_ok_ = false;
    find_robot2_ = false;
    accomplish_ = false;

    send_flag_ = false;
    cancel_flag_ = false;

    commStr_.data = "start";
    I_find_robot2_.linear.x = 0;
    go_center_ = false;

    points_to_look_.push_back(cv::Point2f(0.45, -1.3));
    points_to_look_.push_back(cv::Point2f(0.45, -1.3));
    points_to_look_.push_back(cv::Point2f(0.45, -1.3));

    //Initialize the ROS parameters
    InitRosParameters();
    InitCvCostImg();

}

NavVelPlan::~NavVelPlan()
{
    // costmap2d_ros_->stop();
}

void NavVelPlan::setMoveBaseAction(MoveBaseClient *ac)
{
    ac_ = ac;
}

void cruise()
{

}
void  NavVelPlan::navToCenter()
{
    //Access to map Centre
   if(go_center_ == false)
    {
        move_base_msgs::MoveBaseGoal nextGoal1;
        nextGoal1.target_pose.header.frame_id = "map";
        nextGoal1.target_pose.header.stamp = ros::Time::now();
        nextGoal1.target_pose.pose.position.x = 0;
        nextGoal1.target_pose.pose.position.y = -0.5;
        nextGoal1.target_pose.pose.position.z = 0;
        nextGoal1.target_pose.pose.orientation.w = 1;
        nextGoal1.target_pose.pose.orientation.x = 0;
        nextGoal1.target_pose.pose.orientation.y = 0;
        nextGoal1.target_pose.pose.orientation.z = 0;


        ROS_INFO("Go Center!");
        ac_->sendGoal(nextGoal1 );
        ac_->waitForResult(ros::Duration(90));

        if(ac_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Done ");
            // ac_->cancelGoal();
        }
        else
        {
            ac_->cancelGoal();
        }
        go_center_ = true;
        
    }
}

void NavVelPlan::publishVel()
{
    navToCenter();
    if(go_center_ == true)
    {
        I_find_robot2_.linear.y = 666;
    }

    ros::spinOnce();
    find_pub_.publish(I_find_robot2_);

    //Get the real-time position and orientation of the robot
    unsigned int mx,my;
    bool isOk = costmap2d_ros_->getRobotPose(tmpPose_);//Gets the current robot world coordinates
    double wx = tmpPose_.pose.position.x ;
    double wy = tmpPose_.pose.position.y ;

    if(scan_ok_ == true )
    {
        // std::cout<<"ready"<<std::endl;
        if(commStr_.data == "hello" || commStr_.data == "waitForMe")
        {
            find_robot2_ = true;
            I_find_robot2_.linear.x = 666;
        }
    }

    if(find_robot2_ == true && scan_ok_ == true && accomplish_ == false)
    {
        nextGoal.target_pose.header.frame_id = "map";
        nextGoal.target_pose.header.stamp = ros::Time::now();

        //call move base to plan a long distance.

        nextGoal.target_pose.pose.position.x = initPose_.pose.position.x;
        nextGoal.target_pose.pose.position.y = initPose_.pose.position.y + 0.4;
        nextGoal.target_pose.pose.position.z = 0;
        nextGoal.target_pose.pose.orientation.w = 1;
        nextGoal.target_pose.pose.orientation.x = 0;
        nextGoal.target_pose.pose.orientation.y = 0;
        nextGoal.target_pose.pose.orientation.z = 1;

        if(send_flag_ == false && commStr_.data == "hello")
        {
            ROS_INFO("Sending new Action!");
            ac_->sendGoal(nextGoal 
            ) ;
            send_flag_ = true;
        }

        //If robot 2 requests a stop, the navigation is cancelled
        if(commStr_.data == "waitForMe" && send_flag_ == true )
        {
            ac_->cancelGoal();
            send_flag_ = false;
        }

        if(ac_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Hooray, the base moved a point forward in full path!");
            accomplish_ = true;
            ac_->cancelGoal();
            I_find_robot2_.linear.z = 666;
        }

    }


}

void NavVelPlan::InitRosParameters()
{
    //Declare subscribers and search parameters
    vel_pub_ = private_nh->advertise<geometry_msgs::Twist>("robot1/cmd_vel", 1);
    find_pub_ = private_nh->advertise<geometry_msgs::Twist>("robot1/find_robot2", 1);

    scan_sub_ = private_nh->subscribe("/robot1/scan", 1,&NavVelPlan::scanCallback, this );
    comm_sub_ = private_nh->subscribe("/comm", 1,&NavVelPlan::commCallback, this);

    std::string robotRadius,planInterval, stepForward ,robotLength, robotWidth; 
    std::string useRect, parallelismTolerance;

    robot_radius_ = 0.5;
    if(private_nh->searchParam("robot_radius",robotRadius))
        private_nh->param("robot_radius",robot_radius_,0.5);
    plan_interval_ = 0.3;
    if(private_nh->searchParam("plan_interval",planInterval))
        private_nh->param("plan_interval",plan_interval_,0.3);
    step_forward_ = 0.2;
    if(private_nh->searchParam("step_forward",stepForward))
        private_nh->param("step_forward",step_forward_,0.2);
    robot_length_ = 0.8;
    if(private_nh->searchParam("robot_length",robotLength))
        private_nh->param("robot_length",robot_length_,0.8);
    robot_width_ = 0.6;
    if(private_nh->searchParam("robot_width",robotWidth))
        private_nh->param("robot_width", robot_width_, 0.6);
    use_rect_ = false;
    if(private_nh->searchParam("use_rect",useRect))
        private_nh->param("use_rect", use_rect_, false);

    parallelism_tolerance_ = 10.0;
    if(private_nh->searchParam("parallelism_tolerance",parallelismTolerance))
        private_nh->param("parallelism_tolerance", parallelism_tolerance_, 10.0);

    resolution_ = costmap2d_->getResolution();
    std::cout<<"The resolution of map is "<<resolution_<<std::endl;

}

void NavVelPlan::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{   
    scan_ok_ = true;
    //Zero to 360 degrees, 360 sample points, positive counterclockwise, in radians
    std::cout<<"get scan!!!"<<std::endl;
    scanData_.header = msg->header;
    scanData_.angle_min = msg->angle_min; scanData_.angle_max = msg->angle_max;
    scanData_.angle_increment = msg->angle_increment; scanData_.time_increment = msg->time_increment;
    scanData_.scan_time = msg->scan_time; 
    scanData_.range_min = msg->range_min; scanData_.range_max = msg->range_max;
    scanData_.ranges = msg->ranges; scanData_.intensities = msg->intensities;

}

void NavVelPlan::commCallback(const std_msgs::String::ConstPtr& msg)
{
    commStr_.data = msg->data;
}

void NavVelPlan::InitCvCostImg()
{

    int sizex = costmap2d_->getSizeInCellsX();
    //Accessor for the x size of the costmap in cells.
    int sizey = costmap2d_->getSizeInCellsY();
    std::cout<<"The size of map is "<<sizex<<"  "<<sizey<<std::endl;

    srcCVCostMap_ = cv::Mat(sizey,sizex,CV_8U);
    for(int r = 0; r < sizey; r++){
      for(int c = 0; c < sizex; c++ ){
          unsigned char cost_value = costmap2d_->getCost(c, sizey-1-r);
          srcCVCostMap_.at<uchar>(r,c) = cost_value;
          //??sizey-r-1 caution: costmap's origin is at left bottom ,while opencv's pic's origin is at left-top.
      }
    }

}

void NavVelPlan::updateAndShowCostmap()
{
    layered_costmap_ = costmap2d_ros_->getLayeredCostmap();
    costmap2d_ = layered_costmap_->getCostmap();

    int sizex = costmap2d_->getSizeInCellsX();
    //Accessor for the x size of the costmap in cells.
    int sizey = costmap2d_->getSizeInCellsY();
    std::cout<<"The size of map is "<<sizex<<"  "<<sizey<<std::endl;

    srcCVCostMap_ = cv::Mat(sizey,sizex,CV_8U);
    for(int r = 0; r < sizey; r++){
      for(int c = 0; c < sizex; c++ ){
          unsigned char cost_value = costmap2d_->getCost(c, sizey-1-r);
          srcCVCostMap_.at<uchar>(r,c) = cost_value;
          //??sizey-r-1 caution: costmap's origin is at left bottom ,while opencv's pic's origin is at left-top.
      }
    }
  
}

std::vector<pointIndex> NavVelPlan::CvImageToCostMap(std::vector<pointIndex> &cvpoints)
{
    int sizex = costmap2d_->getSizeInCellsX();  //The width of the costmap
    int sizey = costmap2d_->getSizeInCellsY();  //The height of the costmap
    std::vector<pointIndex> map_points;
    for(int i = 0; i<cvpoints.size(); i++)
    {
        pointIndex po;
        po.col = cvpoints[i].col;   po.row = sizey-1-cvpoints[i].row;   po.theta = cvpoints[i].theta;
        //The origin of Opencv is in the upper left corner and the origin of CostMap is in the lower left corner
        map_points.push_back(po);
    }
    return map_points;
}
pointIndex NavVelPlan::CvImageToCostMap(pointIndex &cvpoint)
{
    int sizex = costmap2d_->getSizeInCellsX();  //The width of the costmap
    int sizey = costmap2d_->getSizeInCellsY();  //The height of the costmap
 
    pointIndex po;
    po.col = cvpoint.col;   po.row = sizey-1-cvpoint.row;   po.theta = cvpoint.theta;
    return po;    
}

float NavVelPlan::vectorDot(const cv::Point2i pt1, const cv::Point2i pt2)
{
    return (float)(pt1.x * pt2.x + pt1.y * pt2.y);
}
float NavVelPlan::pointLineDistance(const cv::Point &pt1, const cv::Point &pt2)
{
    float a = sqrt( (pt1.x - pt2.x)*(pt1.x - pt2.x) + (pt1.y - pt2.y)*(pt1.y - pt2.y) );
    return a;
}

void NavVelPlan::doneCb( const actionlib::SimpleClientGoalState& state,
        const move_base_msgs::MoveBaseActionResultConstPtr& result)
{
    ROS_INFO("Yay! Achieve the init point");
}
void NavVelPlan::activeCb()
{
    ROS_INFO("Goal just went active");
}
// The callback function called after receiving a feedback
void NavVelPlan::feedbackCb(const move_base_msgs::MoveBaseActionFeedbackConstPtr& feedback)
{
    ROS_INFO(" Status : %d ", feedback->status.status);
}