#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/LinearMath/Matrix3x3.h>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <string>
#include <ros/time.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>

class Monitor
{
public:
	/**
	* @brief  Constructor for the actions
	* @param name The name of the action
	* @param n A pointer to a NodeHandle
	*/
	Monitor(ros::NodeHandle *n);

	/**
	* @brief  Performs a control cycle
	*/
	void mainloop();

	/**
	* @brief  Get the position of robot 1
	*/
	void getRobot1Pose();

	/**
	* @brief  Monitor robot 1 status
	*/
	void robot1VelMonitor();
	virtual ~Monitor();

private:
	ros::NodeHandle *_nh;
	tf::TransformListener _listener;

    ros::Subscriber _subOdom;
    ros::Subscriber _subScan2;
    ros::Subscriber _subScan1; 
    // ros::Subscriber _subVel2;
    // ros::Subscriber _subVel1;
    ros::Subscriber _findRobot2Sub;
    // ros::Subscriber _pathSub1;


	ros::Publisher _robot1VelPub;

	sensor_msgs::LaserScan _scanData_1;
	sensor_msgs::LaserScan _scanData_2;
	geometry_msgs::Twist _robot1;
	geometry_msgs::Twist _robot2;
	nav_msgs::Path _robot1_path;

	geometry_msgs::Twist _pub_robot1;
	geometry_msgs::Twist _pub_robot2;

	ros::Time _start;
	ros::Time _end;
	ros::Duration _duration_time;
	tf::StampedTransform _transform_robot1;
	geometry_msgs::Point _robot1TempPose;
	geometry_msgs::Point _robot1OldPose;

	bool _center_flag;
	bool _find_robot2_flag;
	bool _repair_flag;

private:
	double PI = 3.14159;

	int front_len_, left_len_, right_len_, rear_len_;
	std::vector<double> sample_front_, sample_left_, sample_right_, sample_rear_;
	double front_min_, left_min_, right_min_, rear_min_;

private:
	double pointDistance(const geometry_msgs::Point& p1, const geometry_msgs::Point &p2);

	void path1Cb(const nav_msgs::Path::ConstPtr& msg);
	void findCb(const geometry_msgs::Twist::ConstPtr& msg);
	void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) ;
	void robot2_scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) ;
	void robot1_scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) ;
	void robot1_velCallback(const geometry_msgs::Twist::ConstPtr& msg) ;
	void robot2_velCallback(const geometry_msgs::Twist::ConstPtr& msg) ;

};