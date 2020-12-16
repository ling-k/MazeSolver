/*********************************************************************
*
*For detecting and following robot 1, it is used in conjunction with robot 2's navigation stack
*
*********************************************************************/

#ifndef Follow_H
#define Follow_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/LinearMath/Matrix3x3.h>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <vector>
#include <string>
#include <algorithm>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

//typedefs to help us out with the action server so that we don't hace to type so much
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

struct Position
{
    double x;
    double y;
    double theta; 
};

class Follow
{
public:
	/**
	* @brief  Constructor for the actions
	* @param name The name of the action
	* @param tf A reference to a NodeHandle
	*/
	Follow(ros::NodeHandle &n);

	/**
	* @brief  Destructor - Cleans up
	*/
	virtual ~Follow();

	/**
	* @brief  Performs a control cycle
	*/
	void mainloop();

	/**
	* @brief  Initializes the necessary parameters
	*/
	void InitParameters();
	void InitRosParameters();

	/**
	* @brief  Calculate the TF transformation derived from the Robot2 speedometer
	*/
	void tfCalcAndPub();

	/**
	* @brief  The main loop of following Robot1
	*/
	void startFollow();

	/**
	* @brief  Pass in the move_base action client
	*/
	void setMoveBase(MoveBaseClient *ac);

private:
	/**
	* @brief  A callback function for a lidar message
	* @param msg A constant reference to the received LaserScan-type message
	*/
	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
	
	/**
	* @brief  Using lidar data to fit the radius of curvature circle detection robot
	* @return True If the robot is detected
	*/
	bool checkRobot(); //It calls isRobot
	bool isRobot(int start, int end); 

	/**
	* @brief  It fits the curve of any order according to the scatter point and coefficient matrix
	* @param x The x-coordinate vector of the scatter
	* @param y The y-coordinate vector of the scatter
	* @param A The resulting coefficient matrix
	* @return The radius of curvature of a point
	*/
	double CurvatureCalculation(std::vector<double> &x, std::vector<double> &y, cv::Mat &A);

	/**
	* @brief  It uses a scatter to solve the coefficient matrix
	* @param key_point The vector of the scatter
	* @param A The resulting coefficient matrix
	*/
	bool polynomial_curve_fit(std::vector<cv::Point2f>& key_point, int n, cv::Mat& A);

	/**
	* @brief  It uses two-dimensional scatterings to fit the circle
	* @param points two-dimensional scatterings
	* @param center The coordinates of the center of the circle returned
	* @param center The coordinates of the radius of the circle returned	
	*/
	void circleLeastFit(const std::vector<cv::Point2f> points, cv::Point2f &center, double &radius);

private:
	MoveBaseClient *ac_;
	move_base_msgs::MoveBaseGoal nextGoal;
	bool too_far_;
	bool far_to_near_;
	bool robot1_find_me_;
	bool tf_Mode_;
	bool image_show_;

private:
	ros::NodeHandle private_nh_;
	bool tf_flag_;
	bool send_flag_;
	bool cancel_flag_;
	std::string map_frame_id;
	std::string odom_frame_id;

	tf::TransformListener listener_;
	tf::TransformBroadcaster br_;
	tf::Transform transform_;

	tf::StampedTransform transform_robot1_;
	tf::StampedTransform transform_robot2_;

	Position robot1_location_;
	Position robot2_location_;

	double laser_x_;
	double laser_y_;

	sensor_msgs::LaserScan scanData_;
	geometry_msgs::Twist vel_;
	std_msgs::String commStr_;

	ros::Publisher vel_pub_;
	ros::Publisher comm_pub_;
	ros::Subscriber scan_sub_;
	// ros::Subscriber comm_sub_;

	//Environmental parameters scanned by lidar
	double PI = 3.14159;
	bool scan_ok_;
	double robot_dis_;
	int check_len_, front_len_, left_len_, right_len_, rear_len_;
	std::vector<double> sample_front_, sample_left_, sample_right_, sample_rear_;
	double front_min_, left_min_, right_min_, rear_min_;

	//Use when checking the robot
    std::vector<int> leap_index_;	
    std::vector<cv::Point2f> points_;	
    std::vector<cv::Point2f> points1_;	
    cv::Mat image_ ;	

};

#endif