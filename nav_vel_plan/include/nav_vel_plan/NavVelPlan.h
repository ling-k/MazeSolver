#ifndef NavVelPlan_U
#define NavVelPlan_U

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <string>
#include <algorithm>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/layered_costmap.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Path.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <move_base_msgs/MoveBaseActionFeedback.h>
#include <actionlib_msgs/GoalStatus.h>
#include<tf2_ros/buffer.h>
#include<tf2_ros/transform_listener.h>

//typedefs to help us out with the action server so that we don't hace to type so much
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
struct pointIndex
{
    int row;
    int col;
    double theta; 
};

class NavVelPlan
{
public:
	/**
	* @brief  Constructor for the actions
	* @param name The name of the action
	* @param costmap2d_ros A pointer to Costmap2DROS
	* @param n A pointer to a NodeHandle
	*/
	NavVelPlan(costmap_2d::Costmap2DROS *costmap2d_ros, ros::NodeHandle *n);

	/**
	* @brief  Initialize the ros parameters
	*/
	void InitRosParameters();

	/**
	* @brief  Initialize the cost map
	*/
	void InitCvCostImg();	

	/**
	* @brief  Update cost map
	*/
	void updateAndShowCostmap();

	/**
	* @brief  Publish navigation points and monitor robot 2 status
	*/
	void publishVel();	

	/**
	* @brief  Setup move_base client
	*/
	void setMoveBaseAction(MoveBaseClient *ac);
	virtual ~NavVelPlan();

public:
	bool accomplish_;

protected:

	/**
	* @brief  Coordinate transformation from CV to cost map
	* @param cvpoints A vector containing a point expressed in Opencv coordinates
	* @return Points in costMap format
	*/
	std::vector<pointIndex> CvImageToCostMap(std::vector<pointIndex> &cvpoints);
	/**
	* @brief  An overloaded function that converts the coordinates of a point
	*/
	pointIndex CvImageToCostMap(pointIndex &cvpoint);

	//Distance between two points
	float pointLineDistance(const cv::Point &pt1, const cv::Point &pt2);

	//The dot product of two vectors
	float vectorDot(const cv::Point2i pt1, const cv::Point2i pt2);

	//Go to map center
	void navToCenter();

private:
	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
	void commCallback(const std_msgs::String::ConstPtr& msg);

	void cruise();

	void doneCb(const actionlib::SimpleClientGoalState& state,
        const move_base_msgs::MoveBaseActionResultConstPtr& result);
	void activeCb();
	void feedbackCb(const move_base_msgs::MoveBaseActionFeedbackConstPtr& feedback);

private:
	ros::NodeHandle *private_nh;

	costmap_2d::Costmap2DROS* costmap2d_ros_;
    costmap_2d::Costmap2D* costmap2d_;
	costmap_2d::LayeredCostmap* layered_costmap_;

	geometry_msgs::PoseStamped initPose_;	//Robot1 initial position
	geometry_msgs::PoseStamped tmpPose_;		//Robot1 real-time location

	ros::Publisher vel_pub_;
	ros::Publisher find_pub_;

	ros::Subscriber scan_sub_;
	ros::Subscriber comm_sub_;

	bool scan_ok_;
	bool go_center_;
	bool find_robot2_;
	bool send_flag_;
	bool cancel_flag_;
	sensor_msgs::LaserScan scanData_;
	geometry_msgs::Twist vel_;
	std_msgs::String commStr_;
	geometry_msgs::Twist I_find_robot2_;

	
	/***************** Robot and path planning parameters *******************/
	int map_rotation_angle_;
	bool use_rect_;
	double resolution_;
	double robot_radius_;
	double plan_interval_;
	double step_forward_;
	double robot_length_;
	double robot_width_;
	double parallelism_tolerance_; //in degree

	std::vector<cv::Point2f> points_to_look_;

	cv::Mat srcCVCostMap_;	//The cost map

	MoveBaseClient *ac_;
  	move_base_msgs::MoveBaseGoal nextGoal;

};

#endif