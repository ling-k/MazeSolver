#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/LaserScan.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt8.h>

#include <vector>
#include <iostream>
#include <algorithm>

class Navigation_translator
{
    public:
    ros::NodeHandle nh;
    ros::Subscriber targetSub;
    ros::Subscriber statusSub;
    ros::Publisher pubPose = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 100);
    bool at_home = true;

    Navigation_translator () {
        targetSub = nh.subscribe<geometry_msgs::Point>("/target", 5, &Navigation_translator::target_callback, this);
        statusSub = nh.subscribe<actionlib_msgs::GoalStatusArray>("/move_base/status", 5, &Navigation_translator::status_callback, this);
    }

    void target_callback (const geometry_msgs::Point::ConstPtr& msg) {
        std::cout << " Target received..." << std::endl;
        geometry_msgs::PoseStamped target;
        target.header.frame_id = "map";
        target.pose.position.x = msg->x;
        target.pose.position.y = msg->y;
        target.pose.orientation.z = 1.0;
        target.pose.orientation.w = 0.0;
        pubPose.publish(target);
        at_home = false;
    }

    void status_callback (const actionlib_msgs::GoalStatusArray::ConstPtr& msg) {
        if (msg->status_list.size() == 0 ) { return; }
        int current_status = msg->status_list[0].status;
        
        if (current_status == 1) // robot is still moving
        {
            return;
        }
        else if (current_status == 3){ //robot reached the goal
            geometry_msgs::PoseStamped home;
            home.header.frame_id = "map";
            home.pose.orientation.z = 1.0;
            home.pose.orientation.w = 0.0;
            ros::Duration(5.0).sleep();
            std::cout << " Moving back to home position (0,0)..." << std::endl;
            pubPose.publish(home);

        }
        else { //navigation failed
            return;
        }
    }
};
int main (int argc, char** argv) {
    ros::init(argc, argv, "nav_translator");
    Navigation_translator navi_trans;
    ros::spin();

    return 0;
}