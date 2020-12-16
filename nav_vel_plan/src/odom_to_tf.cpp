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

#define RAD2DEG 57.295779513
/**
 * Subscriber callbacks
 */

 std::string odom_frame_id = "odom2";
 std::string base_frame_id = "base_link2";
 sensor_msgs::LaserScan scanData_;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) 
{
	static tf::TransformBroadcaster br;
    

	tf::Transform transform;
    double tx = msg->pose.pose.position.x;
    double ty = msg->pose.pose.position.y;
    double tz = msg->pose.pose.position.z;
	transform.setOrigin(tf::Vector3(tx,ty,tz));

    // Orientation quaternion
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);

    // 3x3 Rotation matrix from quaternion
    tf2::Matrix3x3 m(q);

    // Roll Pitch and Yaw from rotation matrix
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

	tf::Quaternion q1;
    q1.setRPY(0,0,yaw);
    transform.setRotation(q1);
	
    br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),odom_frame_id,base_frame_id));

    // Output the measure
    ROS_INFO("Received odom in '%s' frame : X: %.2f Y: %.2f Z: %.2f - R: %.2f P: %.2f Y: %.2f",
             msg->header.frame_id.c_str(),
             tx, ty, tz,
             roll * RAD2DEG, pitch * RAD2DEG, yaw * RAD2DEG);
}
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) 
{
    scanData_.header.stamp = ros::Time::now();
    scanData_.header.frame_id = "laser2";
    scanData_.angle_min = msg->angle_min; scanData_.angle_max = msg->angle_max;
    scanData_.angle_increment = msg->angle_increment; scanData_.time_increment = msg->time_increment;
    scanData_.scan_time = msg->scan_time; 
    scanData_.range_min = msg->range_min; scanData_.range_max = msg->range_max;
    scanData_.ranges = msg->ranges; scanData_.intensities = msg->intensities;

}



int main(int argc, char** argv) {
    // Node initialization
    ros::init(argc, argv, "odom_to_tf");
    ros::NodeHandle n;

    // Topic subscribers and publishers
    ros::Subscriber subOdom    = n.subscribe("robot2/odom", 1, odomCallback);
    ros::Subscriber subScan    = n.subscribe("robot2/scan", 1, scanCallback);
    ros::Publisher  scan_pub = n.advertise<sensor_msgs::LaserScan>("/robot2/laser2", 1);
    n.param<std::string>("my_odom_frame_id", odom_frame_id, "/odom2");
    n.param<std::string>("my_base_frame_id", base_frame_id, "/base_link2");

    tf::TransformListener listener_;
    tf::TransformBroadcaster br_;

    ros::Rate r(20);
    while(ros::ok())
    {
        ros::spinOnce();

        scan_pub.publish(scanData_);

        try
        {
            tf::StampedTransform transform_robot1_;
            tf::Transform transform_;

            // Find the coordinate transformation of robot2_tf/base_footprint->robot2_tf/base_scan
            listener_.waitForTransform("robot2_tf/base_footprint", "robot2_tf/base_scan", ros::Time(0), 
            ros::Duration(0.03));
            listener_.lookupTransform("robot2_tf/base_footprint", "robot2_tf/base_scan", ros::Time(0), 
            transform_robot1_);

            tf::Quaternion q = transform_robot1_.getRotation();

            double tx = transform_robot1_.getOrigin().getX() ;
            double ty = transform_robot1_.getOrigin().getY() ;
            double tz = transform_robot1_.getOrigin().getZ();
            transform_.setOrigin(tf::Vector3(tx,ty,tz));//Set translation transformation

            transform_.setRotation(q);//Set the Angle transformation

            br_.sendTransform(tf::StampedTransform(transform_,ros::Time::now(),
            "base_link2","laser2"));
        
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
            
        }
        r.sleep();
    }

    return 0;
}