#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include <nav_vel_plan/NavVelPlan.h>

int main(int argc, char** argv)
{
  // Node initialization
  ros::init(argc, argv, "robot1_seracher");
  ros::NodeHandle nh("~/vel_plan_nodehandle");

  //Action client, subscribe to move_base action
  MoveBaseClient ac("move_base", true);

  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  //Declare Buffer and TF listeners
  tf2_ros::Buffer  buffer(ros::Duration(10));
  tf2_ros::TransformListener tf(buffer);
  
  //Costmap2DROS object initialization
  costmap_2d::Costmap2DROS lcr("vel_costmap", buffer);

  //It is used for the work of robot 1's search robot 2
  NavVelPlan *velPlanner = new NavVelPlan(&lcr, &nh);

  //Set the move_base client to the object
  velPlanner->setMoveBaseAction(&ac);

  ros::Rate r(10);
  while(ros::ok())
  {
    //Publish navigation points and monitor status
    velPlanner->publishVel();
    r.sleep();
  }

  delete velPlanner;
  
  return 0;
}
