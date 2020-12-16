#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_vel_plan/Follow.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot2_self");
  ros::NodeHandle nh("~/");

  MoveBaseClient ac("move_base_robot2", true);
  Follow *robot2 = new Follow(nh);

  robot2->setMoveBase(&ac);

  ros::Rate r(10);
  while(ros::ok())
  {
    //Start detecting and following robot 1
    robot2->mainloop();
    r.sleep();
  }

  delete robot2;
  
  return 0;
}
