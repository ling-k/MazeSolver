#include <ros/ros.h>

#include <string>
#include <geometry_msgs/Twist.h>
/**
 * Subscriber callbacks
 */
geometry_msgs::Twist robot1;
geometry_msgs::Twist robot2;

void robot1_velCallback(const geometry_msgs::Twist::ConstPtr& msg) 
{
    robot1.linear = msg->linear;
    robot1.angular = msg->angular;
}
void robot2_velCallback(const geometry_msgs::Twist::ConstPtr& msg) 
{
    robot2.linear = msg->linear;
    robot2.angular = msg->angular;
}

int main(int argc, char** argv) {
    // Node initialization
    ros::init(argc, argv, "topic_conventer");
    ros::NodeHandle n;

    // Topic subscribers
    ros::Subscriber subVel2    = n.subscribe("robot2/cmd_vel", 1, robot2_velCallback);
    ros::Subscriber subVel1    = n.subscribe("robot1/cmd_vel", 1, robot1_velCallback);
    ros::Publisher  vel_pub_1 = n.advertise<geometry_msgs::Twist>("/conv/robot1/cmd_vel", 1);
    ros::Publisher  vel_pub_2 = n.advertise<geometry_msgs::Twist>("/conv/robot2/cmd_vel", 1);

    ros::Rate r(10);
    while(ros::ok())
    {
        ros::spinOnce();
        vel_pub_1.publish(robot1);
        vel_pub_2.publish(robot2);
        r.sleep();
    }

    return 0;
}