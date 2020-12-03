#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

class TranslateJoy 
{
    public:
    ros::NodeHandle nh;
    ros::Subscriber joySub;
    ros::Publisher velPub = nh.advertise<geometry_msgs::Twist> ("cmd_vel", 100);

    TranslateJoy() {
        joySub = nh.subscribe("/joy", 100, &TranslateJoy::joy2vel, this );
    }

    void joy2vel (const sensor_msgs::Joy::ConstPtr& joyMsg) {
        geometry_msgs::Twist newVel;
        newVel.linear.x = 0.2 * (joyMsg->axes[1]);
        newVel.angular.z = 1.0 * (joyMsg->axes[0]);
        velPub.publish(newVel);
    }
};

int main (int argc, char** argv) {
    ros::init(argc, argv, "translator");
    TranslateJoy trans;
    ros::spin();

    return 0;
}