#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

#include <vector>
#include <iostream>
#include <algorithm>

class Explorer 
{
    public:
    ros::NodeHandle nh;
    ros::Subscriber Sub2calmin;
    ros::Publisher PubVel = nh.advertise<geometry_msgs::Twist>("cmd_vel", 100);

    std::vector<double> range_front, range_left, range_right;
    double min_front;
    double min_left;
    double min_right;

    // multiplier to adjust the linear and angular speed
    double multi_rot = 0.35;
    double multi = 0.35;

    Explorer () {
        Sub2calmin = nh.subscribe<sensor_msgs::LaserScan>( "/scan", 10, &Explorer::calculate_mins, this );
        cmd_velocity();
    }

    void calculate_mins (const sensor_msgs::LaserScan::ConstPtr& laserMsg ) {
        // calculating front
        int front_i = 0.0873 / laserMsg->angle_increment;
        range_front = std::vector<double>( laserMsg->ranges.end() - front_i, laserMsg->ranges.end() );
        range_front.insert( range_front.end(), laserMsg->ranges.begin(), laserMsg->ranges.begin() + front_i  );
        min_front = *std::min_element(range_front.begin(), range_front.end());

        // calculating right
        int right_i = 5.236 / laserMsg->angle_increment;
        int right_i_inc = 0.786 / laserMsg->angle_increment;
        range_right = std::vector<double>( laserMsg->ranges.begin() + right_i, laserMsg->ranges.begin() + right_i + right_i_inc );
        min_right = *std::min_element(range_right.begin(), range_right.end());

        // calculating left
        int left_i = 0.262 / laserMsg->angle_increment;
        int left_i_inc = 1.047 / laserMsg->angle_increment;
        range_left = std::vector<double>( laserMsg->ranges.begin() + left_i, laserMsg->ranges.begin() + left_i + left_i_inc );
        min_left = *std::min_element(range_left.begin(), range_left.end());
        std::cout << "Calculate_mins completed!, [front right left]= " << min_front << " " << min_right << " " << min_left << ". " << std::endl;
    }

    void cmd_velocity () {
        geometry_msgs::Twist cmd;
        cmd.linear.x = 0.0 * multi;
        cmd.angular.z = 0.0 * multi_rot;

        ros::Rate rate(5);
        
        bool near_wall = false; // initial wall status
        ros::Duration(1.0).sleep();

        std::cout << "Exploration begin, turning right at the beginning..." << std::endl;
        cmd.linear.x = 0.1 * multi;
        cmd.angular.z = -0.5 * multi_rot;
        PubVel.publish(cmd);
        ros::Duration(2.0).sleep();

        while (ros::ok()) {
            
            while (ros::ok() && near_wall == false) {
                if ( min_front > 0.2 && min_left > 0.2 && min_right > 0.2 ) {
                    cmd.linear.x = 0.15 * multi;
                    cmd.angular.z = -0.1 * multi_rot;
                }
                else if ( min_left < 0.2 ) {
                    near_wall = true;
                    std::cout << "Wall of the left detected, range: " << min_left <<". " << std::endl;
                }
                else {
                    cmd.linear.x = 0.0 * multi;
                    cmd.angular.z = -0.25 * multi_rot;
                }
                PubVel.publish(cmd);
            }

            if (min_front > 0.2) {
                if (min_left < 0.12) {
                    std::cout << "Too close to left wall, backing out a little..." << std::endl;
                    cmd.linear.x = -0.1 * multi;
                    cmd.angular.z = -1.2 * multi_rot;
                }
                else if (min_left > 0.15) {
                    std::cout << "Turning left to follow wall..." << std::endl;
                    cmd.linear.x = 0.15 * multi;
                    cmd.angular.z = 1.2 * multi_rot;
                }
                else {
                    std::cout << "Turning right to follow wall..." << std::endl;
                    cmd.linear.x = 0.15 * multi;
                    cmd.angular.z = 1.2 * multi_rot;
                }
            }
            else {
                std::cout << "Obstacle at the front detected, range: " << min_front << "m, steering away to avoid..." << std::endl;
                cmd.linear.x = 0.0 * multi;
                cmd.angular.z = -1.0 * multi_rot;
            }
            PubVel.publish(cmd);
            std::cout << " one cycle is done..." << std::endl;
            Sub2calmin = nh.subscribe<sensor_msgs::LaserScan>( "/scan", 10, &Explorer::calculate_mins, this );
            rate.sleep();
        }
    }
};

int main (int argc, char** argv ) {
    ros::init(argc, argv, "Maze auto explorer");
    Explorer exp;
    ros::spin();

    return 0;
}