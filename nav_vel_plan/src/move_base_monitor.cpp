
#include "nav_vel_plan/Monitor.h"

int main(int argc, char** argv) {
    // Node initialization
    ros::init(argc, argv, "move_base_monitor");
    ros::NodeHandle n;

    //Initializes the instantiation object for Monitor
    Monitor mo(&n);


    ros::Rate r(20);
    while(ros::ok())
    {
        mo.mainloop();
        r.sleep();
    }

    return 0;
}