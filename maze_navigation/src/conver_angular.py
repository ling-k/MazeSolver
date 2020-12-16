#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Twist


def callback_get(msg):
    myvel.linear.x = msg.linear.x
    myvel.angular.z = -msg.angular.z*1.0
    vel_pub.publish(myvel)

if __name__ == "__main__": 
    try:
        rospy.init_node('angular_deal',anonymous=False)
        myvel = Twist()
        vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        vel_sub = rospy.Subscriber('smoother_cmd_vel',Twist,callback_get,queue_size=1)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("cam_goal node terminated.")
