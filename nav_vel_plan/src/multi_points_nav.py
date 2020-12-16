#!/usr/bin/env python
import rospy
import tf
import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf_conversions import transformations
from math import pi
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
import math 


class navigation_demo:
    def __init__(self):
        self.find_flag_ = False
        self.center_flag_ = False
        self.cancel_flag_ = False
        self.rate = rospy.Rate(10)
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(60))
        self.find_sub = rospy.Subscriber('robot1/find_robot2', Twist, self.find_callback)

    def find_callback(self, msg):
        if(abs(msg.linear.x-666.0)<= 0.01):
            self.find_flag_ = True
        if(abs(msg.linear.y-666.0)<= 0.01):
            self.center_flag_ = True

    def _done_cb(self, status, result):
        pass
        rospy.loginfo("navigation done! status:%d result:%s"%(status, result))

    def _active_cb(self):
        pass
        # rospy.loginfo("[Navi] navigation has be actived")

    def _feedback_cb(self, feedback):
        if(self.find_flag_ == True and self.cancel_flag_ == False):
            self.move_base.cancel_goal()
            self.cancel_flag_ = True
        # pass
        rospy.loginfo("[Navi] navigation feedback\r\n%s"%feedback)
        # print("Status is The ****$$$",feedback.status.status)

    def goto(self, p):
        while(self.center_flag_ == False):
            self.rate.sleep()
        if(self.find_flag_ == True):
            return True
        
        #Fill the ROS navigation points
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = p[0]
        goal.target_pose.pose.position.y = p[1]
        q = transformations.quaternion_from_euler(0.0, 0.0, p[2]/180.0*pi)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        #Publish the navigation points and set the callback function
        self.move_base.send_goal(goal, self._done_cb, self._active_cb, self._feedback_cb)
        
        result = self.move_base.wait_for_result(rospy.Duration(120))

        if not result:
            self.move_base.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
        else:
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("reach goal %s succeeded!"%p)

        return True

    def cancel(self):
        self.move_base.cancel_all_goals()
        return True


if __name__ == "__main__":
    rospy.init_node('navigation_points_pub',anonymous=True)

    #Object initialization
    navi = navigation_demo()

    #Set the cruise point coordinates
    goal_points = [[0.45,-1.3,0],[-0.77,-1.3,0],[-1.3, -1.1,0]]

    #Publish navigation points in a loop
    for goal in goal_points:
        navi.goto(goal)


