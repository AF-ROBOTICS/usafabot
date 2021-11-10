#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose

class Goal:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        
        self.goal = MoveBaseGoal()
        rospy.Subscriber('target_avg', Pose, self.callback_goal)
        self.found_goal = False
        
    def callback_goal(self, data):
        x = data.position.x
        y = data.position.y
        print("goalX:", x)
        print("goalY:", y)
        if(not (x == 0 and y == 0 and not self.found_goal)):
            self.goal.target_pose.header.frame_id = "map"
            self.goal.target_pose.header.stamp = rospy.Time.now()
            self.goal.target_pose.pose.position.x = x
            self.goal.target_pose.pose.position.y = y
            self.goal.target_pose.pose.orientation.w = 1.0
            
            self.client.send_goal(self.goal)
            wait = self.client.wait_for_result()
            if not wait:
                rospy.logerr("Action server not available!")
                rospy.signal_shutdown("Action server not available!")
            else:
                print("Found goal")
                self.found_goal = True
                
       

if __name__ == '__main__':
    rospy.init_node('movebase_client_py')
    Goal()
    rospy.spin()
