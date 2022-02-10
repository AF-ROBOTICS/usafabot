#!/usr/bin/env python3
import rospy
import math
from squaternion import Quaternion # requires squaternion and attrs library

from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry


# Position
#
#  Subscribes to USAFABOT_Bot's current position (from Roadrunner) and destination
#   position (from master node) to determine linear and angular velocity and
#   publishes those values to the TI_Bot
#
#  Subscriber
#   Topic: ti_curr_pos
#     Msg type: Pose
#     Freq: 100 Hz
#   Topic: ti_dest_pos
#     Msg type: Point
#     Freq: 100 Hz
#
#  Publisher
#   Topic: cmd_vel
#     Msg type: Twist
#     Freq: 100 Hz
class Position:

    def __init__(self):
        self.ctrl_c = False
        
        rospy.on_shutdown(self.shutdownhook)

        rospy.Subscriber('odom', Odometry, self.callback_CurrPos)
        self.pub = rospy.Publisher('curr_pos', Pose, queue_size=1)

    # Subscribe function that gets current position of
    # the TI Bot from Gazebo and transforms to Euler
    # Publishes new position over ti_curr_pos topic
    # Topic:
    #   Subscriber: odom
    #   Publisher: ti_cur_pos
    # Msg type:
    #   Subscriber: Odometry
    #   Publisher: Pose
    def callback_CurrPos(self, data):
        if not self.ctrl_c:
            pose = Pose()
            pose.position = data.pose.pose.position
            pose.orientation = data.pose.pose.orientation

            # math magic
            q = Quaternion(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z)
            #e = q.to_euler(degrees=True)
            #q = Quaternion.from_euler(e[0], e[1], e[2]-90, degrees=True)
            pose.orientation.x = q.x
            pose.orientation.y = q.y
            pose.orientation.z = q.z
            pose.orientation.w = q.w

            self.pub.publish(pose)
            
    def shutdownhook(self):
        self.ctrl_c = True
        rospy.loginfo("Shutting down pos_transform.")

if __name__ == '__main__':
    rospy.init_node('position', anonymous=True)

    Position()
    rospy.spin()
