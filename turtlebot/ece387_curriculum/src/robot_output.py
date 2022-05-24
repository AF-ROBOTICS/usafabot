#!/usr/bin/env python3
import rospy
import turtlebot3_msgs
from ece387_curriculum.msg import RobotOutput
from turtlebot3_msgs.msg import Sound

# basic class to receive output messages and send to TurtleBot3
class Output:

    def __init__(self):
        self.ctrl_c = False

        rospy.Subscriber('robot_output', RobotOutput, self.callback_beep)
        self.pub = rospy.Publisher('sound', Sound, queue_size = 1)
        rospy.on_shutdown(self.shutdownhook)

    def callback_beep(self, r_o):
        print(r_o.message)
        if r_o.state == RobotOutput.START:
            self.pub.publish(r_o.state)

        elif r_o.state == RobotOutput.END:
            self.pub.publish(r_o.state)

        elif r_o.state == RobotOutput.STOP:
            self.pub.publish(r_o.state)

        elif r_o.state == RobotOutput.TURN:
            self.pub.publish(r_o.state)

    def shutdownhook(self):
        print("Robot output exiting.")

if __name__ == '__main__':
    rospy.init_node('robot_output')
    Output()
    rospy.spin()
