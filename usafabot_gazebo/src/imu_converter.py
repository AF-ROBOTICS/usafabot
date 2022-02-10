#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped

from squaternion import Quaternion # requires squaternion and attrs library

class Imu_Convert:

    def __init__(self):
        self.ctrl_c = False
        
        rospy.on_shutdown(self.shutdownhook)

        rospy.Subscriber('imu', Imu, self.callback_imu)
        self.pub = rospy.Publisher('imu/rpy', Vector3Stamped, queue_size=1)

    def callback_imu(self, imu):
        if not self.ctrl_c:
            
            new_imu = Vector3Stamped()
            new_imu.header.frame_id = "odom"
            new_imu.header.stamp = rospy.Time.now()

            # math magic
            q = Quaternion(imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z)
            e = q.to_euler()

            new_imu.vector.x = e[0]
            new_imu.vector.y = e[1]
            new_imu.vector.z = e[2]
            
            self.pub.publish(new_imu)
            
    def shutdownhook(self):
        self.ctrl_c = True
        rospy.loginfo("Shutting down imu_converter.")

if __name__ == '__main__':
    rospy.init_node('imu_converter')

    Imu_Convert()
    rospy.spin()
