#!/usr/bin/env python3
import rospy
import math
from math import sin, cos, pi

# Because of transformations
import tf_conversions
from squaternion import Quaternion

import tf2_ros
import tf2_msgs.msg
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from usafabot.msg import WheelVelocity

class broadcaster:
    def __init__(self):
        self.br = tf2_ros.TransformBroadcaster()
        self.robot = TransformStamped()
        
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size = 50)
        
        # position
        self.x = 0
        self.y = 0
        # orientation
        self.th = 0
        
        # values used to determine linear and angular velocity
        self.v_l = 0
        self.v_r = 0
        
        # time
        self.last_time = rospy.Time.now()
        
        rospy.Subscriber('wheel_speeds', WheelVelocity, self.callback_wheel_vel)
        #rospy.Subscriber('imu/rpy', Vector3Stamped, self.callback_imu)
        
        rospy.Timer(rospy.Duration(.01), self.callback_odom)
        
    # get the left and right wheel speeds from robot
    def callback_wheel_vel(self, vel):
        self.v_l = vel.v_l
        self.v_r = vel.v_r
        
    # get orientation from IMU
    # TODO: try getting this value from the odometry instead...
    #       if we want to use the robot localization EKF 
    #       then it doesn't make sense to get value from the same sensor.
    #       If we get it from odom, then put entire callback_odom into
    #       callback_wheelVel.
    def callback_imu(self, rpy):
        self.th = rpy.vector.z
        

    def callback_odom(self, event):
    
        curr_time = rospy.Time.now()

        v_th = 6.7*(self.v_r - self.v_l) 

        dt = (curr_time - self.last_time).to_sec()

        # TODO: try putting back in and using 
        delta_th = v_th * dt
        self.th += delta_th

    
        v_x = .5*(self.v_l + self.v_r)*cos(self.th) # m/s
        v_y = .5*(self.v_l + self.v_r)*sin(self.th) # m/s

        print("v_x: ", v_x)
        print("v_y: ", v_y)

        
        
        delta_x = (v_x*cos(self.th) - v_y*sin(self.th)) * dt
        delta_y = (v_x*sin(self.th) + v_y*cos(self.th)) * dt
        
        
        self.x += delta_x
        self.y += delta_y

        # robot transform in world (point directly center of two wheels)
        self.robot.header.stamp = curr_time
        self.robot.header.frame_id = "odom"
        self.robot.child_frame_id = "base_footprint"
        self.robot.transform.translation.x = self.x
        self.robot.transform.translation.y = self.y
        self.robot.transform.translation.z = 0.0
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, self.th)
        self.robot.transform.rotation.x = q[0]
        self.robot.transform.rotation.y = q[1]
        self.robot.transform.rotation.z = q[2]
        self.robot.transform.rotation.w = q[3]
        self.br.sendTransform(self.robot)
        
        # odometry message for the robot
        odom = Odometry()
        
        odom.header.stamp = curr_time
        odom.header.frame_id = "odom"
        
        odom.pose.pose = (Pose(Point(self.x, self.y, 0.), Quaternion(*q)))
        
        # set the velocity
        odom.child_frame_id = "base_footprint"
        odom.twist.twist = Twist(Vector3(v_x, v_y, 0), Vector3(0, 0, v_th))
        
        self.odom_pub.publish(odom)
        
        self.last_time = curr_time

if __name__ == '__main__':
    rospy.init_node('odometry_publisher')
    broadcaster()
    rospy.spin()
