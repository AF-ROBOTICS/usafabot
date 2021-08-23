#!/usr/bin/env python3
import rospy, logging
import socket, sys, time

from cflib import crazyflie, crtp
from cflib.crazyflie.log import LogConfig
from geometry_msgs.msg import Pose, Vector3, TransformStamped, Twist
from nav_msgs.msg import Odometry

import tf2_ros
import tf2_msgs.msg

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

## Roadrunner
#
#  Connects to a Bitcraze Roadrunner with given URI and publishes
#   current x, y, and z position and yaw orientation.
#
#  Publisher
#   Topic: ti_curr_pos
#     Msg type: Pose
#     Freq: 100 Hz
class Roadrunner:
    PERIOD = 10 # Control period. [ms]

    def __init__(self, URI, name):
        # instance variables unique to each instance of class
        self.rr_pose = Pose()
        self.rr_vel = Twist()
        self.rr = crazyflie.Crazyflie(rw_cache='./cache')

        self.name = name
        
        # Connect some callbacks from the Crazyflie API
        self.rr.connected.add_callback(self._connected)
        self.rr.disconnected.add_callback(self._disconnected)
        self.rr.connection_failed.add_callback(self._connection_failed)
        self.rr.connection_lost.add_callback(self._connection_lost)
        
        print('Connecting to', URI)    
        self.rr.open_link(URI)

        self.br = tf2_ros.TransformBroadcaster()

        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size = 50)
        self.pos_pub = rospy.Publisher("curr_pos", Pose, queue_size = 50)

        self.ctrl_c = False

        rospy.on_shutdown(self.shutdownhook)

        rospy.Timer(rospy.Duration(.01), self.robot_state)

    #-------------------------------------------------------------------------
    # CrazyFlie Callbacks and Functions
    #-------------------------------------------------------------------------
    def _connected(self, link_uri):
        print('Connected to', link_uri)
        
        print('Waiting for position estimate to be good enough...')
        self.reset_estimator()
        
        log_orient = LogConfig(name='Kalman Orientation', period_in_ms=self.PERIOD)
        log_orient.add_variable('kalman.q0', 'float')
        log_orient.add_variable('kalman.q1', 'float')
        log_orient.add_variable('kalman.q2', 'float')
        log_orient.add_variable('kalman.q3', 'float')
        
        log_pos = LogConfig(name='Kalman Position', period_in_ms=self.PERIOD)
        log_pos.add_variable('kalman.stateX', 'float')
        log_pos.add_variable('kalman.stateY', 'float')
        log_pos.add_variable('kalman.stateZ', 'float')
        
        log_vel = LogConfig(name='Kalman Velocity', period_in_ms=self.PERIOD)
        log_vel.add_variable('kalman.statePX', 'float')
        log_vel.add_variable('kalman.statePY', 'float')
        log_vel.add_variable('kalman.statePZ', 'float')
        log_vel.add_variable('gyro.z', 'float')

        try:
            self.rr.log.add_config(log_orient)
            log_orient.data_received_cb.add_callback(self.curr_yaw)
            log_orient.start()
            
            self.rr.log.add_config(log_pos)
            log_pos.data_received_cb.add_callback(self.curr_position)
            log_pos.start()

            self.rr.log.add_config(log_vel)
            log_vel.data_received_cb.add_callback(self.curr_velocity)
            log_vel.start()

        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add Stabilizer log config, bad configuration.')

    def _connection_failed(self, link_uri, msg):
        print('Connection to %s failed: %s' % (link_uri, msg))

    def _connection_lost(self, link_uri, msg):
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        print('Disconnected from %s' % link_uri)

    def _log_error(self, logconf, msg):
        print('Error when logging %s: %s' % (logconf.name, msg))

    def reset_estimator(self):
        self.rr.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        self.rr.param.set_value('kalman.resetEstimation', '0')
        # Sleep a bit, hoping that the estimator will have converged
        # Should be replaced by something that actually checks...
        time.sleep(1.5)
        
    # Callback function that reads position from Bitcraze Roadrunner
    # Frequency: 100 Hz
    def curr_position(self, timestamp, data, logconf):
        self.rr_pose.position = Vector3(data['kalman.stateX'], data['kalman.stateY'], data['kalman.stateZ'])

    # Callback function that reads orientation from Bitcraze Roadrunner
    # Frequency: 100 Hz
    def curr_yaw(self, timestamp, data, logconf):
        self.rr_pose.orientation.x = data['kalman.q1']
        self.rr_pose.orientation.y = data['kalman.q2']
        self.rr_pose.orientation.z = data['kalman.q3']
        self.rr_pose.orientation.w = data['kalman.q0']

    def curr_velocity(self, timestamp, data, logconf):
        self.rr_vel.linear = Vector3(data['kalman.statePX'], data['kalman.statePY'], data['kalman.statePZ'])
        self.rr_vel.angular = Vector3(0,0,data['gyro.z'])

    # Handler that publishes the x, y, and z position and
    # yaw orientation values from the Bitcraze Roadrunner
    # to the Controller node
    # Topic: ti_curr_pos
    # Msg type: Pose
    # Frequency: 100 Hz        
    def robot_state(self, event):
        if not self.ctrl_c:
            curr_time = rospy.Time.now()

            odom_trans = TransformStamped()
            odom_trans.header.stamp = curr_time
            odom_trans.header.frame_id = self.name+"_odom"
            odom_trans.child_frame_id = self.name+"_roadrunner"
            odom_trans.transform.translation = self.rr_pose.position
            odom_trans.transform.rotation = self.rr_pose.orientation
            self.br.sendTransform(odom_trans)

            odom = Odometry()
            odom.header.stamp = curr_time
            odom.header.frame_id = self.name+"_odom"
            odom.child_frame_id = self.name+"_roadrunner"

            odom.pose.pose = self.rr_pose
            odom.twist.twist = self.rr_vel
            self.odom_pub.publish(odom)
            self.pos_pub.publish(self.rr_pose)

    def shutdownhook(self):
        self.ctrl_c = True
        rospy.loginfo("Shutting down roadrunner.")
        self.rr.close_link()

#-------------------------------------------------------------------------------
# main()
#------------------------------------------------------------------------------- 
if __name__ == "__main__":
    rospy.init_node('roadrunner')
    
    crtp.init_drivers(enable_debug_driver=False, enable_serial_driver=True)

    URI_1 = 'serial://ttyAMA0'
    hostname = socket.gethostname()

    Roadrunner(URI_1, hostname)
    rospy.spin()
