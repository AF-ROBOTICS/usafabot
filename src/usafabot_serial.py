#!/usr/bin/env python3
import rospy
import serial.tools.list_ports as port_list
import serial
import sys
# TODO
# Remove Twist and add custom message
from geometry_msgs.msg import Twist
from usafabot.msg import WheelVelocity

## USAFABOT
#
#  Connects to the MSP432 over serial and writes linear/angular velocity
#   to enable the USAFABOT to drive accordingly. The robot operates well around
#   with a linear velocity between 0.05 m/s and 0.25 m/s and an angular velocity of 
#   0.5 m/s and 1.5 m/s.
class USAFABOT:
    def __init__(self):
        # class variable to store the velocity of the wheels
        self.wv = WheelVelocity()
        # read wheel speed from robot at 10 Hz
        rospy.Timer(rospy.Duration(.1), self.callback_read)
        
        # Initialize a serial connection w/ the MSP432 microcontroller
        self.ser = self.find_usafabot()
     
        # publisher to send wheel speeds over the wheel_speeds topic
        self.pub = rospy.Publisher('wheel_speeds', WheelVelocity, queue_size=100)
     
        # TODO
        # change topic and message to match new custom message per instructions
        rospy.Subscriber('cmd_vel', Twist, self.callback_write)        

    # Subscriber function that gets linear/angular velocity
    # values from controller and writes to MSP432
    def callback_write(self, data):
        try:
            # TODO
            # update message field names for data to match custom message
            input = str(data.linear.x) + ',' + str(data.angular.z) + '\r\n'
            self.ser.write(input.encode())
            print(input)
        except serial.SerialException:
            print("Serial port not open")
        
    # Function to get wheel speeds from MSP432 and publish over wheel_speeds topic
    def callback_read(self, event):
        try:
            velocities = self.ser.readline()
            self.wv.v_r, self.wv.v_l = [float(i) for i in velocities.decode().strip().split(',')]
            if(-.05 < self.wv.v_r and self.wv.v_r < .05): self.wv.v_r = 0.0
            if(-.05 < self.wv.v_l and self.wv.v_l < .05): self.wv.v_l = 0.0
            self.pub.publish(self.wv)
        except serial.SerialException:
            print("Serial port not open")
        except:
            print("Invalid data:")
            
    # Helper function to connect to the USAFABOT. Checks each serial connection to Pi
    # until it finds the correct one (MSP432 is running code to respond to a request)
    def find_usafabot(self):
        test = "start\r".encode('ascii')
        ports = list(port_list.comports())
        for Port, Desc, Hwid in sorted(ports):
            print(Port)
            if(Desc[0:6] == 'XDS110'):
                ser = serial.Serial(port = Port,
                    baudrate = 115200,
                    parity = serial.PARITY_NONE,
                    stopbits = serial.STOPBITS_ONE,
                    bytesize = serial.EIGHTBITS,
                    timeout = 1)
                ser.write(test)
                response = ser.readline()
                print(response)
                if(response.decode('ascii') == "start\n"):
                    print("Connected to USAFABOT at", Port)   
                    return ser
                else:
                    ser.write(test)
                    response= ser.readline()
                    print(response)
                    if(response.decode('ascii') == "start\n"):
                        print("Connected to USAFABOT at", Port)   
                        return ser
                    
        print("USAFABOT not found...exiting")
        sys.exit()
        
    def handler(self):
        while not rospy.is_shutdown():
            pass
        input = "0.0,0.0\r\n"
        self.ser.write(input.encode())
        rospy.sleep(.5)
        input = "quit\r\n"
        self.ser.write(input.encode('ascii'))
        response = self.ser.readline()
        while(len(response) > 0):
            print(response)
            response=self.ser.readline()
        self.ser.close()
        print("shutting down")
    
if __name__ == '__main__':
    rospy.init_node('usafabot_serial', anonymous = True)
    
    # list available ports for that the MSP432 is using
    U = USAFABOT()
    U.handler()
