#!/usr/bin/env python3
import rospy
import serial.tools.list_ports as port_list
import serial
import sys
from geometry_msgs.msg import Twist
from usafabot.msg import WheelVelocity

## TiBot
#
#  Connects to the MSP 432 over serial and writes linear/angular velocity
#   to enable the TI_Bot to drive towards the target destination.
#
#  Subscriber
#   Topic: cmd_vel
#     Msg type: Twist
#     Freq: 100 Hz
class TiBot:
    def __init__(self):
        self.wv = WheelVelocity()
        rospy.Timer(rospy.Duration(.1), self.callback_read)
        
        # Initialize a serial w/ the MSP432 microcontroller
        self.ser = self.find_ti()
     
        self.pub = rospy.Publisher('wheel_speeds', WheelVelocity, queue_size=100)
     
        rospy.Subscriber('cmd_vel', Twist, self.callback_write)
        

    # Subscriber function that gets linear/angular velocity
    # values from controller and writes to MSP_432
    # Topic: Cmd_vel
    # Msg type: Twist
    def callback_write(self, data):
        try:
            input = str(data.linear.x) + ',' + str(data.angular.z) + '\r\n'
            self.ser.write(input.encode())
            print(input)
        except serial.SerialException:
            print("Serial port not open")
        
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
            

    def find_ti(self):
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
                    print("Connected to TI-RSLK Max at", Port)   
                    return ser
                else:
                    ser.write(test)
                    response= ser.readline()
                    print(response)
                    if(response.decode('ascii') == "start\n"):
                        print("Connected to TI-RSLK Max at", Port)   
                        return ser
                    
        print("TI Bot not found...exiting")
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
    rospy.init_node('ti_bot', anonymous = True)
    
    # list available ports for that the MSP432 is using
    T = TiBot()
    T.handler()
