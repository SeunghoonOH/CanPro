#!/usr/bin/env python

import rospy
#from can_msgs.msg import Frame
from geometry_msgs.msg import Twist
import struct
import numpy as np
import serial
from ctypes import *
import math 


def float16_to_hex(f):
    return hex(struct.unpack('<H', np.float16(f).tobytes())[0])


def hex_to_float16(hex):
    a = struct.pack("i", int(hex,16))
    return np.frombuffer(a, dtype= np.float16)[0]  


class CanNode():
    def __init__(self, port, bandrate):
     
        self.port = serial.Serial(port, bandrate)
        self.vel_sub = rospy.Subscriber("cmd_vel", Twist, self.VelCallback)
   #     self.cmd_pub = rospy.publisher('received_messages', Frame, queue_size=10)
   #     self.echo_sub = rospy.Subscriber("sent_messages", Frame, Echocallback)
       
    def VelCallback(self, data):

        twist = Twist()
        twist = data

        vel = twist.linear.x 
        delta =twist.angular.z


        self.makeVelcmd(vel, delta)
 

    def makeVelcmd(self, vel, delta):
         
        Canvel = vel * 1000
        Candelta = delta * 100

        Canvel = np.int(Canvel + 1000)
        Candelta = np.int(Candelta + 100)
        
        if(Canvel > 2000):
            Canvel = 2000

        if(Canvel < 0):
            Canvel = 0

        if(Candelta > 200):
            Candelta = 200
        
        if(Candelta < 0):
            Candelta = 0

        """
      # float to Hex 
        Canvel = np.float16(-2.23)

        CanHex = float16_to_hex(Canvel)
        print((CanHex))
    
        print(CanHex.encode())

        print(hex_to_float16(CanHex))
        """

        can_id = 0x140

        data = bytearray(8)
        data[0] = 0x02
        data[1] = 0x03
        data[2] = 0x00
        data[3] = 0x00
        data[4] = Canvel >> 8 & 0xFF
        data[5] = Canvel & 0xFF
        data[6] = Candelta >> 8 & 0xFF
        data[7] = Candelta & 0xFF
 
        self.makecmd(can_id, data)

    def makecmd(self, can_id, data):
        
        CAN_ID = can_id
        buff=bytearray(18)
        UART_CMD_DATA = 0x30
        send_len = 0
        sum = 0

        buff[send_len] = 0x02;
        send_len +=1

        buff[send_len] = 1;   #data length
        send_len +=1

        buff[send_len] = UART_CMD_DATA;   # CMD_DATA
     #   print(buff)

        sum += buff[send_len]
        send_len +=1

        buff[send_len] = CAN_ID >> 8  & 0xFF  # CAN ID

    #    print(buff[send_len])

        sum += buff[send_len]
        send_len +=1

        buff[send_len] = CAN_ID & 0xFF   # CAN ID

    #    print(buff[send_len])

        sum += buff[send_len]
        send_len +=1

        buff[send_len] = 0x00   # dummy

        sum += buff[send_len]
        send_len +=1

        buff[send_len] = 0

        sum += buff[send_len]
        send_len +=1

        buff[send_len] = 8  #data_len

        #buff[send_len] = 0x02

        sum += buff[send_len]
        send_len +=1

        for i in range(8):
            buff[send_len] = data[i]
            sum += buff[send_len]
            send_len +=1

        """
        buff[send_len] = 0x02  #1
        sum += buff[send_len]
        send_len +=1

        buff[send_len] = 0x01 #2
        sum += buff[send_len]
        send_len +=1

        buff[send_len] = 0x00 #3
        sum += buff[send_len]
        send_len +=1

        buff[send_len] = 0x00 #4
        sum += buff[send_len]
        send_len +=1

        buff[send_len] = 0x00 #5
        sum += buff[send_len]
        send_len +=1

        buff[send_len] = 0x00 #6
        sum += buff[send_len]
        send_len +=1

        buff[send_len] = 0x01 ##7
        sum += buff[send_len]
        send_len +=1

        buff[send_len] = 0x01 #8
        sum += buff[send_len]
        send_len +=1
        """

        buff[send_len] = sum
        send_len +=1

    #    print(chr(sum))

    #    print(buff)
    
        buff[send_len] = 0x03  # EOF
        send_len +=1

        if( send_len > 4):
            buff[1] = send_len -4

        print(send_len)

        self.sender(buff)

    def sender(self, buff):    

        self.port.write(buff)

    def Echocallback(self, Frame_data):
        """
       id = Frame_data.id
       data = Frame_data.data
       
       velary = []
       velary.append(data[4])
       velary.append(data[5])

       deltarary = []
       deltarary.append(data[6])
       deltarary.append(data[7])

       vel, = struct.unpack("<h", velary.encode())
       delta, = struct.unpack("<h", deltarary.encode())

       ROS_INFO("data[0] =%X , data[1] =%X., data[2] =%X, data[3] =%X ,data[4] =%X, data[5] =%X, data[6] =%X, data[7] =%X", data[0], data[1], data[2], data[3], vel, delta)
        """
      

    def main(self):
        rospy.spin()



   
if __name__ == '__main__':
    rospy.init_node("CanNode", anonymous=True)
    node = CanNode('/dev/ttyUSB0', 500000)
    node.main()      
    

