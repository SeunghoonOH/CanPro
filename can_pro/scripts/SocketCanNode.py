#!/usr/bin/env python

import rospy
from can_msgs.msg import Frame
from geometry_msgs.msg import Twist
import struct
import numpy as np


class CanNode():
    def __init__(self):
        self.cmd_pub = rospy.Publisher('sent_messages', Frame, queue_size=10)
        self.motorenable()
        self.vel_sub = rospy.Subscriber('cmd_vel', Twist, self.VelCallback)
       
    #    self.echo_sub = rospy.Subscriber('received_messages', Frame, self.Echocallback)
       

    def motorenable(self):

        can_id = 0x140

        data = bytearray(8)
        data[0] = 0x02
        data[1] = 0x01
        data[2] = 0x00
        data[3] = 0x00
        data[4] = 0x00
        data[5] = 0x00
        data[6] = 0x01
        data[7] = 0x01

        CanFrame = Frame()
        CanFrame.id = can_id
        for i in range(8):
            CanFrame.data[i] = data[i]

        self.cmd_pub.publish(CanFrame)
    

    def VelCallback(self, data):

        twist = Twist()
        twist = data

        vel = twist.linear.x 
        delta =twist.angular.z

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


#	    data1 = (Canvel&255)
#	    data2 = (Canvel>>8)
#    	data3 = (Candelta&255)
#	    data4 = (Candelta>>8)


	#CanFrame.data =  index + module + command + channel + data1 + data2 + data3 + data4
    # CanFrame.data[0] = 1
 
        CanFrame = Frame()
        CanFrame.id = can_id
        for i in range(8):
            CanFrame.data[i] = data[i]

        self.cmd_pub.publish(CanFrame)

        """
        CanFrame.data[0] = '\x02'
        CanFrame.data[1] = '\x23'
        CanFrame.data[2] = '\x00'
        CanFrame.data[3] = '\x00'
        CanFrame.data[3] = '\x00'
        CanFrame.data[3] = '\x00'
        CanFrame.data[3] = '\x00'
        """

    def Echocallback(self, Frame_data):
       """
        print("call_back")
        id = Frame_data.id
        data = Frame_data.data
       
       
        index = data[0]
	module = data[1]
	command = data[2]
	channel = data[3]

        data1 = data[4]
        data2 = data[5]

        data3 = data[6]
        data4 = data[7]

        dataset1 = 0
        dataset1 = dataset1 | data1 << 8
        dataset1 = dataset1 | data2

        dataset2 = 0
        dataset2 = dataset2 | data3 << 8
        dataset2 = dataset2 | data4


        hexdataset1 = self.hex_to_float(dataset1)
        hexdataset2 = self.hex_to_float(dataset2)

        print('index={}, module={}, command={}, channel={}, vel={}, delta={}'.format(hex(index),  hex(module),  hex(command),  hex(channel), hex(hexdataset1), hex(hexdataset2))
        """
      

    def main(self):
        rospy.spin()



   
if __name__ == '__main__':
    rospy.init_node("CanNode", anonymous=True)
    node = CanNode()
    node.main()      
    




