#!/usr/bin/env python

import rospy
from can_msgs.msg import Frame
from geometry_msgs.msg import Twist
import struct
import numpy as np


class CanNode():
    def __init__(self):
     
        self.vel_sub = rospy.Subscriber('cmd_vel', Twist, self.VelCallback)
        self.cmd_pub = rospy.Publisher('sent_messages', Frame, queue_size=10)
        self.echo_sub = rospy.Subscriber('received_messages', Frame, self.Echocallback)
       
    
    def float_to_hex(self, f):
        return hex(struct.unpack('<H', np.float16(f).tobytes())[0])
    
    def hex_to_float(self, hex):
        a = struct.pack("i", int(hex,16))
        return  np.frombuffer(a, dtype= np.float16)[0])

    def VelCallback(self, data):

        twist = Twist()
        twist = data

        vel = twist.linear.x 
        delta =twist.angular.z 
        
        if( vel > 1000):
            vel = 1000
        elif ( vel < - 1000):
            vel = -1000

        if(delta > 200):
            delta = 200
        elif (delta < -200):
            delta = -200

        Canvel = np.float16(vel)
        Candelta = np.float16(delta)
    	print(delta)

        CanFrame = Frame()

        CanFrame.id = int('0x141', 16)
	
	index = np.uint8(int('0x02', 16))
	module = np.uint8(int('0x23', 16))
	command = np.uint8(int('0x00', 16))
	channel = np.uint8(int('0x00', 16))

        VelHex = self.float_to_hex(Canvel)
        VelHex1 = np.uint8(VelHex >> 8 & 0xFF)
        VelHex2 = np.uint8(VelHex  & 0xFF)

        DeltaHex = self.float_to_hex(Candelta)        
        DeltaHex1 = np.uint8(DeltaHex >> 8 & 0xFF)
        DeltaHex2 = np.uint8(DeltaHex  & 0xFF)

#	    data1 = (Canvel&255)
#	    data2 = (Canvel>>8)
#    	data3 = (Candelta&255)
#	    data4 = (Candelta>>8)


	#CanFrame.data =  index + module + command + channel + data1 + data2 + data3 + data4
    # CanFrame.data[0] = 1
       
 	CanFrame.data =  index + module + command + channel + VelHex1 + VelHex2 + DeltaHex1 + DeltaHex2
        
	
	
	    print('index={}, module={}, command={}, channel={}, vel={}, delta={}'.format(hex(CanFrame.data[0]),  hex(CanFrame.data[1]),  hex(CanFrame.data[2]),  hex(CanFrame.data[3]), hex(VelHex), hex(DeltaHex))

	"""
        ROS_INFO("data[0] =%X , data[1] =%X., data[2] =%X, data[3] =%X , vel =%d, delta =%d", CanFrame.data[0], CanFrame.data[1], CanFrame.data[2], CanFrame.data[3], vel, delta )
        """

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
   
      

    def main(self):
        rospy.spin()



   
if __name__ == '__main__':
    rospy.init_node("CanNode", anonymous=True)
    node = CanNode()
    node.main()      
    




