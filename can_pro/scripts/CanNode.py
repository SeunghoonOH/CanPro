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
       
    def VelCallback(self, data):

        twist = Twist()
        twist = data

        vel = twist.linear.x * 1000
        delta =twist.angular.z * 1000
        
        Canvel = np.uint8(vel)
        Candelta = np.uint8(delta)
	print(delta)

        CanFrame = Frame()

        CanFrame.id = int('0x141', 16)
	
	index = int('0x02', 16)
	module = int('0x23', 16)
	command = int('0x00', 16)
	channel = int('0x00', 16)
	data1 = (Canvel&255)
	data2 = (Canvel>>8)
	data3 = (Candelta&255)
	data4 = (Candelta>>8)


	#CanFrame.data =  index + module + command + channel + data1 + data2 + data3 + data4
       # CanFrame.data[0] = 1
       
	CanFrame.data =  '\x02' + '\x23' + '\x00' + '\x00' + chr(Canvel&255) + chr(Canvel>>8) + chr(Candelta&255) + chr(Candelta>>8)
        
	
	
	print('index={}, module={}, command={}, channel={}, vel={}, delta={}'.format(CanFrame.data[0],  CanFrame.data[1],  CanFrame.data[2],  CanFrame.data[3], Canvel, Candelta))

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
       
       velary = []
       velary.append(data[4])
       velary.append(data[5])

       deltarary = []
       deltarary.append(data[6])
       deltarary.append(data[7])

       vel, = struct.unpack("<h", velary.encode())
       delta, = struct.unpack("<h", deltarary.encode())

       ROS_INFO("data[0] =%X , data[1] =%X., data[2] =%X, data[3] =%X ,data[4] =%X, data[5] =%X, data[6] =%X, data[7] =%X", data[0], data[1], data[2], data[3], vel, delta)
        
      

    def main(self):
        rospy.spin()



   
if __name__ == '__main__':
    rospy.init_node("CanNode", anonymous=True)
    node = CanNode()
    node.main()      
    




