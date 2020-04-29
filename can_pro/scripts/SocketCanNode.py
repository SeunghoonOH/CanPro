#!/usr/bin/env python

import rospy
from can_msgs.msg import Frame
from geometry_msgs.msg import Twist
import struct
import numpy as np
import sys
import threading
from enum import Enum
class CanInfo(Enum):
    motor = 0x02
    wheel_error = 0xF0
    

class CanNode():
    def __init__(self):

        self.id = 0
        self.data_len = 0
        self.data = None
        self.read_lock = threading.RLock()   
        self.callbacks = dict()
        self.callbacks[CanInfo.motor.value + CanInfo.wheel_error.value ] = self.can_error

	    self.can_sub = rospy.Subscriber('received_messages', Frame, self.callback_echo)
	    self.vel_sub = rospy.Subscriber('cmd_vel', Twist, self.VelCallback)
        self.cmd_pub = rospy.Publisher('sent_messages', Frame, queue_size=10)
	    num = self.cmd_pub.get_num_connections()

	    while not rospy.is_shutdown() and self.cmd_pub.get_num_connections() == num :
		    pass

        self.motorenable()

    def bytes_to_int(self, bytes):
        result = 0
          for b in bytes:
              result = result * 256 + int(b)

        return result


    def tryRead(self, index, length):
        
        bytes_remaining = length
        result = bytearray()
        while bytes_remaining != 0:
            with self.read_lock:
                received = self.data(index)
                index = index + 1
                if( index == self.data_len ):
                    index = self.data_len - 1   
                
            if len(received) != 0:
                result.extend(received)
                bytes_remaining -= len(received)

        return bytes(result), length

        
       
    #    self.echo_sub = rospy.Subscriber('received_messages', Frame, self.Echocallback)

    def callback_echo(self, frame):
        
        with self.read_lock:
            self.data_len = frame.dlc
            if(self.data_len == 0):
                pass
            
            self.id = frame.id
            if(self.id  == 0):
                pass

            self.data = frame.data
            if(self.data == ' '):
                pass

        if(self.id == 0x141):

            print(hex(id))	   
      #      data = frame.data
       #     len = frame.dlc
        
          
            data = ''
            read_step = None
            data_index = 0
            read_index = 0
            callback_id =0

            # Find can index
            read_step = 'index'
            can_index, read_index = self.tryRead(data_index, 1)
            if (can_index == 0x00):
                pass

            data_index = data_index + read_index  
            callback_id = callback_id + self.bytes_to_int(can_index)
        
            # Find can module
            read_step = 'module'
            can_module, read_index = self.tryRead(data_index, 1)
            if (can_module == 0x00):
                pass

            data_index = data_index + read_index  
            callback_id = callback_id + self.bytes_to_int(can_module)

            # Read can command
            read_step = 'command'
            can_command, read_index = self.tryRead(data_index, 1)
            if(can_command != 0x00):
                callback_id = callback_id + self.bytes_to_int(can_command)

            data_index = data_index + read_index  

            # Read can channel
            read_step = 'chennel'
            can_channel, read_index = self.tryRead(data_index, 1)
            if(can_channel != 0x00):
                callback_id = callback_id + can_channel

            data_index = data_index + read_index  

            # Read can data (4 bytes)
            read_step = 'data'        
            can_data, read_index = self.tryRead(data_index, 4)
            if(can_data != 0x00):
                pass

            data_index = data_index + read_index  
            if(data_index !=  (self.data_len -1))
                rospy.logerr("Error opening serial: %s", e)
                

            self.callbacks[callback_id](can_data)
            


    def motorenable(self):

        can_id = 0x140

        EnableCanFrame = Frame()
        EnableCanFrame.id = can_id
	EnableCanFrame.dlc = 8
	EnableCanFrame.is_rtr = 0
	EnableCanFrame.is_extended = 0
	EnableCanFrame.is_error = 0
	EnableCanFrame.data = "\x02" + "\x10\x00\x00\x00\x00\x00\x00"

        self.cmd_pub.publish(EnableCanFrame)


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
	#strdata = data.deco
	
#	    data1 = (Canvel&255)
#	    data2 = (Canvel>>8)
#    	data3 = (Candelta&255)
#	    data4 = (Candelta>>8)


	#CanFrame.data =  index + module + command + channel + data1 + data2 + data3 + data4
        
#+ 0x03 + 0x00 + 0x00 + 0x00 + 0x00 + 0x00 + 0x00 
 
        CanFrame = Frame()
        CanFrame.id = can_id
	CanFrame.dlc = 8
	CanFrame.is_rtr = 0
	CanFrame.is_extended = 0
	CanFrame.is_error = 0
	CanFrame.data = "\x02" + "\x03\x00\x00" +  chr(Canvel >>8) + chr(Canvel & 255) + chr(Candelta >>8) +chr(Candelta & 255)
#        for i in range(8):
#            CanFrame.data[i] = "11"

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

	print("receive")

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
    




