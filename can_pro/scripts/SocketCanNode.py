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
    wheel_enable = 0x01
    wheel_enable_req = 0x10
    wheel_velocity = 0x22
    wheel_velocity_req = 0x11
    wheel_vel_dir = 0x03
    wheel_error = 0xF0
    
class WheelError(Enum):
    FAULT = 0
    PWRGB = 1
    ADC = 2
    HALL = 3




class CanNode():
    def __init__(self):

        self.id = 0
        self.data_len = 0
        self.data = None
	self.init_motor = 0
        self.read_lock = threading.RLock()   
        self.callbacks = dict()
        self.callbacks[CanInfo.motor.value + CanInfo.wheel_enable.value   ] = self.motor_wheel_enble
	self.callbacks[CanInfo.motor.value + CanInfo.wheel_enable_req.value ] = self.motor_wheel_enable_req
        self.callbacks[CanInfo.motor.value + CanInfo.wheel_velocity.value ] = self.motor_wheel_velocity
	self.callbacks[CanInfo.motor.value + CanInfo.wheel_velocity_req.value ] = self.motor_wheel_velocity_req
        self.callbacks[CanInfo.motor.value + CanInfo.wheel_vel_dir.value ] = self.motor_wheel_vel_dir
        self.callbacks[CanInfo.motor.value + CanInfo.wheel_error.value ] = self.motor_wheel_error
	self.can_sub = rospy.Subscriber('received_messages', Frame, self.callback_echo)
	self.vel_sub = rospy.Subscriber('cmd_vel', Twist, self.VelCallback)
        self.cmd_pub = rospy.Publisher('sent_messages', Frame, queue_size=10)
	num = self.cmd_pub.get_num_connections()

	while not rospy.is_shutdown() and self.cmd_pub.get_num_connections() == num :
		pass

        self.motorenable()
	self.init_motor = 1 

    def motor_wheel_enble(self, can_data):
	print("wheel_enble")

    def motor_wheel_enable_req(self, can_data):
	print("whee_enable_req")
#	data_len =can_data.__len__()
	wheel_info = []
	enable_wheel = can_data[3]

	left_wheel = enable_wheel >> 4 & 0xF
	if(left_wheel > 0):
		wheel_info.append("left wheel enable ")
	else:
		wheel_info.append("left wheel disable ")	

	right_wheel = enable_wheel & 0xF
	if(right_wheel > 0):
		wheel_info.append("right wheel enable ")
	else:
		wheel_info.append("right wheel disable ")
	
	print(wheel_info)
	

    def motor_wheel_velocity(self, can_data):
	print("wheel_velocity")
    
    def motor_wheel_velocity_req(self, can_data):
	vel_data = can_data	
	vel_left = 0
	vel_left = vel_left | vel_data[0] << 8
	vel_left = vel_left | vel_data[1]

	vel_right = 0
	vel_right = vel_right | vel_data[2] << 8
	vel_right = vel_right | vel_data[3]
	
	print("wheel_velocity_req ")
	print("left : {0} , right : {1}".format(vel_left, vel_right))

    def motor_wheel_vel_dir(self, can_data):
	print("wheel_vel_dir")

    def motor_wheel_error(self, can_data):

	can_id = 0x140

        CanFrame = Frame()
        CanFrame.id = can_id
	CanFrame.dlc = 8
	CanFrame.is_rtr = 0
	CanFrame.is_extended = 0
	CanFrame.is_error = 0
	CanFrame.data = "\x02" + "\x03\x00\x00\x03\xE8\x00\x64"

        self.cmd_pub.publish(CanFrame)

        DisableCanFrame = Frame()
        DisableCanFrame.id = can_id
	DisableCanFrame.dlc = 8
	DisableCanFrame.is_rtr = 0
	DisableCanFrame.is_extended = 0
	DisableCanFrame.is_error = 0
	DisableCanFrame.data = "\x02" + "\x01\x00\x00\x00\x00\x00\x00"

        self.cmd_pub.publish(DisableCanFrame)
	
	print("error_data")

	error_type = []
	data_len = can_data.__len__()
	for i in range(data_len):
		error_left = can_data[i] >> 4 & 0xF
		if(error_left > 0):
			error_cont = WheelError(i).name + "_left "
			error_type.append(error_cont)
		#	error_type.append("left  ")					

		error_right = can_data[i] & 0xF
		if(error_right > 0):
			error_cont = WheelError(i).name + "_right "
			error_type.append(error_cont)
		#	error_type.append("right  ")

	
	print(error_type)
		

    def bytes_to_int(self, bytes):
        result = 0
        for b in bytes:
        	result = result * 256 + int(b)

        return result


    def tryRead(self, index, length):
        try:
		bytes_remaining = length
		result = bytearray()
		while bytes_remaining != 0:
		    with self.read_lock:
		        received = self.data[index]
		        index = index + 1
		        if( index == self.data_len ):
		            index = self.data_len - 1   
		        
		    if len(received) != 0:
		        result.extend(received)
		        bytes_remaining -= len(received)

	  	return result, length
	       # return bytes(result), length

	except Exception as e:
		rospy.logger("can data read failure : %s" %e )		

    def callback_echo(self, frame):
        
     
        
	    with self.read_lock:
           	 self.data_len = frame.dlc
           	 if(self.data_len == 0 | self.data_len > 8):
           	 	pass
            
	   	# print(self.data_len)
	         self.id = frame.id
        	 if(self.id  == 0):
                 	pass

	#	 print(self.id)
         	 self.data = frame.data
	         if(self.data == ' '):
                 	pass
	    
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
		
#            print(type(can_index))
#	    print(can_index)
            data_index = data_index + read_index  
#	    can_index_byte = str.encode(can_index)
#   	    print(type(can_index_byte))	
            callback_id = callback_id + int(can_index[0])
#	    print(callback_id)   
#	    print(type(callback_id))     

            # Find can module
            read_step = 'module'
            can_module, read_index = self.tryRead(data_index, 1)
            if (can_module == 0x00):
                pass

#	    print(type(can_module))
#	    print(can_module)
            data_index = data_index + read_index  
            callback_id = callback_id + int(can_module[0])
	    
#	    print(callback_id)
            # Read can command
            read_step = 'command'
            can_command, read_index = self.tryRead(data_index, 1)
          #  if(can_command != 0x00):
          #      callback_id = callback_id + can_command[0]

            data_index = data_index + read_index  

            # Read can channel
            read_step = 'chennel'
            can_channel, read_index = self.tryRead(data_index, 1)
         #   if(can_channel != 0x00):
         #       callback_id = callback_id + can_channel[0]

            data_index = data_index + read_index  

            # Read can data (4 bytes)
            read_step = 'data'        
            can_data, read_index = self.tryRead(data_index, 4)
      #      if(can_data != 0x00):
      #          pass
	    
#	    can_byte_data = str.encode(can_data)	
	  #  print(int(can_data[0], 16))
            data_index = data_index + read_index  
         #   if(data_index !=  (self.data_len -1))
         #       rospy.logerr("Error opening serial: %s", e)
                
#	    print(callback_id)
#	    print(type(callback_id))
	#    try:
            self.callbacks[callback_id](can_data)
	 #   except KeyError:
	#	rospy.logerr("tried to call callbackfunctikon before configurted, callbac_id %d" callback_id)
            


    def motorenable(self):

        can_id = 0x140

        EnableCanFrame = Frame()
        EnableCanFrame.id = can_id
	EnableCanFrame.dlc = 8
	EnableCanFrame.is_rtr = 0
	EnableCanFrame.is_extended = 0
	EnableCanFrame.is_error = 0
	EnableCanFrame.data = "\x02" + "\x01\x00\x00\x00\x00\x01\x01"

        self.cmd_pub.publish(EnableCanFrame)


    def VelCallback(self, data):

        twist = Twist()
        twist = data

        vel = twist.linear.x 
        delta =twist.angular.z

        Canvel = vel * 1000
        Candelta = delta * 10

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

	"""
        data = bytearray(8)
        data[0] = 0x02
        data[1] = 0x03
        data[2] = 0x00
        data[3] = 0x00
        data[4] = Canvel >> 8 & 0xFF
        data[5] = Canvel & 0xFF
        data[6] = Candelta >> 8 & 0xFF
        data[7] = Candelta & 0xFF
	"""

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

     
    def main(self):
        rospy.spin()



   
if __name__ == '__main__':
    rospy.init_node("CanNode", anonymous=True)
    node = CanNode()
    node.main()      
    




