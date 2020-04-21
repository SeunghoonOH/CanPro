#!/usr/bin/env python

import rospy
#from can_msgs.msg import Frame
from geometry_msgs.msg import Twist
import struct
import numpy as np

from ctypes import *
import math 

from serial import SerialException
from time import sleep


def bytes_to_int(bytes):
    result = 0
    for b in bytes:
        result = result * 256 + int(b)
    return result

def float16_to_hex(f):
    return hex(struct.unpack('<H', np.float16(f).tobytes())[0])


def hex_to_float16(hex):
    a = struct.pack("i", int(hex,16))
    return np.frombuffer(a, dtype= np.float16)[0]  


class CanNode():
    def __init__(self, port, bandrate):
        
        
        self.port = serial.Serial(port, bandrate)
        if self.port is None:
            print("Port is None")
            return

        self.motor_enable()
        self.vel_sub = rospy.Subscriber("cmd_vel", Twist, self.velcallback)
   #     self.cmd_pub = rospy.publisher('received_messages', Frame, queue_size=10)
   #     self.echo_sub = rospy.Subscriber("sent_messages", Frame, Echocallback)

    def motor_enable(self):

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

        self.makecmd(can_id, data)

    def velcallback(self, data):

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

        sum += buff[send_len]
        send_len +=1

        for i in range(8):
            buff[send_len] = data[i]
            sum += buff[send_len]
            send_len +=1

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

    def Echocallback(self):
                
        readbuff=bytearray(18)
        reabuff_len = 0
        readsum = 0
        state = 0

        flag = self.port.read(1)   #0
        if (flag == b'\x02'):
            readsum = 0
            state += 1

        data_len = self.port.read(1)
        if(data_len > b'\x08'):
            state = 0
        else:
            readsum += bytes_to_int(data_len)
            state += 1
            print(bytes_to_int(data_len))

        #print(readsum)

        cancmd  = self.port.read(1)  #2
        readsum += bytes_to_int(cancmd)
        state += 1
        #print(readsum)
        #print(bytes_to_int(b'\x30'))

        #print(int_to_bytes(48,1))

        can_id1 = self.port.read(1)  # 3
        int_can_id1 = bytes_to_int(can_id1)
        #int_can_id1 = int_can_id1 & 0xFF

        readsum += int_can_id1
        #sum += can_id
        state += 1
        print(readsum)

        can_id2 = self.port.read(1)   #4
        int_can_id2 = bytes_to_int(can_id2)
        #int_can_id2 = int_can_id2 & 0xFF

        #int_can_id2= (int(can_id2.decode(), base=16))
        print(int_can_id2)

        readsum += int_can_id2
        #sum += bytes_to_int(can_id1) 
        print(readsum)

        can_id3 = bytes_to_int(can_id2)  << 8 | bytes_to_int(can_id1)
        state += 1

        #can_id = ser.read(2)
        #print(can_id)
        #topic_id, = struct.unpack("<h", can_id)
        #print(topic_id)

        #print(int_to_bytes(can_id1,2))

        cantype = self.port.read(1)  #5
        int_cantype = bytes_to_int(cantype)
        #int_cantype = int_cantype & 0xFF

        readsum += int_cantype
        state += 1
        #reabuff_len += 1
        #print(readbuff[reabuff_len])

        int_data = 0
        read_buff_len = 0
        read_buff = bytearray(8)  #6
        for i in range(8):
            int_data = bytes_to_int(self.port.read(1))
        #  int_data = int_data &  0xFF
            read_buff[i] = int_data
            #read_buff[read_buff_len] = readbuff[reabuff_len]
            readsum += int_data
            int_data = 0
        #   state += 1
            #reabuff_len += 1
        
        state += 1
     #   print(readsum)

        timestamp1 = self.port.read(1)  # 7
        #can_id = can_id 
        int_timestamp1 = bytes_to_int(timestamp1)
        #int_timestamp1 = int_timestamp1 & 0xFF

        readsum += int_timestamp1
        state += 1
        #reabuff_len += 1

        print(readsum)


        timestamp2 = self.port.read(1) #8

        int_timestamp2 = bytes_to_int(timestamp2)
        #int_timestamp2 = int_timestamp2 & 0xFF
        #int_timestamp2= (int(timestamp2.decode(), base=32))

        readsum += int_timestamp2
        readsum = readsum & 0xFF
        tempstamp = bytes_to_int(timestamp2)  << 8 | bytes_to_int(timestamp1)
        state += 1

        sum_data  = self.port.read(1)   # 9
        print(bytes_to_int(sum_data))

        if (bytes_to_int(sum_data) == readsum):
            print("ok")
            state += 1
        else:
            state = 0

        eof = bytes_to_int(self.port.read(1))
        if(eof == 0x03):
            print("end")

       

    def run(self):
        while not rospy.is_shutdown():
            try:
                with self.read_lock:
                    if self.port.inWaiting() < 1:
                        sleep(0.001)
                        continue

                self.Echocallback()


    def main(self):
        rospy.spin()


    def end(self):
        self.port.Close()
   
if __name__ == '__main__':
    rospy.init_node("CanNode", anonymous=True)

    while not rospy.is_shutdown():
        try:
            node = CanNode('/dev/ttyUSB0', 500000)
            node.run()
        except KeyboardInterrupt:
            break
        except SerialException:
            sleep(1.0)
            continue
        except OSError:
            sleep(1.0)
            continue

    #node.end()

