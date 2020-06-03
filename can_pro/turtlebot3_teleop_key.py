#!/usr/bin/env python

# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy
from geometry_msgs.msg import Twist
import sys, select, os
import threading
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

msg = """
Control Your TurtleBot3!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity (Burger : ~ 0.22, Waffle and Waffle Pi : ~ 0.26)
a/d : increase/decrease angular velocity (Burger : ~ 2.84, Waffle and Waffle Pi : ~ 1.82)

space key, s : force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""

def getKey():
    if os.name == 'nt':
      return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(target_linear_vel, target_angular_vel):
    return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)

def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input

    return output

def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input

    return input

def checkLinearLimitVelocity(vel):
    if turtlebot3_model == "burger":
      vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
    elif turtlebot3_model == "waffle" or turtlebot3_model == "waffle_pi":
      vel = constrain(vel, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)
    else:
      vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)

    return vel

def checkAngularLimitVelocity(vel):
    if turtlebot3_model == "burger":
      vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
    elif turtlebot3_model == "waffle" or turtlebot3_model == "waffle_pi":
      vel = constrain(vel, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)
    else:
      vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)

    return vel



vel_stop = 0
vel_dec_10_state = 0
vel_dec_20_state = 0




sec_10_count = 0
sec_20_count = 0

def timer_run_10_dec(): #intoosh
    vel_dec_10_state = 1
    global sec_10_count
    sec_10_count += 1
  #  print(count)

    timer = threading.Timer(0.1, timer_run_10_dec)
    timer.start()

    if sec_10_count == 5:
        print("Canceling Timer..")
        timer.cancel()


def timer_run_20_dec(): # intoosh
    vec_dec_20_state = 1
    global sec_20_count
    sec_20_count += 1
  #  print(count)

    timer = threading.Timer(0.1, timer_run_20_dec)
    timer.start()

    if sec_20_count == 6:
        print("Canceling Timer..")
        timer.cancel()




def timer_vel_dec(swi):  #intoosh
    timer_vel = 0.0
    if swi == 1:
      timer_vel = 30-(0.50)
    elif swi == 2:
      timer_vel = 15-(0.64)
    elif swi == 3:
      timer_vel = 50 -(0.50)     
    elif swi == 4:
      timer_vel = 25-(0.64)

    if (swi == 1 or swi == 3):
      timer = threading.Timer(timer_vel, timer_run_10_dec)
      timer.start()
    elif (swi == 2 or swi == 4):
      timer = threading.Timer(timer_vel, timer_run_20_dec)
      timer.start()

def timer_run(): # intoosh
    vel_stop = 1

def timer_vel_stop(timer_vel): #intoosh
    timer = threading.Timer(timer_vel, timer_run)
    timer.start()







if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('turtlebot3_teleop')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    turtlebot3_model = rospy.get_param("model", "burger")

    status = 0
    target_linear_vel   = 0.0
    target_angular_vel  = 0.0
    control_linear_vel  = 0.0
    control_angular_vel = 0.0

    vel_timer = 0.0
    try:
        print msg
        while(1):
            key = getKey()
            if key == 'w' :
                target_linear_vel = checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE)
                status = status + 1
                print vels(target_linear_vel,target_angular_vel)
            elif key == 'x' :
                target_linear_vel = checkLinearLimitVelocity(target_linear_vel - LIN_VEL_STEP_SIZE)
                status = status + 1
                print vels(target_linear_vel,target_angular_vel)
            elif key == 'a' :
                target_angular_vel = checkAngularLimitVelocity(target_angular_vel + ANG_VEL_STEP_SIZE)
                status = status + 1
                print vels(target_linear_vel,target_angular_vel)
            elif key == 'd' :
                target_angular_vel = checkAngularLimitVelocity(target_angular_vel - ANG_VEL_STEP_SIZE)
                status = status + 1
                print vels(target_linear_vel,target_angular_vel)

            elif key == 't' :  #intoosh
                vel_timer = 2
                for i in range(vel_timer):
                  target_linear_vel = checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE)
                  status = status + 1
                  print vels(target_linear_vel,target_angular_vel)    

                timer_vel_stop(50)

            elif key == 'y' :  #intoosh
                vel_timer = 5
                for i in range(vel_timer):
                  target_linear_vel = checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE)
                  status = status + 1
                  print vels(target_linear_vel,target_angular_vel)

                timer_vel_stop(20)

            elif key == 'u' :  #intoosh
                vel_timer = 10
                for i in range(vel_timer):
                  target_linear_vel = checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE)
                  status = status + 1
                  print vels(target_linear_vel,target_angular_vel)
                
                timer_vel_dec(1)

            elif key == 'i' :  #intoosh
                vel_timer =20
                for i in range(vel_timer):
                  target_linear_vel = checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE)
                  status = status + 1
                print vels(target_linear_vel,target_angular_vel)

                timer_vel_dec(2)
                
            elif key == 'o' :  #intoosh
                vel_timer = 10
                for i in range(vel_timer):
                  target_linear_vel = checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE)
                  status = status + 1
                  print vels(target_linear_vel,target_angular_vel)

                timer_vel_dec(3)

            elif key == 'p' :  #intoosh
                vel_timer =20
                for i in range(vel_timer):
                  target_linear_vel = checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE)
                  status = status + 1
                  print vels(target_linear_vel,target_angular_vel)

                timer_vel_dec(4)

            elif key == 'g' :
                target_linear_vel = checkLinearLimitVelocity(target_linear_vel - LIN_VEL_STEP_SIZE)
                status = status + 1
                print vels(target_linear_vel,target_angular_vel) 

            elif key == ' ' or key == 's' :
                target_linear_vel   = 0.0
                control_linear_vel  = 0.0
                target_angular_vel  = 0.0
                control_angular_vel = 0.0
                print vels(target_linear_vel, target_angular_vel)
            else:
                if (key == '\x03'):
                    break

            if status == 20 :
                print msg
                status = 0

            if vel_stop == 1 :
                target_linear_vel   = 0.0
                control_linear_vel  = 0.0
                target_angular_vel  = 0.0
                control_angular_vel = 0.0
                print vels(target_linear_vel, target_angular_vel)
                vel_stop = 0

            if vel_dec_10_state == 1:
              if sec_10_count == 1:
                  for i in range(2):  # 8
                    target_linear_vel = checkLinearLimitVelocity(target_linear_vel - LIN_VEL_STEP_SIZE)
              
              elif sec_10_count == 2:
                  for i in range(2): # 6
                    target_linear_vel = checkLinearLimitVelocity(target_linear_vel - LIN_VEL_STEP_SIZE)
              elif sec_10_count == 3:
                                    # 5
                  target_linear_vel = checkLinearLimitVelocity(target_linear_vel - LIN_VEL_STEP_SIZE)

              elif sec_10_count == 4:
                  for i in range(2): # 3
                    target_linear_vel = checkLinearLimitVelocity(target_linear_vel - LIN_VEL_STEP_SIZE)
              elif sec_10_count == 5:
                  for i in range(3): # 0

                    target_linear_vel = checkLinearLimitVelocity(target_linear_vel - LIN_VEL_STEP_SIZE)

                  sec_10_count = 0
                  vel_dec_10_state = 0
                  vel_stop = 1


            if vel_dec_20_state == 1:
              if sec_20_count == 1:
                  for i in range(4):  # 16
                    target_linear_vel = checkLinearLimitVelocity(target_linear_vel - LIN_VEL_STEP_SIZE)
              
              elif sec_20_count == 2:
                  for i in range(3): # 13
                    target_linear_vel = checkLinearLimitVelocity(target_linear_vel - LIN_VEL_STEP_SIZE)

              elif sec_20_count == 3:
                  for i in range(3): # 10
                    target_linear_vel = checkLinearLimitVelocity(target_linear_vel - LIN_VEL_STEP_SIZE)

              elif sec_20_count == 4:
                  for i in range(4): # 6
                    target_linear_vel = checkLinearLimitVelocity(target_linear_vel - LIN_VEL_STEP_SIZE)

              elif sec_20_count == 5:
                  for i in range(3): # 3
                    target_linear_vel = checkLinearLimitVelocity(target_linear_vel - LIN_VEL_STEP_SIZE)

              elif sec_20_count == 6:
                  for i in range(3): # 0
                    target_linear_vel = checkLinearLimitVelocity(target_linear_vel - LIN_VEL_STEP_SIZE)


                  sec_20_count = 0
                  vel_dec_20_state = 0
                  vel_stop = 1


            twist = Twist()

            control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
            twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

            control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel

            pub.publish(twist)

    except:
        print e

    finally:
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        pub.publish(twist)

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)