#!/usr/bin/env python

import rospy
import roslib
import sys
from std_msgs.msg import Float64, Float64MultiArray
from geometry_msgs.msg import Twist,TwistStamped
import time
import math
import argparse
from simple_pid import PID
from main import drone

PID_switch = True
FOV = 68.7549                   # field of view of camera in degrees
display_size = (640,360)        # size of video frame
max_lin_vel = 500        # max forward/vertical linear velocity 
max_ang_vel = 500               # max yaw rate 
scale = 1000
tolerance_x = 40
tolerance_y = 180
area_min = 100                                    # Min 1,000 Pixels
area_max = (display_size[0]/4)*(display_size[1]/4) # Max 14,400 Pixels
tolerance_area = [3000,8000]
desired_area = 80*50
search = False                  # Drone rotate in place to search for target, if set to True.

class visual_servo(drone):

    def __init__(self, Kp =1.0, Ki= 0.0, Kd=0.0, sample_time = 0.01):

        self.camera_sub = rospy.Subscriber('/observer/camera_coord', Float64MultiArray, self.callback)
        self.vel_pub= rospy.Publisher('/observer/cmd_vel', Float64MultiArray, queue_size=10)
       
        self.pidx = PID(1.0, 0.0, 0.0, setpoint = 0)
        self.pidy = PID(1.0, 0.0, 0.0, setpoint = 0)
        self.pida = PID(1.0, 0.0, 0.0, setpoint = desired_area)  

        self.pidx.sample_time = sample_time
        self.pidy.sample_time = sample_time
        self.pida.sample_time = sample_time

        self.pidx.output_limits = (-max_ang_vel, max_ang_vel)
        self.pidy.output_limits = (-max_lin_vel, max_lin_vel)
        self.pida.output_limits = (-max_lin_vel, max_lin_vel)
        
        self.x,self.y,self.w,self.h,self.det, self.area = 0,0,0,0,0,0
        self.v_x,self.v_y,self.yaw_rate = 0,0,0
        
    def callback(self,data):
        data = data.data
        
        # coordinates of bbox centre and width and height
        self.x,self.y,self.w,self.h,self.det = int(data[0]), int(data[1]), int(data[2]), int(data[3]), int(data[4])
        self.area = self.w*self.h


def main():
    try:
    
        rospy.init_node('visual_servo', anonymous=True)
        control = visual_servo()
        rate = rospy.Rate(20)           # minimum 2 Hz
        
        while not rospy.is_shutdown():

            error_x = control.x - display_size[0]/2 
            error_y = control.y - display_size[1]/2
            error_area = control.area - desired_area
            #print(x,y,w,h,det)
            
            if control.det == 0:    # No Detection of Drone
                rospy.loginfo('No drone detected')
                if search:
                    # Rotate ccw to search for target
                    control.v_x, control.v_y, control.yaw_rate = 0,0,300
                else:
                    # Stop Moving
                    control.v_x, control.v_y, control.yaw_rate = 0,0,0
                    
                
            else:
                if (abs(error_x) > tolerance_x):
                    control.yaw_rate = control.pidx(error_x) 
                else:
                    control.yaw_rate = 0
                
                if control.area > area_max:
                    control.v_x = -max_lin_vel
                elif control.area < area_min and control.area > 0:
                    control.v_x = max_lin_vel
                else:
                    if control.area > tolerance_area[0] and control.area < tolerance_area[1]:
                        control.v_x = 0
                    else:
                        control.v_x = control.pida(error_area)
            
            cmd_vel = (control.v_x/scale, control.v_y/scale, control.yaw_rate/scale)
            rospy.loginfo(cmd_vel)
            control.vel_pub.publish(Float64MultiArray(data = cmd_vel))
            
            rate.sleep()

        print("Shutting down")
        control.v_x, control.v_y, control.yaw_rate = 0,0,0
        cmd_vel = (control.v_x/scale, control.v_y/scale, control.yaw_rate/scale)
        control.vel_pub.publish(Float64MultiArray(data = cmd_vel))
        
        
    except KeyboardInterrupt:
        print("Shutting down")
        control.v_x, control.v_y, control.yaw_rate = 0,0,0
        cmd_vel = (control.v_x/scale, control.v_y/scale, control.yaw_rate/scale)
        control.vel_pub.publish(Float64MultiArray(data = cmd_vel))

   
if __name__ == '__main__':
    main()  
