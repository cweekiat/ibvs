#! /usr/bin/env python

import sys
import time
import math
import argparse
import rospy
from std_msgs.msg import Float64, Float64MultiArray
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

observer_udp = '127.0.0.1:14561' 
display_size = (1280,720)
FOV = 68.7549
PID_switch = True
max_lin_vel = 3
max_ang_vel = 3
heading, vx_output, vz_output = 0,0,0


ros_rate = 15 # must be more than 2Hz

class drone():
    def __init__(self, name, FOV = FOV, Kp =0.2, Ki= 0.1, Kd=0.0, sample_time = int(1/10), 
    desired_error_x = 0, desired_error_y = 0, desired_error_w = 100, desired_error_h = 44):
    
        #self.image_sub = rospy.Subscriber('/camera_coord', Float64MultiArray, self.callback)
        
        #self.local_vel_sub = rospy.Subscriber('/observer/cmd_vel', Float64MultiArray, self.vel_callback)
        
        self.state_sub = rospy.Subscriber("mavros/state", State, self.state_callback)    
        
        self.local_vel_pub = rospy.Publisher("mavros/setpoint_raw/local", PositionTarget, queue_size=10)

        rospy.wait_for_service("/mavros/cmd/arming")
        self.arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

        rospy.wait_for_service("/mavros/set_mode")
        self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
    
        self.name = name
        self.current_state = State()
        self.cmd_vel=PositionTarget()
        self.cmd_vel.coordinate_frame = 8
        self.cmd_vel.type_mask = 0b010111000111
        self.cmd_vel.velocity.x = 0
        self.cmd_vel.velocity.y = 0
        self.cmd_vel.yaw_rate = 0.0
        

    def state_callback(self,data):
        #global current_state
        #data = data.data
        self.current_state = data

    def vel_callback(self,data):
        data = data.data
        self.cmd_vel.velocity.x = data[0]
        self.cmd_vel.velocity.y = data[1]
        self.cmd_vel.yaw_rate = data[2]

    def set_mode(self,arg='AUTO.LAND'): #change to position control
        set_Mode = SetModeRequest()
        set_Mode.custom_mode = arg
        if(self.set_mode_client.call(set_Mode).mode_sent == True):
            rospy.loginfo(arg+" enabled")
            rospy.sleep(1)
            rospy.loginfo('Current mode: ' + self.current_state.mode)
        else:
            rospy.loginfo(arg+ " cannot be enabled")

    def stop(self):
        self.cmd_vel.velocity.x = 0.0
        self.cmd_vel.velocity.y = 0.0
        self.cmd_vel.yaw_rate = 0.0
        self.local_vel_pub.publish(self.cmd_vel)    

   
def main():
    try:
        rospy.init_node("offb_node_py")
        observer  = drone('observer')
        rospy.loginfo('Initialising Observer Drone...')
        rospy.on_shutdown(observer.set_mode)
        rate = rospy.Rate(ros_rate)

        # Wait for Flight Controller connection
        while(not rospy.is_shutdown() and not observer.current_state.connected):
            rospy.loginfo('Not connected. Trying to connect...')
            rate.sleep()

        # Send a few setpoints before starting
        for i in range(100):
            if(rospy.is_shutdown()):
                break
            observer.stop()
            rate.sleep()
            
        observer.cmd_vel.yaw_rate = 0.2
        
        last_req = rospy.Time.now()

        
        while(not rospy.is_shutdown()):
            if(observer.current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(1.0)):
                rospy.loginfo("OFFBOARD is NOT enabled on RC")
                rospy.loginfo('Current mode: ' + observer.current_state.mode)
                last_req = rospy.Time.now()
			    
            else:
                if(not observer.current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(1.0)):
                    rospy.loginfo("Vehicle is NOT armed")
                    last_req = rospy.Time.now()

                else:
                    if((rospy.Time.now() - last_req) > rospy.Duration(1.0)):
                        msg = [observer.cmd_vel.velocity.x, observer.cmd_vel.velocity.y,observer.cmd_vel.yaw_rate]
                        rospy.loginfo(msg)
                        last_req = rospy.Time.now()
                    
                    observer.local_vel_pub.publish(observer.cmd_vel)             

            rate.sleep()
            
        observer.stop()
        print('Stopping')
        
            
    except KeyboardInterrupt:
        print("Shutting down")



if __name__ == '__main__':
    try: 
        main()
    except rospy.ROSInterruptException:
        pass
    

        
        

