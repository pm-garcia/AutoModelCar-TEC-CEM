#!/usr/bin/env python
"""
This node implements a proportional control and is intended to be used
together with the lane_detector node. It is assumed both lane borders 
are given by two straight lines in rho-theta form. Given a desired rho-theta
for each lane border, an error is calculated.
Steering is calculated proportional to this error and linear speed is
set as a constant. 
"""
import cv2
import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Int16, UInt8

#
# Steering is calculated proportional to two errors: distance error and angle error.
# These errors correspond to differences between an observed line (in normal form)
# and a desired line.
# Speed is calculated as the max speed minus a speed proportional to the steering.
# In this way, car goes with lower speed in curves and at max speed in straight roads. 

def end_callback():
    global pub_angle, pub_speed
    steering = 90
    speed = 0
    pub_speed.publish(speed)
    pub_angle.publish(steering)
#
def calculate_control(rho_l, theta_l, rho_r, theta_r, goal_rho_l, goal_theta_l, goal_rho_r, goal_theta_r):
    global max_speed, k_rho, k_theta
    error_rho_l   = goal_rho_l   - rho_l
    error_theta_l = goal_theta_l - theta_l
    error_rho_r   = rho_r - goal_rho_r 
    error_theta_r = theta_r - goal_theta_r
    if rho_l != 0 and rho_r != 0:
        error_rho   = (error_rho_l + error_rho_r)/2
        error_theta = (error_theta_l + error_theta_r)/2
    elif rho_l != 0:
        error_rho   = error_rho_l
        error_theta = error_theta_l
    else:
        error_rho   = error_rho_r
        error_theta = error_theta_r
        
    print("rho", error_rho)
    print("th", error_theta)
    
    steering = -k_rho*error_rho - k_theta*error_theta
    vel_steer = -0.001*error_rho - 0.01*error_theta
    steering = max(15, min(165, steering))
    # steering = min(15, max(165, steering))
    speed = max_speed*(1 - 1.5*abs(vel_steer))
    # steering = np.degrees(steering)
    print("steering", steering)
    print("speed", speed)
    return speed, steering

def callback_left_lane(msg):
    global lane_rho_l, lane_theta_l
    lane_rho_l, lane_theta_l = msg.data

def callback_right_lane(msg):
    global lane_rho_r, lane_theta_r
    lane_rho_r, lane_theta_r = msg.data

def main():
    global lane_rho_l, lane_theta_l, lane_rho_r, lane_theta_r
    global max_speed, k_rho, k_theta
    max_speed = 100
    k_rho   = 27
    k_theta = 18
    lane_rho_l   = 0
    lane_theta_l = 0
    lane_rho_r   = 0
    lane_theta_r = 0
    # goal_rho_l   = 190
    # goal_theta_l = 2.15
    # goal_rho_r   = 190
    # goal_theta_r = 1.10

    #720
    #99.8
    
    goal_rho_l   = 211
    goal_theta_l = 2.4
    goal_rho_r   = 141
    goal_theta_r = 1.0

    if rospy.has_param('~max_speed'):
        max_speed = rospy.get_param('~max_speed')
    if rospy.has_param('~k_rho'):
        k_rho = rospy.get_param('~k_rho')
    if rospy.has_param('~k_theta'):
        k_theta = rospy.get_param('k_theta')
    
    print("Waiting for lane detection...")
    msg_left_lane  = rospy.wait_for_message('/demo/left_lane' , Float64MultiArray, timeout=100)
    msg_right_lane = rospy.wait_for_message('/demo/right_lane', Float64MultiArray, timeout=100)
    print("Using:")
    print("Max speed: " + str(max_speed))
    print("K_rho: " + str(k_rho))
    print("K_theta: " + str(k_theta))
    while not rospy.is_shutdown():
        speed, steering = calculate_control(lane_rho_l, lane_theta_l, lane_rho_r, lane_theta_r, goal_rho_l, goal_theta_l, goal_rho_r, goal_theta_r)
        pub_speed.publish(speed)
        pub_angle.publish(steering)
        rate.sleep()
    

if __name__ == '__main__':
    try:
        print('INITIALIZING LANE TRACKING NODE...')
        rospy.init_node('lane_tracking')

        # Configurar suscriptores para los topicos de la camara RealSense y el LIDAR RPLIDAR
        rospy.Subscriber("/demo/left_lane" , Float64MultiArray, callback_left_lane)
        rospy.Subscriber("/demo/right_lane", Float64MultiArray, callback_right_lane)

        pub_speed = rospy.Publisher('/speed', Int16, queue_size=10)
        pub_angle = rospy.Publisher('/steering', UInt8, queue_size=10)

        rate = rospy.Rate(30)

        rospy.on_shutdown(end_callback)

        # Ejecutar la rutina de estacionamiento en paralelo
        main()

    except rospy.ROSInterruptException:
        pass

    
