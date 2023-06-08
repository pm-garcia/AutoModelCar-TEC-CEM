#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16, UInt8

regions = {}  # Dictionary to store range values of different regions
estado = 'PrimerPaso'  # State variable for the parking process
parking_space = []  # List to track the parking space detection

def lidar_callback(laser_data):
    global regions
    ranges = laser_data.ranges

    front_index = ranges[0:8] + ranges[352:360]

    rango_minimo = 0.1  
    rango_maximo = 10.0  

    regions = {
        "front": min(max(min(front_index), rango_minimo), rango_maximo),
        "right_front": min(max(min(ranges[299:344]), rango_minimo), rango_maximo),
        "left_front": min(max(min(ranges[23:68]), rango_minimo), rango_maximo),
        "right": min(max(min(ranges[253:298]), rango_minimo), rango_maximo),
        "left": min(max(min(ranges[69:114]), rango_minimo), rango_maximo),
        "back": min(max(min(ranges[161:206]), rango_minimo), rango_maximo),
        "right_back": min(max(min(ranges[207:252]), rango_minimo), rango_maximo),
        "left_back": min(max(min(ranges[115:160]), rango_minimo), rango_maximo)
    }

def end_callback():
    # Stop the car when the program ends
    steering = 90
    speed = 0
    pSpeed.publish(speed)
    pSteering.publish(steering) 

def stop():
    # Stop the car's movement
    speed = 0
    steering = 90
    pSpeed.publish(speed)
    pSteering.publish(steering)

def turn_right():
    # Turn the car to the right
    speed = 100
    steering = 0
    pSpeed.publish(speed)
    pSteering.publish(steering)

def turn_left():
    # Turn the car to the left
    speed = 100
    steering = 180
    pSpeed.publish(speed)
    pSteering.publish(steering)

def back_right():
    # Move the car backward and to the right
    speed = -100
    steering = 0
    pSpeed.publish(speed)
    pSteering.publish(steering)

def back_left():
    # Move the car backward and to the left
    speed = -100
    steering = 180
    pSpeed.publish(speed)
    pSteering.publish(steering)

def backwards():
    # Move the car backward
    speed = -100
    steering = 90
    pSpeed.publish(speed)
    pSteering.publish(steering)

def forwards():
    # Move the car forward
    speed = 100
    steering = 87
    pSpeed.publish(speed)
    pSteering.publish(steering)

def steering_0():
    # Adjust the steering to 0 degrees
    steering = 10
    speed = -90
    pSpeed.publish(speed)
    pSteering.publish(steering)

def steering_180():
    # Adjust the steering to 180 degrees
    steering = 170
    speed = -90
    pSpeed.publish(speed)
    pSteering.publish(steering)

def forwards_parking():
    # Move the car forward during the parking process
    speed = 90
    steering = 10
    pSpeed.publish(speed)
    pSteering.publish(steering)

def main():
    while not rospy.is_shutdown():
        global estado
        if len(regions) != 0:
            if estado == 'BuscandoEspacio':
                forwards()
                if regions['right'] < 0.4 and len(parking_space) == 0:
                    parking_space.append(1)
                    print("Car detected")
                elif regions['right'] > 0.4 and len(parking_space) == 1:
                    parking_space.append(1)
                    print("Space detected")
                elif regions['right'] < 0.4 and len(parking_space) == 2:
                    parking_space.append(1)
                    print("Car detected")
                    estado = 'Alinearse'
            
            elif estado == 'Alinearse':
                 forwards()
                 print(regions['right'])
                 if regions['right'] > 0.4:
                    print("Aligned")
                    estado = 'PrimerPaso'
                    stop()

            elif estado == 'PrimerPaso':
                steering_0()
                print("Right",regions['right'])
                print("Back", regions['back'])
                if regions['back'] < 0.5 and regions['right'] < 0.3:
                    estado = 'PrimerPasoCorrecion'

            elif estado == 'PrimerPasoCorrecion':
                backwards()
                print("Right",regions['right'])
                print("Back", regions['back'])
                if regions['back'] < 0.35 and regions['right'] < 0.4:
                    estado = 'SegundoPaso'
                
            elif estado == 'SegundoPaso':
                steering_180()
                print("Right",regions['right'])
                print("Back", regions['back'])
                if regions['back'] < 0.20 and regions['right'] < 0.20:
                    estado = 'TercerPaso'
            
            elif estado == 'TercerPaso':
                forwards_parking()
                if regions['front'] < 0.5:
                    estado = 'CuartoPaso'

            elif estado == 'CuartoPaso':
                backwards()
                if regions['back'] < 0.35:
                    estado = 'QuintoPaso'

            elif estado == 'QuintoPaso':
                forwards_parking()
                if regions['front'] < 0.35:
                    estado = 'SextoPaso'

            elif estado == 'SextoPaso':
                backwards()
                if regions['back'] < 0.32:
                    stop()

        rate.sleep()

if __name__ == '__main__':
    try:
        # Initialize the ROS node for parking
        rospy.init_node('parking')

        # Subscribe to the '/scan' topic to receive laser scan data
        rospy.Subscriber('/scan', LaserScan, lidar_callback)

        # Create publishers for the '/speed' and '/steering' topics
        pSpeed = rospy.Publisher('/speed', Int16, queue_size=10)
        pSteering = rospy.Publisher('/steering', UInt8, queue_size=10)

        rate = rospy.Rate(30)  # Set the loop rate to 30 Hz

        rospy.on_shutdown(end_callback)  # Register the end_callback() function to be called on shutdown

        main()  # Execute the main parking routine

    except rospy.ROSInterruptException:
        pass

