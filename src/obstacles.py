#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16, String, UInt8

# l = lengh 42
# widht 18.5
# l 25.5
regions = {}
color = None
estado = 'Stop'

def lidar_callback(laser_data):
    global regions
    ranges = laser_data.ranges
    # print(type(ranges))
    # print(ranges)

    front_index = ranges[0:8] + ranges[352:360]
    # print(front_index)
    # print(min(front_index))

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

def color_callback(msg):
    global color
    color = msg.data

def stop():
    speed = 0
    steering = 90
    pSpeed.publish(speed)
    pSteering.publish(steering)

def turn_right():
    speed = 100
    steering = 45
    pSpeed.publish(speed)
    pSteering.publish(steering)

def turn_left():
    speed = 100
    steering = 135
    pSpeed.publish(speed)
    pSteering.publish(steering)

# def forwards():
#     speed = 200
#     steering = 90
#     pSpeed.publish(speed)
#     pSteering.publish(steering)

def backwards():
    speed = -20
    steering = 0
    pSpeed.publish(speed)
    pSteering.publish(steering)

def forwards():
    distancia_limite = 0.5  # Distancia de seguridad limite en metros
    velocidad_base = 200  # Velocidad lineal base del robot
    coeficiente_reduccion = .5  # Coeficiente de reduccion de velocidad

    # Obtener la distancia minima medida por el LiDAR
    distancia_minima = regions['front']

    # Calcular la velocidad lineal en funcion de la distancia
    velocidad_lineal = velocidad_base * (distancia_minima / distancia_limite) * coeficiente_reduccion
    if velocidad_lineal > velocidad_base:
        velocidad_lineal = velocidad_base

    speed = velocidad_lineal
    steering = 90
    pSpeed.publish(speed)
    pSteering.publish(steering)

def main():
    global estado
    # Parametros del vehiculo
    # L = .255 # Distancia entre los ejes delantero y trasero del vehiculo

    while not rospy.is_shutdown():
        
        # forwards()
        if len(regions) != 0 and color != None:
            if color == 'R' or regions["front"] <= .50:
                 estado = 'Stop'
            if color == 'G':
                estado = 'Avanzar'

            if estado == 'Avanzar':
                forwards()
            elif estado == 'Stop':
                stop()

        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('obstacles')

        # Configurar suscriptores para los topicos de la camara RealSense y el LIDAR RPLIDAR
        rospy.Subscriber('/scan', LaserScan, lidar_callback)
        rospy.Subscriber('/color_detected', String, color_callback)

        # Configurar publicadores para los topicos de velocidad y direccion
        pSpeed = rospy.Publisher('/speed', Int16, queue_size=10)
        pSteering = rospy.Publisher('/steering', UInt8, queue_size=10)

        rate = rospy.Rate(30)

        # Ejecutar la rutina de estacionamiento en paralelo
        main()

    except rospy.ROSInterruptException:
        pass
