import math
from controller import Robot
from controller import Motor
from controller import PositionSensor
import numpy as np
import cv2 as cv
from controller import Robot, DistanceSensor, GPS, Camera, Receiver, Emitter
import random
import time
import struct

robot = Robot()  # Create robot object
# timeStep = numero de milisegundos entre actualizaciones mundiales (del mundo)
timeStep = 32
tile_size = 0.12  # Tamaño de casilla
angulo_actual = 0
tiempo_anterior = 0
media_baldoza = 0.06
speed = 6.28
global start
global finalLetter
global a
# Distance sensor initialization
distancia_sensor1 = robot.getDevice("distance sensor1")
distancia_sensor1.enable(timeStep)
maxima_distancia = 0.4

# Motor initialization
ruedaIzquierda = robot.getDevice("wheel1 motor")
ruedaDerecha = robot.getDevice("wheel2 motor")
ruedaIzquierda.setPosition(float('inf'))
ruedaDerecha.setPosition(float('inf'))

# Camera initialization
camera_centro = robot.getDevice('camera1')
camera_centro.enable(timeStep)
camera_der = robot.getDevice("camera2")
camera_der.enable(timeStep)
camera_izq = robot.getDevice("camera3")
camera_izq.enable(timeStep)

# Gyroscope initialization
gyro = robot.getDevice("gyro")
gyro.enable(timeStep)

#Gps initialization
gps = robot.getDevice("gps")
gps.enable(timeStep)
global posinicial
posinicial = gps.getValues()
print()
global x, y

while robot.step(timeStep) != -1:
    
    pos = gps.getValues()
    x = int(round((pos[2] - 0) * (-1) * (7 / 2.1) + 4))
    y = int(round((pos[0] - 0) * (-1) * (7 / 2.1) + 0.2))
    print([x, y])


