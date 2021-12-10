import math
from controller import Robot
from controller import Motor
from controller import PositionSensor
import numpy as np
import cv2 as cv
from controller import Robot, DistanceSensor, GPS, Camera
import random
import time

robot = Robot()
timeStep = 32


#Gps initialization
gps = robot.getDevice("gps")
gps.enable(timeStep)
robot.step(timeStep) # Actualizo los valores de los sensores
startX = gps.getValues()[0] # Cargo La posicion inicial
startY = gps.getValues()[2]
offset_xy = [0, 2]
x = 0
y = 0



while robot.step(timeStep) != -1:
    x = int(gps.getValues()[0]*100)
    y = int(gps.getValues()[2]*100)
    print(x,y)
