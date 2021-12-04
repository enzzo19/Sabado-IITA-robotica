from controller import Robot
import cv2
from controller import DistanceSensor


robot = Robot()
timeStep = 32

# camera = robot.getDevice("camera1")
# camera.enable(timeStep)

distancia = robot.getDevice("distance sensor1")
distancia.enable(timeStep)
while robot.step(timeStep) != -1:
    sensor_value = distancia.getValue()
    print(sensor_value)
	