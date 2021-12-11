from controller import Robot
import cv2
from controller import DistanceSensor


robot = Robot()
timeStep = 32

# camera = robot.getDevice("camera1")
# camera.enable(timeStep)

distancia_sensor1 = robot.getDevice("distance sensor1")
distancia_sensor1.enable(timeStep)
distancia_sensorDer = robot.getDevice("distance sensor2")
distancia_sensorDer.enable(timeStep)
distancia_sensorIzq = robot.getDevice("distance sensor2")
distancia_sensorIzq.enable(timeStep)
while robot.step(timeStep) != -1:
    sensor_value_centro = distancia_sensor1.getValue()
    print("Sensor centro",sensor_value_centro)
    sensor_value_Der = distancia_sensorDer.getValue()
    print("Sensor derecha",sensor_value_Der)
    sensor_value_Izq = distancia_sensorIzq.getValue()
    print("Sensor izquierda",sensor_value_Izq)
	