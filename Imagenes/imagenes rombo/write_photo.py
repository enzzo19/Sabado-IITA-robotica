from controller import Robot
import cv2
import numpy as np


robot = Robot()
timeStep = 32
camera = robot.getDevice("camera_centre")
camera.enable(timeStep)

while robot.step(timeStep) != -1:
    img = camera.getImage()
    img = np.array(np.frombuffer(img, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4)))
    cv2.imwrite("C:/Users/JOAKOL/Desktop/robotica/Siempre Repositorio/Sabado-IITA-robotica/Imagenes/imagenes rombo/rombo_.png" , img)
    cv2.imshow("Image", img)
    cv2.waitKey(1)