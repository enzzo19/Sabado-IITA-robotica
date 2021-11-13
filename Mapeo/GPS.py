from controller import Robot, GPS
import math

robot = Robot()

gps = robot.getDevice("gps")

timestep = 32
tilesize = 0.06
gps.enable(timestep)

robot.step(timestep) # Actualizo los valores de los sensores
startX = gps.getValues()[0]/tilesize # Cargo La posicion inicial
startY = gps.getValues()[2]/tilesize

while robot.step(timestep) != -1:
    x = gps.getValues()[0] - startX
    y = gps.getValues()[2] - startY
    print("Imprimo la posicion actual x:",round(x,1),"y:",round(y,1))