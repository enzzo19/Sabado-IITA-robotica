from controller import Robot, DistanceSensor, GPS, Camera,receiver,display
from controller import PositionSensor
import numpy

robot = Robot() # Create robot object
timeStep = 32   # timeStep = numero de milisegundos entre actualizaciones mundiales (del mundo)
tile_size = 0.12 # Tamaño de casilla
media_baldoza = 0.06
tile = 0.12

# Emitter and Receiver initialization
receiver = robot.getDevice("receiver") # Retrieve the receiver and emitter by device name
emitter = robot.getDevice("emitter")
receiver.enable(timeStep)

# Display initialization
display = robot.getDevice("display")
display.enable(timeStep)
#Gps initialization
gps = robot.getDevice("gps")
gps.enable(timeStep)
robot.step(timeStep) # Actualizo los valores de los sensores
startX = gps.getValues()[0] # Cargo La posicion inicial
startY = gps.getValues()[2]
offset_xy = [0, 2]

# Distance sensor initialization
distancia_sensor1 = robot.getDevice("distance sensor1")
distancia_sensor1.enable(timeStep)

# Color sensor initialization
colorSensor = robot.getDevice("colour_sensor")
colorSensor.enable(timeStep)


class Grid():
    def __init__(self,grilla,x,y):
        self.grilla = grilla
        self.x = x
        self.y = y
    
    def define_grid_size(self):
        x = (gps.getValues()[0])
        y = (gps.getValues()[2])
        # Option 1
        receiver.getDataSize
        grilla = numpy.zeros(receiver)
        # Option 2
        w = display.getWidth()
        h = display.getHeight()
        grilla = numpy.zeros(w,h)
        # Option 3
        for i in tile:
            x = gps.getValues()[0]
            y = gps.getValues()[2]
            listx = []
            listy = []
            listx.append(x)
            listy.append(y)
        valx = max(listx)
        valy = max(listy)


    def identify_type(self):
        image = colorSensor.getImage()
        r = colorSensor.imageGetRed(image, 1, 0, 0)
        g = colorSensor.imageGetGreen(image, 1, 0, 0)
        b = colorSensor.imageGetBlue(image, 1, 0, 0)
        # Swamp
        if r >= 150 and r <= 119 :
            return 3
        # Hole
        if r <= 148:
            return 2
        # Checkpoint
        if r > 148 and r < 150:
            return 4
        # Common
        if r <= 220:
            return 0
        # Wall
        if distancia_sensor1.getValue() > media_baldoza:
            return 1
        # Victim
        Letter = 0
        finalLetter = Letter
        if Letter == 'H':
            return 'H'
        if Letter == 'U':
            return 'U'
        if Letter == 'H':
            return 'H'

    def prints_on_matrix(self,identify_type,grilla,thing):
        if identify_type == thing:
            x = gps.getValues()[0]
            y = gps.getValues()[2]
            tile_x = (x + numpy.sign(x) * tile_size/2) / tile_size
            tile_y = (y + numpy.sign(y) * tile_size/2) / tile_size
            # Imprimo el 1 en el lugar que corresponde teniendo en cuenta los valores anteriores
            grilla[tile_x,tile_y] = thing
            print(grilla)

"""
grilla = []
x = (gps.getValues()[0])
y = (gps.getValues()[2])
g = Grid(grilla,x,y)
g.prints_on_matrix(3)
g.prints_on_matrix(4)
g.prints_on_matrix(0)
g.prints_on_matrix(2)
g.prints_on_matrix('H')
g.prints_on_matrix('U')
g.prints_on_matrix('S')
"""