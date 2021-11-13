import math
from controller import Robot
from controller import Motor
from controller import PositionSensor
from controller import Robot, DistanceSensor, GPS
import numpy

# Crear una matriz de 5 x 7 rellena de 0
grilla = numpy.zeros((10,10))

robot = Robot() # Create robot object
timeStep = 32   # timeStep = numero de milisegundos entre actualizaciones mundiales (del mundo)
tile_size = 0.12 # Tamaño de casilla
angulo_actual = 0

# Distance sensor initialization
distancia_sensor1 = robot.getDevice("distance sensor1")
distancia_sensor1.enable(timeStep)
maxima_distancia = 0.4

#Motor initialization
ruedaIzquierda = robot.getDevice("wheel1 motor")
ruedaDerecha = robot.getDevice("wheel2 motor")
ruedaIzquierda.setPosition(float('inf'))
ruedaDerecha.setPosition(float('inf'))

# Gyroscope initialization
gyro = robot.getDevice("gyro")
gyro.enable(timeStep)

#Gps initialization
gps = robot.getDevice("gps")
gps.enable(timeStep)
robot.step(timeStep) # Actualizo los valores de los sensores
startX = gps.getValues()[0] # Cargo La posicion inicial
startY = gps.getValues()[2]
offset_xy = [0, 2]
x = 0
y = 0
# Defino el estado como valores de 0 al 7 siendo 0 el estado inicial
estado = 0


# Create your functions here
def avanzar(vel):
    ruedaIzquierda.setVelocity(vel)
    ruedaDerecha.setVelocity(vel)

def girar(vel):
    ruedaIzquierda.setVelocity(-vel)
    ruedaDerecha.setVelocity(vel)

def rotar(angulo):
    global angulo_actual
    tiempo_anterior = 0
    #  iniciar_rotacion
    girar(1.2)
    # Mientras no llego al angulo solicitado sigo girando
    while (abs(angulo - angulo_actual) > 1):
        tiempo_actual = robot.getTime()
        # print("Inicio rotacion angulo", angulo, "Angulo actual:",angulo_actual)
        tiempo_transcurrido = tiempo_actual - tiempo_anterior  # tiempo que paso en cada timestep
        radsIntimestep = abs(gyro.getValues()[1]) * tiempo_transcurrido   # rad/seg * mseg * 1000
        degsIntimestep = radsIntimestep * 180 / math.pi
        # print("rads: " + str(radsIntimestep) + " | degs: " + str(degsIntimestep))
        angulo_actual += degsIntimestep
        # Si se pasa de 360 grados se ajusta la rotacion empezando desde 0 grados
        angulo_actual = angulo_actual % 360
        # Si es mas bajo que 0 grados, le resta ese valor a 360
        if angulo_actual < 0:
            angulo_actual += 360
        tiempo_anterior = tiempo_actual
        robot.step(timeStep)
    print("Rotacion finalizada.")
    # angulo_actual = 0
    return True

# 1. El robot gira "angle" en forma bloqueante
# 2. Retorna:
#           True: Cuando llega a las coordenadas [x,y]
#           False: Cuando no llega a las coordenadas.
def MovimientoPa(angle,x0,y0,vel):
    margen = 0.02
    if rotar(angle):
        print("Rotacion de 90 terminada, me detengo")
        avanzar(0)
    if not ((x0-margen<= x <= x0+margen) and (y0-margen <= y <= y0+margen)):
        print("Avanzamos, pa")
        avanzar(vel)
        print("x:",x, "y:",y)
        # El robot no llego a las coordenadas x,y
        return False
    else:
        # El robot llego a coordenadas x,y
        return True

# Code here
estado = 0
avanzar(0)
while robot.step(timeStep) != -1:
    # Calculo los valores de X e Y
    x = gps.getValues()[0] - startX
    y = gps.getValues()[2] - startY
    #print("x:",round(x,1), "// y:",round(y,1))
# Etapa 0
    if estado == 0:
        # Etapa inicial: Gira 90 grados y avanza hasta la pared
        print("Inicio etapa 0")
        if MovimientoPa(90,0.13,0,6) == True:
            estado = 1
"""# Etapa 1
    elif estado == 1:
    #GIRO 2: Gira 90 grados y avanza hasta el fondo del mapa
        print("Inicio etapa 1")
        if MovimientoPa(90,0.14,0,6,90) == True:
            estado = 2"""

"""# Etapa 2
    elif estado == 2:
    #GIRO 3: Gira y avanza hasta la zona del pozo
        print("Inicio etapa 2")
        if MovimientoPa(180,0.7,0.2,6):
            estado = 2"""

    # #GIRO 4: Gira y avanza un poco, subiendo (que en realidad es bajando) por y hasta que llega a la última curva
    # print("Inicio etapa 4")
    # MovimientoPa(270,5.2,4.7,0.8,1.3,6)

    # #GIRO 5: Gira y avanza a la izquierda para llegar al último tramo
    # print("Inicio etapa 5")
    # MovimientoPa(270,3.2,2.7,0.8,1.3,6)

    # #GIRO 6: Gira y avanza hacia arriba por "y" para acomodarse en el último giro al pozo
    # print("Inicio etapa 6")
    # MovimientoPa(90,2.7,3.2,-0.8,-1.10,6)

    # #GIRO 7: Llega al pozo. Gira y avanza, luego cae.
    # print("Inicio etapa 7")
    # MovimientoPa(90,2.7,4.2,-2.8,-1.3,6)