import math
from controller import Robot
from controller import Motor
from controller import PositionSensor
from controller import Robot, DistanceSensor, GPS, Camera

robot = Robot() # Create robot object
timeStep = 32   # timeStep = numero de milisegundos entre actualizaciones mundiales (del mundo)
tile_size = 0.12 # Tamaño de casilla
angulo_actual = 0
tiempo_anterior = 0
media_baldoza = 0.06

# Distance sensor initialization
distancia_sensor1 = robot.getDevice("distance sensor1")
distancia_sensor1.enable(timeStep)
maxima_distancia = 0.4

# Motor initialization
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

# Camera initialization
colorSensor = robot.getDevice("colour_sensor")
colorSensor.enable(timeStep)

def avanzar(vel):
    ruedaIzquierda.setVelocity(vel)
    ruedaDerecha.setVelocity(vel)

def girar(vel):
    ruedaIzquierda.setVelocity(-vel)
    ruedaDerecha.setVelocity(vel)

def rotar(angulo):
    global angulo_actual
    global tiempo_anterior
    #  iniciar_rotacion
    girar(0.5)
    # Mientras no llego al angulo solicitado sigo girando
    if (abs(angulo - angulo_actual) > 1):
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
        return False
    print("Rotacion finalizada.")
    angulo_actual = 0
    return True

estado = 1
while robot.step(timeStep) != -1:
    # Estado 1
    if estado == 1:
        image = colorSensor.getImage()
        r = colorSensor.imageGetRed(image, 1, 0, 0)
        g = colorSensor.imageGetGreen(image, 1, 0, 0)
        b = colorSensor.imageGetBlue(image, 1, 0, 0)
        if (r >= 24) or (g >= 44) or (b >= 49):
            print("Entramos en la arena")
            estado = 2
            print("r: " + str(r) + " g: " + str(g) + " b: " + str(b))
        elif (r <= 19) and (g <= 34) and (b <= 34):
            print("Casi caemos al pozo!")
            avanzar(0)
        else:
            estado == 4

    # Estado 2
    if estado == 2:
        if distancia_sensor1.getValue() > media_baldoza: # Lee los valores del sensor de distancia
            avanzar(2) # Si no encuentra nada a una distancia de 0.06, avanza
        else:
            avanzar(0) # Sino, frena y cambia de estado
            estado = 3
            print("Paso al estado:",estado)

    # Estado 3
    if estado == 3:
        if rotar(90) == True: # Como ya detectó algo en el estado 1, rota 90
            if distancia_sensor1.getValue() <= media_baldoza: # Lee si detecta algo.
                print("Valores del sensor de distancia:",distancia_sensor1.getValue())
                rotar(90) # Si si, rota de vuelta
            else:
                estado = 1 # Si no, vuelve al estado 1
                print("paso al estado:",estado)

    # Estado 4
    if estado == 4:
        if distancia_sensor1.getValue() > media_baldoza: # Lee los valores del sensor de distancia
            avanzar(6) # Si no encuentra nada a una distancia de 0.06, avanza
        else:
            avanzar(0) # Sino, frena y cambia de estado
            estado = 3
            print("Paso al estado:",estado)