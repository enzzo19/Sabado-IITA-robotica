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

all = True
if all == True:
# VARIABLES
    robot = Robot() # Create robot object
    timeStep = 32   # timeStep = numero de milisegundos entre actualizaciones mundiales (del mundo)
    tile_size = 0.12 # Tamaño de casilla
    angulo_actual = 0
    tiempo_anterior = 0
    media_baldoza = 0.06
    speed = 6.28
    global start
    global finalLetter

# Distance sensor initialization
    distancia_sensor1 = robot.getDevice("distance sensor1")
    distancia_sensor1.enable(timeStep)
    distancia_sensorDer = robot.getDevice("distance sensor2")
    distancia_sensorDer.enable(timeStep)
    distancia_sensorIzq = robot.getDevice("distance sensor3")
    distancia_sensorIzq.enable(timeStep)
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

# Gps initialization
    gps = robot.getDevice("gps")
    gps.enable(timeStep)
    robot.step(timeStep) # Actualizo los valores de los sensores
    startX = gps.getValues()[0] # Cargo La posicion inicial
    startY = gps.getValues()[2]
    offset_xy = [0, 2]
    x = 0
    y = 0

# Color sensor initialization
    colorSensor = robot.getDevice("colour_sensor")
    colorSensor.enable(timeStep)

# Emitter and Receiver initialization
    receiver = robot.getDevice("receiver") # Retrieve the receiver and emitter by device name
    emitter = robot.getDevice("emitter")
    receiver.enable(timeStep)

# Message Sending
    def SendMessage(posx, posy, identifier):
        victimType = bytes("H","utf-8")
        message = struct.pack("i i c", posx * 100, posy * 100, victimType)
        print("I have just sent the message")
        emitter.send(message)

# Util Functions
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

    def rotar_b(angulo):
        global angulo_actual
        tiempo_anterior = 0
        #  iniciar_rotacion
        girar(0.5)
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
        # angulo_actual = 0
        return True

    def classifyVictim(img):
        finalLetter = ' '
        # cv.imshow("imagen normal", img)
        img = cv.resize(img, (100, 100))
        # cv.imshow("imagen redimensionada", img)
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        #cv.imshow("imagen escala grises", gray)
        thresh1 = cv.threshold(gray, 100, 255, cv.THRESH_BINARY_INV)[1]
        # cv.imshow("imagen tresh", thresh1)
        conts, h = cv.findContours(thresh1, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        x, y, w, h = cv.boundingRect(conts[1])
        # print(thresh1.shape[0])
        cv.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), thickness=1)
        # cv.imshow("Rectangle", img)
        letter = thresh1[y:y + h, x:x + w]
        letter = cv.resize(letter, (100, 100), interpolation=cv.INTER_AREA)
        #letterColor = cv.cvtColor(letter, cv.COLOR_GRAY2BGR)
        areaWidth = 20
        areaHeight = 30
        areas = {
            "top": ((0, areaHeight), (50 - areaWidth // 2, 50 + areaWidth // 2)),
            "middle": ((50 - areaHeight // 2, 50 + areaHeight // 2), (50 - areaWidth // 2, 50 + areaWidth // 2)),
            "bottom": ((100 - areaHeight, 100), (50 - areaWidth // 2, 50 + areaWidth // 2))
        }
        images = {
            "top": letter[areas["top"][0][0]:areas["top"][0][1], areas["top"][1][0]:areas["top"][1][1]],
            "middle": letter[areas["middle"][0][0]:areas["middle"][0][1], areas["middle"][1][0]:areas["middle"][1][1]],
            "bottom": letter[areas["bottom"][0][0]:areas["bottom"][0][1], areas["bottom"][1][0]:areas["bottom"][1][1]]
        }
        #cv.rectangle(letterColor,(areas["top"][1][0], areas["top"][0][0]), (areas["top"][1][1], areas["top"][0][1]), (0, 255, 0), 1)
        #cv.rectangle(letterColor, (areas["middle"][1][0], areas["middle"][0][0]), (areas["middle"][1][1], areas["middle"][0][1]), (0, 0, 255), 1)
        #cv.rectangle(letterColor,(areas["bottom"][1][0], areas["bottom"][0][0]), (areas["bottom"][1][1], areas["bottom"][0][1]), (225, 0, 255), 1)
        counts = {}
        acceptanceThreshold = 50
        for key in images.keys():
            count = 0
            for row in images[key]:
                for pixel in row:
                    if pixel == 255:
                        count += 1
            counts[key] = count > acceptanceThreshold
        letters = {
            "H": {'top': False, 'middle': True, 'bottom': False},
            "S": {'top': True, 'middle': True, 'bottom': True},
            "U": {'top': False, 'middle': False, 'bottom': True}
        }
        for letterKey in letters.keys():
            if counts == letters[letterKey]:
                finalLetter = letterKey
                break
        return finalLetter

    def detectVisualSimple(image_data, camera):
        coords_list = []
        img = np.array(np.frombuffer(image_data, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4)))
        img[:,:,2] = np.zeros([img.shape[0], img.shape[1]])
        #convert from BGR to HSV color space
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        #apply threshold
        thresh = cv.threshold(gray, 140, 255, cv.THRESH_BINARY)[1]
        # draw all contours in green and accepted ones in red
        contours, h = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        for c in contours:
            if cv.contourArea(c) > 1000:
                coords = list(c[0][0])
                coords_list.append(coords)
                return ((int(coords[0])),int(coords[1]))

    def type_floor():
        image = colorSensor.getImage()
        r = colorSensor.imageGetRed(image, 1, 0, 0)
        g = colorSensor.imageGetGreen(image, 1, 0, 0)
        b = colorSensor.imageGetBlue(image, 1, 0, 0)
        # print("r: " + str(r) + " g: " + str(g) + " b: " + str(b))
        if r >= 150 and r <= 119 :
            # print("Entramos en la arena")
            return 'arena'
            # print("r: " + str(r) + " g: " + str(g) + " b: " + str(b))
        if r <= 148:
            return 'pozo'
        if r > 148 and r < 150:
            return 'checkpoint'
        if r <= 220:
            return 'common'

    def speeds_per_floor(floor,speed):
        if floor in ['arena', 'pozo']:
            ruedaIzquierda.setVelocity(speed*0.3)
            ruedaDerecha.setVelocity(speed*0.3)
        if floor in ['common', 'checkpoint']:
            ruedaIzquierda.setVelocity(speed)
            ruedaDerecha.setVelocity(speed)

    estado = 'avanzar_libre'

# MAIN CODE
    while robot.step(timeStep) != -1:
 # CONFIGURACIONES DEL SENSOR DE COLOR
        image = colorSensor.getImage()
        r = colorSensor.imageGetRed(image, 1, 0, 0)
        g = colorSensor.imageGetGreen(image, 1, 0, 0)
        b = colorSensor.imageGetBlue(image, 1, 0, 0)
        floor = type_floor()
        print('Floor:',floor)
    
 # FUNCIONES DEL PISO
        if floor == 'arena' and estado != 'retroceso' and estado != 'girito':
            estado = 'arena'

        if floor == 'pozo' and estado != 'girito_victima':
            start = robot.getTime()
            estado = 'retroceso'
        
 # ESTADO ARENA
        if estado == 'arena':
            print("estado arena")
            if floor == 'pozo':
                # print("Pozo")
                start = robot.getTime()
                estado = 'retroceso'
            if floor == 'common':
                # print('baldoza comun')
                start = robot.getTime()
                estado = 'avanzar_libre'    
            if distancia_sensor1.getValue() > media_baldoza:  # Lee los valores del sensor de distancia
                speeds_per_floor(floor,speed) # Si no encuentra nada a una distancia de 0.06, avanza
                img_der = camera_der.getImage()
                img_izq = camera_izq.getImage()
                deteccion_izq = detectVisualSimple(img_izq, camera_izq)
                deteccion_der = detectVisualSimple(img_der, camera_der)
                if (deteccion_izq != None) or (deteccion_der != None):
                    estado = 'deteccion'
            else:
                avanzar(0) # Sino, frena y cambia de estado
                estado = 'girito'
                # print("Paso al estado:",estado)

 # ESTADO RETROCESO
        if estado == 'retroceso':
            print("estado retroceso")
            speeds_per_floor(floor,-speed)
            if robot.getTime() >= start + 0.55:
                # print(start)
                # print(robot.getTime())
                estado = 'girito'

 # ESTADO GIRITO 
        if estado == 'girito':
            print("estado girito")
            
            if distancia_sensorDer.getValue()  > 0.5:
                print("voy a girar 90 grados")
                if rotar(90) == True:
                    print("rotacion finalizada pa")
                    estado = 'avanzar_libre'
            else: rotar(270)
            '''b = 0
            if b == 0:
                print("b =", b)
                if distancia_sensorDer.getValue() > 0.06:
                    print("voy a girar 90 grados")
                    if rotar(90) == True:
                        estado = 'avanzar_libre'
                else: b = 1
            if b == 1:
                print("b =", b)
                if distancia_sensorIzq.getValue() > 0.06:
                    print("voy a girar 270 grados")
                    if rotar(270) == True:
                        estado = 'avanzar_libre'''
        #distancia_sensorDer.getValue() > media_baldoza 
 # ESTADO GIRITO VICTIMA
        if estado == 'girito_victima':
            print("estado girito_victima")
            img_centro = camera_centro.getImage()
            deteccion_centro = detectVisualSimple(img_centro, camera_centro)
            if rotar(270) == True:
                print(deteccion_centro)
                if deteccion_centro == None:
                    angule = random.choice([90, 270])
                    rotar(angule) 
                else:
                    a = 0
                    start = robot.getTime()
                    estado = 'clasificacion' 

 # ESTADO AVANZAR LIBRE
        if estado == 'avanzar_libre':
            print("estado avanzar_libre")
                
            if distancia_sensor1.getValue() > media_baldoza: # Lee los valores del sensor de distancia
                start = robot.getTime()
                if robot.getTime() <= start + 1:
                    speeds_per_floor('arena', speed)
                    # print("ESTOY ESPERANDO")
                else:
                    speeds_per_floor(floor, speed)
                img_der = camera_der.getImage()
                img_izq = camera_izq.getImage()
                deteccion_izq = detectVisualSimple(img_izq, camera_izq)
                deteccion_der = detectVisualSimple(img_der, camera_der)
                if (deteccion_izq != None) or (deteccion_der != None):
                    estado = 'deteccion'

            else:
                avanzar(0) # Sino, frena y cambia de estado
                estado = 'girito'
                # print("Paso al estado:",estado)

 # ESTADO DETECCIÓN DE VÍCTIMA
        if estado == 'deteccion':
            print('Estado deteccion')
            avanzar(0.3)
            img_der = camera_der.getImage()
            img_izq = camera_izq.getImage()
            deteccion_izq = detectVisualSimple(img_izq, camera_izq)
            deteccion_der = detectVisualSimple(img_der, camera_der)
            print(f'{deteccion_izq}, {deteccion_der}')
            if deteccion_izq[0] < 20:
                estado = 'girito_victima'
                print("CAMBIO AL ESTADO girito_victima")
            else:
                print(f'{deteccion_izq}, {deteccion_der}')
        
 # ESTADO CLASIFICACIÓN DE VÍCTIMA
        if estado == 'clasificacion':
            print("Estado clasificacion")
            avanzar(0)
            if robot.getTime() >= start + 1:
                if a == 0:
                    # Robot catches cetre camera image and classifies it
                    img = camera_centro.getImage()
                    img = np.array(np.frombuffer(img, np.uint8).reshape((camera_centro.getHeight(), camera_centro.getWidth(), 4)))
                    identifier = classifyVictim(img)
                    print(identifier)
                    # Prints the message necessary to score points considering the GPS POSITION and the TYPE OF VICTIM
                    victimType = bytes(identifier, "utf-8") # The victim type being sent is the letter 'H' for harmed victim
                    send_time=robot.getTime() + 2
                    x = gps.getValues()[0]
                    y = gps.getValues()[2]
                    message = struct.pack("i i c", int(x*100) , int(y*100) , victimType)
                    emitter.send(message)
                    a = 1
            if robot.getTime() >= start + 3:
                estado = 'girito'