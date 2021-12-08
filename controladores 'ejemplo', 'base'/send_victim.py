from controller import Robot
from controller import GPS
from controller import Emitter
import struct # Use the struct module in order to pack the message sent to the Main Supervisor

robot = Robot()
timestep = 32


# Cargamos el GPS
gps = robot.getDevice("gps")
gps.enable(timestep)

# Inicializo el emisor
emitter = robot.getDevice("emitter")


victimType = bytes('U', "utf-8") # The victim type being sent is the letter 'H' for harmed victim

send_time=robot.getTime() + 15

while robot.step(timestep) != -1:

    x = gps.getValues()[0]
    y = gps.getValues()[2]
    print(f'{x*100} , {y*100} ,c {victimType}')
    if robot.getTime() > send_time:
        message = struct.pack("i i c", int(x*100) , int(y*100) , victimType)
        emitter.send(message) # Send out the message
        send_time=robot.getTime() + 15

