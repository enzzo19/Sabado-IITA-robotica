from controller import Robot

timeStep = 32            # Set the time step for the simulation
max_velocity = 2.0      # Set a maximum velocity time constant
velocity_a = -6.28       # para girar

# Make robot controller instance
robot = Robot()

speed = 6.28
# Define the wheels 
wheel1 = robot.getDevice("wheel1 motor")   # Create an object to control the left wheel
wheel2 = robot.getDevice("wheel2 motor") # Create an object to control the right wheel

# Set the wheels to have infinite rotation 
wheel1.setPosition(float("inf"))       
wheel2.setPosition(float("inf"))

def giro(direccion):
    start = robot.getTime()
    if direccion == "derecha":
        wheel1.setVelocity(speed)
        wheel2.setVelocity(-speed)
        if robot.getTime() >= start + 0.36:
            print("dou")
            wheel1.setVelocity(0)
            wheel2.setVelocity(0)
            direccion = ""
        
    if direccion == "izquierda":
        wheel1.setVelocity(-speed)
        wheel2.setVelocity(speed)
        if robot.getTime() >= start + 0.36:
            print("uod")
            wheel1.setVelocity(0)
            wheel2.setVelocity(0)
            direccion = ""

# start para comparar

while robot.step(timeStep) != -1:

    giro("izquierda")
    # t_inicial = robot.getTime()
    # giro("izquierda",start)