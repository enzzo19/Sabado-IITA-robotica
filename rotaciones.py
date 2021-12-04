from controller import Robot

timeStep = 32            # Set the time step for the simulation
max_velocity = 6.28      # Set a maximum velocity time constant
velocity_a = -6.28       # para girar



# Make robot controller instance
robot = Robot()

# Define the wheels 
wheel1 = robot.getDevice("wheel1 motor")   # Create an object to control the left wheel
wheel2 = robot.getDevice("wheel2 motor") # Create an object to control the right wheel

# Set the wheels to have infinite rotation 
wheel1.setPosition(float("inf"))       
wheel2.setPosition(float("inf"))

# Defino avanzar
def avanzar(rotaciones):
    rotacion = range(rotaciones)

    for i in rotacion:
        print(i)   
        tiempo_inicial = robot.getTime()
        while True:
            wheel1.setVelocity(max_velocity)
            wheel2.setVelocity(max_velocity)
            if robot.getTime() >= tiempo_inicial + 1:
                wheel1.setVelocity(0)
                wheel2.setVelocity(0)
                break

            
# start para comparar
start = robot.getTime()

while robot.step(timeStep) != -1:    
    
    avanzar(2)