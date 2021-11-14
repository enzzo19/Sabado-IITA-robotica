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

# start para comparar
start = robot.getTime()

while robot.step(timeStep) != -1:

    speed1 = max_velocity
    speed2 = max_velocity
    
    
    wheel1.setVelocity(speed1)
    wheel2.setVelocity(speed2)

    # Para girar 90 grados debes cambiar el 1 por 0.36. 
    # 1 para 1 rotacion completa que equivale a 1 baldosa si avanzas, o casi 270 grados girando
    # 0.36 para 90 grados

    if robot.getTime() >= start + 1:
        print(start)
        print(robot.getTime())
        break
        
    

wheel1.setVelocity(0)
wheel2.setVelocity(0)