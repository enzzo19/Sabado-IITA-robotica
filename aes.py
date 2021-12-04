from controller import Robot

robot = Robot()
timeStep = 32

while robot.step(timeStep) != -1:    
    
    print(robot.getTime())