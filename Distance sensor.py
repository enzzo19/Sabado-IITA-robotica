from controller import Robot

robot = Robot()
timeStep = 32
sensor = robot.getDevice("ps1")
sensor.enable(timeStep)

while robot.step(timeStep) != -1:
    value = sensor.getValue()
    print(value)