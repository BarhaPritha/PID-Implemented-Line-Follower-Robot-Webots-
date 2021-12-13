# CSE461 Sec-4 Group-6
# Lab Final

from controller import Robot
from controller import Camera
from controller import DistanceSensor
from controller import Motor

# create the Robot instance.
robot = Robot()

time_step = 64

# enabling camera
cm = robot.getDevice("cam")
cm.enable(time_step)  
cm.recognitionEnable(time_step)

# enabling ir sensors
ds = []
dsNames = ['ds_left', 'ds_right']
for i in range(2):
    ds.append(robot.getDevice(dsNames[i]))
    ds[i].enable(time_step)
    
# initializing motor wheels    
wheels = []
wheelsNames = ['wheel1', 'wheel2']
for i in range(2):
    wheels.append(robot.getDevice(wheelsNames[i]))
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)
 
# PID Implementation    
last_error = intg = diff = prop = waitcounter = 0

kp = 1.5
ki = 0.0
kd = 1.5

def pid(error):
    global last_error, intg, diff, prop, kp, ki, kd
    prop = error
    intg = error + intg
    diff = error - last_error
    balance = (kp*prop) + (ki*intg) + (kd*diff)
    last_error = error
    return balance 
    
def setSpeed(base_speed,balance):
    wheels[0].setVelocity(-base_speed+balance)
    wheels[1].setVelocity(-base_speed-balance)

# Simulation
while robot.step(time_step) != -1:

    # Read the ir sensors:
    l_val = ds[0].getValue()
    r_val = ds[1].getValue()
    
    print("left: {}, right: {}".format(l_val,r_val), end=", ")
    
    if l_val<400 and r_val<400:
        error = 0
        setSpeed(4, pid(error))
        print("Move Forward")
        
    elif l_val>=400 and r_val>=400:
        error = 0
        setSpeed(2, pid(error))
        print("Slow Down")
         
    elif l_val<400 and r_val>=400:
        error = -2
        setSpeed(4, pid(error))
        print("Turn Right")
        
    elif l_val>=400 and r_val<400:
        error = 2
        setSpeed(4, pid(error))
        print("Turn Left")

    pass