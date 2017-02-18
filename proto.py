

import robot


robot.load()
robot.zeroft()

def showpos():
    print(robot.rshm('x'),robot.rshm('y'))

print("Position:")
showpos()

print("Now I will tell the robot to stay fixed at this position")
input()

print("Staying")
robot.stay()

input()

print("Position:")
showpos()

x,y = robot.rshm('x'),robot.rshm('y')
tx,ty = x+.1,y

print("About to move to %f,%f, okay?"%(tx,ty))

input()
robot.move_to(tx,ty,3.)

input()

robot.unload()








"""

sandbox

import robot

shm = robot.spawn_process('robot/shm') # assume a robot process is already running

robot.send(shm,'h')

robot.read(shm)


"""


