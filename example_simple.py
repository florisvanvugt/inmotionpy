
import time
import robot.interface as robot
#import robot.dummy as robot


robot.load()

print("Loaded")

v = robot.dump_shm()
print(v)

print(robot.rshm('fvv_trial_phase'))

print(robot.status())

for _ in range(5):
    x,y = robot.rshm('x'),robot.rshm('y')
    time.sleep(.1)
    print(x,y)

robot.move_to(.3,.1,2)

for _ in range(20):
    x,y = robot.rshm('x'),robot.rshm('y')
    time.sleep(.1)
    print(x,y)


robot.unload()

