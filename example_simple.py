
import time
import robot



robot.load()

print("Loaded")

print(robot.rshm('fvv_trial_phase'))

print(robot.status())

for _ in range(50):
    x,y = robot.rshm('x'),robot.rshm('y')
    time.sleep(.1)
    print(x,y)

robot.unload()

