
import time
import robot.interface as robot



robot.load()

print("Loaded")

v = robot.dump_shm()
print(v)

print(robot.rshm('fvv_trial_phase'))

print(robot.status())

for _ in range(50):
    x,y = robot.rshm('x'),robot.rshm('y')
    time.sleep(.1)
    print(x,y)

robot.unload()

