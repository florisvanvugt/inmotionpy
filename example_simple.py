
import time
import robot.interface as robot



robot.load()

print("Loaded")

print(robot.rshm('fvv_trial_phase'))

print(robot.status())

showvars = ["x","y","shoulder_angle_degrees","elbow_angle_degrees"]
print(" ".join(showvars))

for _ in range(200):
    
    vals = [ robot.rshm(v) for v in showvars ]
    print(" ".join([ str(v) for v in vals ]))
    
    time.sleep(.1)
    #print(x,y)

robot.unload()

