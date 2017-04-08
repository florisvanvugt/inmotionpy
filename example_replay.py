
import time
import robot.interface as robot

import numpy as np


robot.load()

robot.zeroft()

print(robot.status())

raw_input("Press <ENTER> to start capturing trajectory")
traj = robot.capture_trajectory(3)
print("Capture done.")


firstx,firsty = traj[0]
print("Moving to starting point %f,%f"%(firstx,firsty))
raw_input("Press <ENTER> to start")

robot.move_to(firstx,firsty,3.)
while not robot.move_is_done():
    pass
robot.stay_at(firstx,firsty)


robot.prepare_replay(traj)

print("Ready to start replaying trajectory.")
raw_input("Press <ENTER> to start")

robot.start_replay()

while not robot.replay_is_done():
    pass

print("Replay is done.")


robot.unload()
