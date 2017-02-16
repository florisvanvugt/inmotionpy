

import robot


robot.start_lkm()
robot.start_shm()


print("Okidoki")


robot.stop_shm()
robot.stop_lkm()






"""

sandbox

import robot

shm = robot.spawn_process('robot/shm') # assume a robot process is already running

robot.send(shm,'h')

robot.read(shm)


"""


