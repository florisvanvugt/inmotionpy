
import time
import robot.interface as robot


robot.load()

robot.start_log('log.txt',13)

time.sleep(.1)

robot.stop_log()

robot.unload()
