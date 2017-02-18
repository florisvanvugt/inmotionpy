

"""

This is for interacting with the InMotion2 robot using Python.

"""


import os
import subprocess
import time
import fcntl
import sys


# Import code for interacting with the shared memory (this allows us to exchange information with the C script)
#from shm_ext import *
from shm import *



# Locations of the robot executables
robot_dir   = os.getcwd()+"/robot" # we assume that a subdirectory of our script contains the robot code.
robot_start = "%s/go"%robot_dir
robot_stop  = "%s/stop"%robot_dir




# This is an object where we set various data for quick access.
ob = {}


# This will contain the process we are using to communicate with the shared memory
shm = None

# This contains the robot process (normally we don't need to look at it anymore)
rob = None


# Error buffer: this is where we can drop errors when they occur
errbuff = []



def status():
    """ Return the status of the robot. """
    if rob==None:
        print("robot launch process: none found")
    else:
        print("robot launch process: exit code = %s"%(str(rob)))
        
        print("robot: paused = %s"%str(rshm('paused')))
        print("robot: controller %i"%get_controller())
        print("robot position: %.03f,%.03f"%(rshm('x'),rshm('y')))

    if len(errbuff)>0:
        print("last errors:\n%s"%"\n".join(errbuff))



def load():
    """ 
    Starts up the robot. Loads the robot C program, starts our shared memory (shm) connection
    with it, and starts the robot main loop.
    """

    print("Loading robot...")
    start_lkm()
    start_shm()
    put_init_calib()
    start_loop()

    # Not sure if this does anything, but just to make sure
    wshm("plg_last_fX",0.0)
    wshm("plg_last_fY",0.0)

    # Remove the safety zone so that the robot arm can move everywhere - careful!
    wshm("no_safety_check",1)

    print("done")
    return



def unload():
    """ Unloads the robot. """
    controller(0)
    print("Unloading robot...")

    # b_pause_proc $w
    # pause pauses actuator output, stop stops all main loop i/o.
    stop_loop()
    stop_shm()
    stop_lkm()

    #set ob(loaded) 0
    print("done")
    
    return



def put_init_calib():
    """ 
    This function basically puts all the initial calibration stuff 
    to the robot when we first launch it. The settings come from robot/imt2.cal
    TODO: This can of course be done much nicer, without having to go through the 
    calibration syntax.
    """
    f = open('robot/imt2.cal','r')
    lns = f.readlines()
    f.close()

    for l in lns:
        l = l.strip()
        if len(l)>0 and l[0]!="#":
            if l=="ok":
                continue
            fs = [ v.strip() for v in l.split() ]
            if fs[0]=="s":
                # If this is a set command
                if get_info(fs[1])==None:
                    print("WARNING: ignoring config setting %s"%l)
                else:
                    #print(fs)
                    if len(fs)==3: # "simple" variable set
                        #print(fs[1],tryenc(fs[2]))
                        wshm(fs[1],tryenc(fs[2]))
                    elif len(fs)==4: # array item set
                        #print(fs[1],fs[3],fs[2])
                        wshm(fs[1],tryenc(fs[3]),int(fs[2]))
                    else:
                        print("Not sure what to do with calibration line: %s"%l)

            else:
                print("Not sure what to do with calibration line: %s"%l)
                



def start_lkm():
    """ Starts the robot process. """
    global rob
    rob = subprocess.call(robot_start,cwd=robot_dir)

    time.sleep(.1) # Wait for everything to start

    #subprocess.call(robot_start,cwd="./robot/") #,cwd=robot_dir)
    #subprocess.call(robot_start) #,cwd=robot_dir)
    # TODO: catch result from starting the robot

    # TODO: figure out if we want to do something like goose_process, i.e.
    #subprocess.call(["sudo","/usr/bin/renice","-10","[pid]"])
    #goose_process


def stop_lkm():
    """ Stops the robot process (assuming one is running) """
    subprocess.call(robot_stop,cwd=robot_dir)

    


def get_controller():
    """ Return which controller is currently running. """
    return rshm("slot_fnid")
        



def controller(n):
    """ 
    Select the n-th controller on the robot. 
    
    Arguments
    n : the id of the controller to be selected
    """

    # Don't ask me why...
    slot_id   = 0
    slot_fnid = n

    tslot_go = rshm('slot_go')
    if tslot_go>0: # if we are currently changing slots...?
        time.sleep(.01)
	# reschedule myself in 10 ms
	# puts "movebox2, rescheduling..."
	# dtime "movebox: resched"

    wshm("slot_id",slot_id)
    wshm("slot_fnid",slot_fnid)
    wshm("slot_running",1)
    wshm("slot_go",1)


    


def start_loop():
    # TODO: check if this makes sense
    wshm("paused",0)
    wshm("slot_max",4)


def pause_loop():
    # this pauses the main loop if you want to restart it.
    # sensors are read.  actuators are not written.
    wshm("paused",1)


def stop_loop():
    # this stops the main loop and must be called before stop_lkm
    # paused is set first to zero motor command voltages
    # to receive and process the commands.  (100 ms is 20 ticks at 200 Hz.)
    wshm("paused",1)
    time.sleep(.1)

    wshm('quit',1)
    time.sleep(2) # wait for the robot control loop to shut itself down

    return





#
#
# A bit more high-level
#
#


# Quite strong
#stiffness = 4000. 
#damping = 40.

stiffness = 800. 
damping = 8.



def bias_force_transducers():
    """ 
    A little wrapper script to remove the bias of the force transducers.
    This function asks the user not to hold the handle and then reads
    the force transducer output.
    """
    print("")
    print("*** biasing force transducers: please let go of the handle")
    print("*** and hit ENTER when ready to start zeroing procedure")
    input()

    zeroft()
   
    print(" ")
    print("*** zeroing the force transducers is done, please hold the handle now")
    print("*** and hit ENTER when ready to continue")
    input()

    


def bias_report():
    print("Robot ATI bias summary")
    for i in range(6):
        print("%i -> %f"%(i,rshm('ft_bias',i)))

    

def zeroft():
    """ 
    Samples the force torque transducer values and zeroes them.
    It is important to run this when there are no forces on the handle (i.e. subject is not
    holding it).

    Gribble's comments:
    # use this instead of ft_bias, which only reads one sample
    # plg_zeroft samples for 100 samples and takes the mean
    """
    
    # initialize some shared memory variables to zero
    wshm('plg_ftzerocount',0)
    for i in range(6):
        wshm('plg_ftzero',0,i)

    #for {set i 0} {$i < 6} {incr i} {
    #wshm plg_ftzero 0 $i
    #}
    
    # select the zero_ft controller
    controller(2)

    # run it for a little while
    time.sleep(1.)
    
    # select the null field controller
    controller(0)

    count = rshm('plg_ftzerocount')

    print("Averaging over %i ATI samples"%count)

    for i in range(6):
        cal = rshm('plg_ftzero',i)/float(count)
        wshm('ft_bias',cal,i)

    bias_report()
    



def stay():
    """ 
    Fix the robot handle at the current location. 
    """
    x,y = rshm('x'),rshm('y')
    stay_at(x,y)



def stay_at(x,y):
    """
    Fix the robot handle at location x,y.
    CAUTION: we should be at x,y already otherwise we will snap to it!
    """
    wshm('plg_p1x',x)
    wshm('plg_p1y',y)
    wshm('plg_stiffness',stiffness)
    wshm('plg_damping',damping)
    controller(16) # static_ctl
    return


def move_to(x,y,t):
    """
    Move the robot handle to location (x,y) in t seconds.
    The function returns when the controller is started,
    and hence generally before the robot reaches the end location.
    """
    wshm('plg_movetime',0.0)
    wshm('plg_p2x',x)
    wshm('plg_p2y',y)
    wshm('plg_movetime',float(t))
    wshm('plg_stiffness',stiffness)
    wshm('plg_damping',damping)
    wshm('plg_moveto_done',0)
    wshm('plg_p1x',rshm('x'))
    wshm('plg_p1y',rshm('y'))
    wshm('plg_counterstart',rshm('i')) # record the current sample
    controller(4)
    return


def move_is_done():
    """ Returns whether a move is done. This assumes that a move has been started using moveto(). """
    return (rshm('plg_moveto_done')==1)



def move_stay(x,y,t):
    """ 
    Moves the robot handle to location (x,y) in t seconds and then holds it there.
    This function returns when the robot handle reaches the target location.
    """
    move_to(x,y,t)
    while not move_is_done():
        pass
    stay_at(x,y) # when the move is done



# Check the executables
for executable in [robot_start,robot_stop]:

    if not os.path.isfile(executable):
        print("ERROR: could not find executable %s (did you compile the robot code already?)"%executable)


    




def error_buffer(err):
    """ Write a particular error to the buffer. """
    global errbuff
    print("-- ERROR -- "+err)
    errbuff.append(err)
    






    

def tryenc(r):
    """ Try to guess the encoding of this item."""
    
    # Try if this is an integer
    try:
        return int(r)
    except ValueError:
        pass

    # Try if this is a float
    try:
        return float(r)
    except ValueError:
        pass

    return r

