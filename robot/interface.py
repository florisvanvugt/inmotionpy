## \mainpage InMotion2-Python
#
# This is for interacting with the InMotion2 robot using Python.
#
# We assume that the following controllers are set (controlled in `pl_uslot.c`)
#
# - controller 0  : null field
# - controller 2  : zero_ft
# - controller 4  : movetopt (move to location)
# - controller 5  : static_ctl_fade (hold and fade)
# - controller 9  : trajectory_reproduce (replay a trajectory loaded into memory)
# - controller 16 : static_ctl (hold at specified location)
#
#
#
# The main files here are:
# - `interface.py` - controls the interface with the robot C code, launching, etc.
# - `shm.py` - controls reading/writing to shared memory, which the robot C code will read
#



from __future__ import absolute_import


import os
import subprocess
import time
import sys



# Import code for interacting with the shared memory (this allows us to exchange information with the C script)
#from shm_ext import *
from .shm import *



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



def lkm_status():
    """ Try to find out whether the Linux kernel modules are loaded. """
    mods = str(subprocess.check_output('lsmod'))

    loaded = []
    if mods.find('xeno_')>-1: loaded.append("Xenomai")
    if mods.find('pwrdaq')>-1: loaded.append("PowerDAQ")

    if len(loaded)==0:
        return "none found"
    else:
        return "%s loaded"%(", ".join(loaded))

    


def probe_process():
    """ Try to find out if there is a robot process running. """
    try:
        rob = str(subprocess.check_output(['pkill','-0','-x','robot']))
        return True
    except subprocess.CalledProcessError: # pkill will return exit status 1 if there is no process found
        return False



def status():
    """ Return the status of the robot. """
    status = ""
    if rob==None:
        status += "robot launch process: none found, probably you have not run the robot from here\n"
    else:
        status += "robot launch process: exit code = %s\n"%(str(rob))

    status += "linux kernel modules: %s\n"%lkm_status()

    status += "robot process: %s"%("found" if probe_process() else "not found")

    status += "\n\n"
    status += "shm status: %s\n"%shm_status()

    if shm_connected():
        try:
            status += "robot: paused = %s\n"%str(rshm('paused'))
            status += "robot: controller %i\n"%get_controller()
            status += "robot position: %.03f,%.03f\n"%(rshm('x'),rshm('y'))
        except:
            status += "error reading from shared memory: try start_shm() ?\n"
        
    if len(errbuff)>0:
        status += "last errors:\n%s"%"\n".join(errbuff)
    return status
        


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

    time.sleep(.1)
    wshm("no_safety_check",0)    # Enable the safety zone

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
# Logging
#
#

savedatpid = None

def start_log(fname,n):
    """ Starts the log. 

    Arguments
    fname    : log file name
    n        : number of columns
    """

    # write log header
    #logheader(fname,n,headerf) # $logfile $num $uheaderfile
    wshm('nlog',0) # suspends the log writing

    global savedatpid
    if savedatpid!=None:
        print("Log already running! Not starting another log process.")
        return
        
    savedatpid = subprocess.Popen(['cat','/proc/xenomai/registry/pipes/crob_out'],stdout=open(fname,'wb'))

    wshm('nlog',n) # starts the log writing
    
    # Original: [exec cat < /proc/xenomai/registry/pipes/crob_out >> $logfile &]


def stop_log():
    """ Stops the log. """
    global savedatpid
    if savedatpid!=None:
        wshm('nlog',0)
        savedatpid.kill()
        savedatpid=None

    


def logheader(fname,n,headerfile=""):
    """ Write log file header.
    pad with commented dots to 4096 bytes of ascii stuff
    (or truncate)
    make sure this is ascii, multi-byte chars will be messy here.
    """
    subprocess.call(['robot/loghead',fname,n])
    #exec $ob(crobhome)/loghead $filename $ncols






#
#
# A bit more high-level
#
#


# Quite strong (production mode - use this when you are confident about your scripts)
stiffness = 4000. 
damping = 40.

# Testing strength
#stiffness = 800. 
#damping = 8.



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
    bias = rshm('ft_bias')
    print(bias)
    return bias
    
    

    

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
    wshm('plg_ftzero',[0]*6)

    # select the zero_ft controller
    controller(2)

    # run it for a little while
    time.sleep(1.)
    
    # select the null field controller
    controller(0)

    count = rshm('plg_ftzerocount')

    print("Averaging over %i ATI samples"%count)

    ftzero = rshm('plg_ftzero')
    wshm('ft_bias',[ z/float(count) for z in ftzero ])

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



def stay_fade(x,y):
    """
    Starts out as an attractor controller for the point (x,y) but
    gradually fades out the forces. This is supposed to not give subjects
    a sudden jolt when they gain back control of the robot.
    """
    wshm("fvv_force_fade",1.0) # This starts at 1.0 but exponentially decays to infinitely small
    wshm("plg_p1x",x)
    wshm("plg_p1y",y)
    wshm("plg_stiffness",stiffness)
    wshm("plg_damping",damping)
    controller(5)  # static_ctl_fade

    







TRAJECTORY_BUFFER_SIZE = 3000  # the maximum size of a replay trajectory (should correspond to #DEFINE TRAJECTORY_BUFFER_SIZE in robdecls.h)

# This defines the SQUARE of the allowable distance from the beginning of a trajectory.
# As a safety precaution, we check that we are close enough to the starting point of a trajectory before we will replay it. This value controls how close to it we need to be.
# NOTE: this is a squared number, and it will be compared to the SQUARED distance to the starting position.
REPLAY_START_SAFETY_PROXIMITY = .01**2


def capture_trajectory(duration=1.):
    """ 
    Captures a trajectory for the given duration (in sec)
    and returns it as a list of x,y pairs.
    """

    start_capture()
    t0=time.time()
    while time.time()<t0+duration:
        pass
    controller(0) # null field controller; setting this will stop the capture

    return retrieve_trajectory()
    


def start_capture():
    """
    Starts to capture the current movement trace 
    and put it in shared memory.
    """
    wshm('traj_count',0) # define that we are starting from the starting point
    controller(8)        # trajectory_capture
    




def retrieve_trajectory():
    """
    If you have called start_capture(),
    this retrieves the trajectory that was captured.
    """
    xs,ys=rshm('trajx'),rshm('trajy')
    n = rshm('traj_count')
    return zip(xs[:n],ys[:n])
    







def prepare_replay(trajectory):
    """
    Prepares the robot to replay a trajectory.

    Arguments
    trajectory : a list of pairs (x,y) of positions to be replayed
    """

    # Tell the robot how many samples this trajectory will be
    traj_n_samps = len(trajectory)
    # Caution: make sure the trajectory is not longer than the array size of ob->trajx and trajy
    if traj_n_samps>TRAJECTORY_BUFFER_SIZE:
        raise ValueError("This trajectory is too long: %i position samples > %i buffer length."%(traj_n_samps,TRAJECTORY_BUFFER_SIZE))
    wshm('traj_n_samps',traj_n_samps)
    
    # Extract the X and Y time course
    trajx,trajy = zip(*trajectory)

    # The final position of this trajectory
    lastx,lasty = trajx[-1],trajy[-1]
    
    # Top up the trajectory replicating its last value until it is TRAJECTORY_BUFFER_SIZE items long.
    # (These position samples should never be played, but it doesn't hurt to make sure the position
    # array has good integrity).
    topup = TRAJECTORY_BUFFER_SIZE-traj_n_samps
    trajx = list(trajx) + [lastx]*topup
    trajy = list(trajy) + [lasty]*topup

    assert len(trajx)==TRAJECTORY_BUFFER_SIZE
    assert len(trajy)==TRAJECTORY_BUFFER_SIZE

    # Youpi, we can now write the whole list with one statement, nice!
    wshm('trajx',trajx)
    wshm('trajy',trajy)

    wshm('traj_final_x',lastx)
    wshm('traj_final_y',lasty)

    # For real power
    #wshm('replay_damping',  40.0)
    #wshm('replay_stiffness',4000.0)

    # For debug
    wshm('replay_damping',  40.0)
    wshm('replay_stiffness',4000.0)

    return

    





def start_replay():
    """
    Starts replaying a trajectory.

    IMPORTANT: you need to have run prepare_trajectory() before.

    CAUTION: we really need the robot handle to already be at the starting point of the
    trajectory, otherwise you get a big jerking movement.
    """

    # Probably I should check using rshm('x') that we are
    # actually close enough to the starting position before I
    # allow replay to start.
    
    # Get current x,y position
    x,y = rshm('x'),rshm('y')

    # Get the first position of the replay trajectory
    firstx,firsty = rshm('trajx',0),rshm('trajy',0)

    # Compute how far we are from the starting position
    sqdist = (x-firstx)**2 + (y-firsty)**2
    if sqdist> REPLAY_START_SAFETY_PROXIMITY:
        print ("Refusing to replay trajectory because current starting position (%f,%f) is too far away from trajectory starting position (%f,%f) (distance^2=%f)"%(x,y,firstx,firsty,sqdist))
        return

    wshm('replay_done',0)
    wshm('traj_count',0) # define that we are starting from the starting point
    controller(9)        # trajectory_reproduce



    

def replay_is_done():
    """ Returns true when the replay is done."""
    return rshm('replay_done')==1
    



  




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










    

# Check the executables
for executable in [robot_start,robot_stop]:

    if not os.path.isfile(executable):
        print("ERROR: could not find executable %s (did you compile the robot code already?)"%executable)


    
