

"""

This is for interacting with the InMotion2 robot using Python.

"""


import os
import subprocess
import time
import fcntl
import sys





# Locations of the robot executables
robot_dir   = os.getcwd()+"/robot" # we assume that a subdirectory of our script contains the robot code.
robot_start = "%s/go"%robot_dir
robot_stop  = "%s/stop"%robot_dir

shm_start   = "%s/shm"%robot_dir



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
        
        if shm==None:
            print("shm: no connection")
        else:
            print("shm: process available")
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
    # TODO: Make sure the robot is not already loaded
    start_lkm()
    start_shm()
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

    
def start_shm():
    global shm
    shm = spawn_process(shm_start,robot_dir)
    # TODO: set buffering to line end
    
    time.sleep(.1) # Wait for everything to start

    # TODO: perform this check
    check = rshm('last_shm_val')
    if check!=12345678:
        error_buffer("Error: rshm check returned %s instead of 1234578.\nMake sure all software has been compiled with latest cmdlist.tcl"%(str(check)))
        sys.exit(-1)





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


    

    
def rshm(variable,index=0):
    """ Read a variable from the shared memory and return its value. """

    global shm
    # TODO: check that the shm is running

    # This is the query string we will send to the shm script
    query = "g %s %i"%(variable,index)
    send(shm,query)

    # Get the response to our query
    reply = read(shm)
    resp = reply.strip().split(' ')

    # Parse it to get the output we need
    if resp[0]=="?":
        # An error was generated
        error_buffer("Error in rshm('%s',%i): '%s'"%(variable,index,reply))
        return None


    # Try if this is an integer
    try:
        return int(resp[3])
    except ValueError:
        pass

    # Try if this is a float
    try:
        return float(resp[3])
    except ValueError:
        pass
    
    return resp[3]  # Give up and return it as-is (string)
    



def wshm(variable,value,index=0):
    """ Writes the given value to the given variable in the shared memory. """
    
    # TODO: check if the shm process exists
    query = "s %s %i %s"%(variable,index,str(value))
    send(shm,query)

    # Get the response to our query
    reply = read(shm)
    resp = reply.strip().split(' ')

    if reply=="ok": # This is what shm will say if it was successful
        return value
    
    if resp[0]=="?":
        # An error was generated
        error_buffer("Error in wshm('%s',%s,%i): '%s'"%(variable,value,index,reply))
        return None
    
    return None
    
    



def send_shm(cmd):
    global shm
    send(shm,cmd)


def read_shm():
    global shm
    return read(shm)

    


def stop_shm():
    ## TODO
    #subprocess.call(shm_stop,cwd=robot_dir)
    global shm
    send(shm,'q') # quit the shm process
    shm = None


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
for executable in [robot_start,robot_stop,shm_start]:

    if not os.path.isfile(executable):
        print("ERROR: could not find executable %s (did you compile the robot code already?)"%executable)


    




def error_buffer(err):
    """ Write a particular error to the buffer. """
    global errbuff
    print("-- ERROR -- "+err)
    errbuff.append(err)
    





# Perhaps a nicer way to read/write to processes:
# http://stackoverflow.com/questions/19880190/interactive-input-output-using-python




"""

Stuff that relates to interactive communication with child processes.

"""

import os
import fcntl
import subprocess


def setNonBlocking(fd):
    """
    Set the file description of the given file descriptor to non-blocking.
    """
    flags = fcntl.fcntl(fd, fcntl.F_GETFL)
    flags = flags | os.O_NONBLOCK
    fcntl.fcntl(fd, fcntl.F_SETFL, flags)



def spawn_process(cmd,cwd=None):
    p = subprocess.Popen(cmd,
                         stdin  = subprocess.PIPE,
                         stdout = subprocess.PIPE,
                         stderr = subprocess.PIPE,
                         bufsize = 1,
                         cwd = cwd,
                         universal_newlines = True # use this in Python3 to signal string-interactions
                         )
    setNonBlocking(p.stdout)
    setNonBlocking(p.stderr)

    return p
    



def send(proc,cmd):
    """ Sends a command to a process. """
    try:
        proc.stdin.write(cmd+"\n")
    except:
        error_buffer("Error writing to process!")
        


    

def readline(proc):
    """ Read a single line from the process, or return None if nothing is available. """
    try:
        #out1 = shm.stdout.read()
        out1 = proc.stdout.readline()
        if len(out1)>0:
            return out1
    except IOError:
        print("Got error reading.")
        #continue
    #else:
    #    break
    return None
    


def readall(proc):
    """ Read all available output from the process. """
    lines = []
    line = readline(proc)
    while line!=None:
        lines.append(line)
        line = readline(proc)
    return lines




def read(proc):
    resp=None

    if proc==None:
        error_buffer("Cannot read from something that is not a process.")
        return None
    
    while True:
        try:
            #out1 = shm.stdout.read()
            out1 = proc.stdout.read()
            if len(out1)>0:
                if resp==None:
                    resp = out1
                else:
                    resp += out1
        except IOError:
            #print("Got error")
            continue
        except TypeError:
            continue
        else:
            break
    return resp
    

