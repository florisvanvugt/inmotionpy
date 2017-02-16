

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



def load():
    """ TODO """

    # TODO: Make sure the robot is not already loaded
    start_lkm()
    start_shm()
    start_loop()

    # Not sure if this does anything, but just to make sure
    wshm("plg_last_fX",0.0)
    wshm("plg_last_fY",0.0)

    # Remove the safety zone so that the robot arm can move everywhere - careful!
    wshm("no_safety_check",1)
    
    return



def unload():
    """ TODO """
    #plg_select_controller 0; # should be the null field

    # b_pause_proc $w
    # pause pauses actuator output, stop stops all main loop i/o.
    stop_loop()
    stop_shm()
    stop_lkm()

    #set ob(loaded) 0
    




def start_lkm():
    """ Starts the robot process. """
    print(robot_start,robot_dir)
    global rob
    rob = spawn_process(robot_start,robot_dir)
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

    check = rshm('last_shm_val')
    if check!=12345678:
        print("Error: rshm check returned %s instead of 1234578."%(str(check)))
        sys.exit(-1)

    #set ob(shm) [open "|$ob(crobhome)/shm" r+]
    #fconfigure $ob(shm) -buffering line
    #after 100
    #set check [rshm last_shm_val]
    #if {$check != 12345678} {
    #puts "start_shm: bad shm check value."
    #puts "make sure all software has been compiled with latest cmdlist.tcl"
    #exit 1
    #}

    


    
def rshm(variable,index=0):
    """ Read a variable from the shared memory and return its value. """

    global shm
    # TODO: check that the shm is running

    # This is the query string we will send to the shm script
    query = "g %s %i"%(variable,index)
    send(shm,query)

    return read(shm)
    
    #try:
    #    response, errs = shm.communicate(input=query,timeout=15)#
    # TODO: Various response checking
    #return response
    #except TimeoutExpired:
    #    print("Time out while communicating with robot shared memory.")

    return None




def wshm(variable,value,index=0):
    """ Writes the given value to the given variable in the shared memory. """
    
    # TODO: check if the shm process exists
    query = "s %s %i %s"%(variable,index,str(value))
    try:
        response, errs = shm.communicate(input=query,timeout=15)

        # TODO: Various response checking
        
        return response
    except TimeoutExpired:
        print("Time out while communicating with robot shared memory.")
    pass




def send_shm(proc,cmd):
    global shm
    send(shm,cmd)




def stop_shm():
    ## TODO
    subprocess.call(shm_stop,cwd=robot_dir)
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


def bias_force_transducers():
    ## TODO!!
    print("")
    print("*** biasing force transducers: please let go of the handle")
    print("*** and hit ENTER when ready to start zeroing procedure")
    input()
    plg_zeroft() # TODO
    
    print(" ")
    print("*** zeroing the force transducers is done, please hold the handle now")
    print("*** and hit ENTER when ready to continue")
    input()

    





if not os.path.isfile(robot_start):
    print("ERROR: could not find executable %s"%robot_start)


if not os.path.isfile(robot_stop):
    print("ERROR: could not find executable %s"%robot_stop)
    
    










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
        print("Error writing to process!")


    

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
        else:
            break
    return resp
    

