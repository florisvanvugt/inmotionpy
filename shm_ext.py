"""

This is the "traditional", external way to connect to the shared memory,
namely by keeping open a pipe connection with a C script "shm".

I don't intend to keep using this since it is not a very elegant solution
and probably much slower than having Python just grab info from memory
directly.

"""

import os
import fcntl

import subprocess
import time


# The script to execute when we start the shm
robot_dir   = os.getcwd()+"/robot" # we assume that a subdirectory of our script contains the robot code.
shm_start   = "%s/shm"%robot_dir

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





def stop_shm():
    ## TODO
    #subprocess.call(shm_stop,cwd=robot_dir)
    global shm
    send(shm,'q') # quit the shm process
    shm = None
        


# Perhaps a nicer way to read/write to processes:
# http://stackoverflow.com/questions/19880190/interactive-input-output-using-python




"""

Stuff that relates to interactive communication with child processes.

"""


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
    

