
import sys
import sysv_ipc

OB_KEY = 0x494D5431
#define OB_KEY   0x494D5431
#define ROB_KEY  0x494D5432
#define DAQ_KEY  0x494D5433
#define PREV_KEY 0x494D5434
#define GAME_KEY 0x494D5435
#define USER_KEY 0x494D5436
#define DYNCMP_KEY 0x494D5437


# Let's create the hash table so we can look up where items are in the shared memory


def comment_remover(text):
    """ Source: http://stackoverflow.com/questions/241327/python-snippet-to-remove-c-and-c-comments """
    import re
    def replacer(match):
        s = match.group(0)
        if s.startswith('/'):
            return " " # note: a space and not an empty string
        else:
            return s
    pattern = re.compile(
        r'//.*?$|/\*.*?\*/|\'(?:\\.|[^\\\'])*\'|"(?:\\.|[^\\"])*"',
        re.DOTALL | re.MULTILINE
    )
    return re.sub(pattern, replacer, text)    



def parse_robdecls():
    """ 
    Now we need to know where C stores its stuff...
    """
    f = open('../robot/robdecls.h','r')
    robdecls = f.read()
    f.close()

    purec = comment_remover(robdecls)


    td = re.compile(r'typedef\s*struct\s*(\w+)\s*{([^}]*)}\s*(\w+);')
    matchs = td.findall(txt)
    structs = {}
    for match in matchs:
        obname = match[-1]
        obcont = match[1]

        contents = [ itm.strip() for itm in obcont.split(";") ]

        vrs = []
        for cont in contents:
            if len(cont)>0:
                fields = cont.split(' ')
                if len(fields)!=2:
                    print("Error parsing %s!"%cont)
                else:
                    vrs.append(fields)
        structs[obname]=vrs
    
    return structs


res = parse_robdecls()
res["xy"]

txt = parse_robdecls()



# PROBLEM: from the typedef I want to know where in the memory I may find that item... I assume they are in order of appearance but I need to figure out how large each thing is for me to easily find the memory addresses.... now what?



# Try to attach to the robot memory
try:
    ob = sysv_ipc.SharedMemory(OB_KEY) #, [flags = 0, [mode = 0600, [size = 0 or PAGE_SIZE, [init_character = ' ']]]])
except:
    print("Cannot access shared memory key OB_KEY. Robot process not running?")
    sys.exit(-1)




# Map the type definitions used in the C robdecls.h code
# to stuff we can do in Python
py_types = {"u32":"I"}

import struct

def read(what,loc):
    """ Read from the shared memory.

    Arguments
    what : data type to be read, e.g. "u32". 
    loc : location in the shared memory (offset)
    """
    tp = py_types[what]
    sz = struct.calcsize(tp)
    # Access something in the shared memory
    res = ob.read(byte_count=sz,offset=loc)

    return struct.unpack(tp,res)


# Read u32 at a given memory location in the shared memory
# memloc = 6448


def write(what,loc,value):
    """ Write something to the shared memory. 
    
    Arguments
    what  : data type to be written
    loc   : location (offset) in the shared memory where it should be written
    value : the value to be written
    """
    tp = py_types[what]
    dt = struct.pack(tp,value)
    ob.write(dt,offset=loc)
    return
    




def rshm(var):
    """ Read from shared memory. """
    loc  = get_location(var)
    what = get_type(var)

    return read(what,loc)[0]
    

def wshm(var,value):
    """ Write to the shared memory. """
    loc  = get_location(var)
    what = get_type(var)

    write(what,loc,value)




locs = {"fvv_trial_phase":6448}


def get_location(var):
    return locs[var]








def parse_cmdlist():
    cmds = {}
    f = open('cmdlist.txt','r')
    i = 0
    for ln in f.readlines():
        ln = ln.strip()
        elts = ln.split(' ')
        name = elts[1]
        cmds[name]={"index":i,
                    "type":elts[0],
                    "loc":elts[2],
                    "size":elts[3]}
        i+=1
    f.close()

    return cmds






cmds = parse_cmdlist()


def get_type(var):
    """ Get the data type for this variable. """
    return cmds[var]["type"]


def gethindex(key):
    """ Given a key, return its index (in the shared memory). """
    return cmds[key]




# When you are done...    
ob.detach()




