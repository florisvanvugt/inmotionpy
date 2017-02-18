
import sys
import sysv_ipc
import struct



# Map the type definitions used in the C robdecls.h code
# to the data types that python's struct uses.
py_types = {
    "u32":"I",
    "f64":"d",
}



# Below are the keys that we use to access the shared memory areas
# where they are located.
objects = {
    "Ob"         : 0x494D5431,
    "Dyncmp_var" : 0x494D5437
}

#define OB_KEY   0x494D5431
#define ROB_KEY  0x494D5432
#define DAQ_KEY  0x494D5433
#define PREV_KEY 0x494D5434
#define GAME_KEY 0x494D5435
#define USER_KEY 0x494D5436
#define DYNCMP_KEY 0x494D5437



# If you don't like to type much, you can define variable aliases.
# For example add "x":"pos.x" to the varname_aliases dict
# to make every rshm() or wshm() call with the variable "x"
# to actually operate on the variable "pos.x".
varname_aliases = {
    "x":"pos.x",
    "y":"pos.y"
}



# Read in a table that tells us where each variable is within each object.
def read_address_probe():
    """ 
    This reads a table where each row is OB,TYPE,VAR,ADDR
    where OB is the object of which VAR is a member, and ADDR
    is the address where that VAR is found, and TYPE is its data type.

    Example line: 
    Ob u32 mkt_isMcGill 6444

    Returns:
    A dict where the keys are the variable names (hopefully unique)
    and the values are tuples (OB,TYPE,ADDR).
    """
    f = open('field_addresses.txt','r')
    lns = f.readlines()
    f.close()

    fields = {}
    for ln in lns:
        elts = ln.strip().split(" ")
        if len(elts)==4:
            ob,tp,var,addr=elts
            if var in fields:
                print("WARNING: %s already exist in object."%var)
            else:
                if not addr.isdigit():
                    print("Cannot parse %i as an address (should be an integer)"%addr)
                else:
                    fields[var] = (ob,tp,int(addr))
        else:
            print("Ignoring line %s"%ln)
                
    return fields






def read(memobj,what,loc):
    """ Read from the shared memory.

    Arguments
    memobj : the shared memory object
    what : data type to be read, e.g. "u32". 
    loc : location in the shared memory (offset)
    """
    tp = py_types[what]
    sz = struct.calcsize(tp)
    res = memobj.read(byte_count=sz,offset=loc)

    return struct.unpack(tp,res)


# Read u32 at a given memory location in the shared memory
# memloc = 6448


def write(memobj,what,loc,value):
    """ Write something to the shared memory. 
    
    Arguments
    what  : data type to be written
    loc   : location (offset) in the shared memory where it should be written
    value : the value to be written
    """
    tp = py_types[what]
    dt = struct.pack(tp,value)
    memobj.write(dt,offset=loc)
    return
    




def rshm(var):
    """ Read from shared memory. """
    ob,what,loc  = get_info(var)
    global memobjects
    return read(memobjects[ob],what,loc)[0]
    


def wshm(var,value):
    """ Write to the shared memory. """
    ob,what,loc  = get_info(var)
    global memobjects
    write(memobjects[ob],what,loc,value)




def get_info(var):
    # Return the OB,TYPE,ADDR for the given variable.
    global locs

    # Look whether this variable is an alias for something else
    if var in varname_aliases:
        var = varname_aliases[var]

    # Otherwise, continue looking it up.
    if var in locs:
        return locs[var]
    else:

        # Oops, this variable doesn't seem to exist!
        print("Variable %s not found!"%var)
        return None





def init():

    global locs
    global memobjects
    
    # Read address information about the data
    locs = read_address_probe()

    memobjects = {}

    for ob in objects:

        # Try to attach to the robot memory
        try:
            memobj = sysv_ipc.SharedMemory(objects[ob]) #, [flags = 0, [mode = 0600, [size = 0 or PAGE_SIZE, [init_character = ' ']]]])
            memobjects[ob]= memobj
        except:
            print("Cannot access shared memory key OB_KEY. Robot process not running?")
            sys.exit(-1)




def quit():
    # When you are done...    

    global memobjects

    for ob in memobjects:
        ob.detach()
    
            


        
init()
