
import sys
import sysv_ipc
import struct
import collections


# Map the type definitions used in the C robdecls.h code
# to the data types that python's struct uses.
py_types = {
    "s32":"i",
    "u32":"I",
    "f64":"d",
}



# Below are the keys that we use to access the shared memory areas
# where they are located.
objects = {
    "Ob"         : 0x494D5431,
    "Dyncmp_var" : 0x494D5437,
    "Robot"      : 0x494D5432,
    "Daq"        : 0x494D5433,
}

# These data types come from robdecls.h I think:
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
    "x"               :"pos_x",
    "y"               :"pos_y",
    "no_safety_check" :"safety_override",
    "slot_fnid"       :"copy_slot_fnid",
    "slot_id"         :"copy_slot_id",
    "slot_go"         :"copy_slot_go",
    "slot_running"    :"copy_slot_running",
    "have_pc7266"     :"pc7266_have",
    "have_pci4e"      :"pci4e_have",
}



def add_aliases(fname):
    """ 
    Some aliases are exceeding erratic (for historical reasons I suppose)
    so I just add them to a file and here I can read that file.
    """
    print("Adding aliases from %s"%fname)
    f = open(fname,'r')
    for ln in f.readlines():
        ln = ln.strip().split(' ')
        if len(ln)==2:
            ali,name = ln[0].strip(),ln[1].strip()
            name = name.replace('.','_') # in the original naming scheme, dots were used because they were c names, however, at this level we already use our own names, where underscore is the new standard.
            
            if name!=ali: # If this isn't a name that we would have guessed anyway...
                varname_aliases[ali]=name
                #print("Added alias %s"%ali)
    f.close()
    return




# Read in a table that tells us where each variable is within each object.
def read_address_probe():
    """ 
    This reads a table where each row is OB,TYPE,VAR,ARRSTRUCT,ADDR
    where OB is the object of which VAR is a member, and ADDR
    is the address where that VAR is found, and TYPE is its data type.
    If the variable is an array, ARRSTRUCT (an int) tells us how many elements it has,
    and 0 means it is not an array.

    Example line: 
    Ob u32 mkt_isMcGill 0 6444

    Returns:
    A dict where the keys are the variable names (hopefully unique)
    and the values are tuples (OB,TYPE,ARRSTRUCT,ADDR).
    """
    try:
        f = open('robot/field_addresses.txt','r')
    except:
        print("Can't open robot/field_addresses.txt. This usually means you haven't run 'make robot' or 'make rob'.")
        sys.exit(-1)
    lns = f.readlines()
    f.close()

    fields = {}
    for ln in lns:
        elts = ln.strip().split(" ")
        if len(elts)==5:
            ob,tp,var,arrn,addr=elts
            if var in fields and var!="tag": # tag is allowed to co-occur
                print("WARNING: %s already exist in object."%var)
            else:
                if not addr.isdigit():
                    print("Cannot parse %s as an address (should be an integer)"%addr)
                else:
                    if not arrn.isdigit():
                        print("Cannot parse %s as an array length (should be an integer)"%arrn)
                    else:
                        fields[var] = (ob,tp,int(arrn),int(addr))
        else:
            print("Ignoring line %s"%ln)
                
    return fields






def read(memobj,what,loc,n=1):
    """ Read from the shared memory.

    Arguments
    memobj : the shared memory object
    what : data type to be read, e.g. "u32". 
    loc : location in the shared memory (offset)
    n : how many (consecutive) data points to read.
    """
    tp = py_types[what]
    sz = struct.calcsize(tp)
    res = memobj.read(byte_count=sz*n,offset=loc)

    dat = struct.unpack("%i%s"%(n,tp),res)
    return dat




def write(memobj,what,loc,value,n=1):
    """ Write something to the shared memory. 
    
    Arguments
    what  : data type to be written
    loc   : location (offset) in the shared memory where it should be written
    value : the value to be written
    """
    tp = py_types[what]
    fmt = "%i%s"%(n,tp)
    dt = struct.pack(fmt,*value) if isinstance(value, collections.MutableSequence) else struct.pack(fmt,value)
    memobj.write(dt,offset=loc)
    return
    



def get_element_loc(loc,arrn,index,what):
    """ 
    Get the location of a data, where the data may be an array.
    
    Arguments
    loc   : the location of the (start) of the data, i.e. the location of the data itself or the start of the array
    arrn  : the number of items in the data; 0 means it is not an array, anything larger is the number of elements
    index : the requested index in the array
    what  : the data type to be read (and if we read from an array, the data type of the elements of the array)
    """
    if index==None or index==0: # If we don't specify a location, either this is not an array or if it is, we by default return the first element
        return loc # TODO: if this is an array, we might want to just return the actual array instead of just its first element.
    else:

        # Okay, we are requesting the non-zero location of something, that means it has to be an array, and the location has
        # to be less than the array size!
        if not arrn>0:
            print("Error: trying to read index of non-array object!")
            return None
        if index>=arrn:
            print("Error: trying to read index %i which is larger than the size of the array (%i)."%(index,arrn))
            return None

        # Compute the size of elements in the array
        sz = struct.calcsize(py_types[what])
        return loc+ (index*sz)



    

def rshm(var,index=None):
    """ Read from shared memory. """
    try:
        ob,what,arrn,eloc  = get_info(var,index)
        global memobjects
        
        # Determine how many items to read
        n = arrn if arrn!=None and arrn>0 and index==None else 1
        # If this is an array variable and we did *not* give an index,
        # then we assume the user wants the whole array.

        dat = read(memobjects[ob],what,eloc,n)

        if index!=None or arrn==None or arrn==0:
            return dat[0]
        else:
            return list(dat)

    except:
        return None
    


def wshm(var,value,index=None):
    """ Write to the shared memory. """
    ob,what,arrn,eloc  = get_info(var,index)

    # Determine exactly what we are going to write.
    n = 1 # how many data points to write
    if arrn==None or arrn==0:
        if index!=None:
            print("WARNING: ignoring index %i when writing to non-array %s"%(index,var))
    else:
        # If this /is/ an array object
        if index==None:
            # Set the whole array at once
            if not isinstance(value, collections.MutableSequence):
                print("Trying to write to array %s but you haven't specified an index and you haven't given a list of %i values either."%(var,arrn))
                return
            if len(value)!=arrn:
                print("ERROR: trying to set array %s of %i elements to an array of %i elements."%(var,arrn,len(value)))
                return
            else:
                n = len(value)
                # Write the whole array in one fell swoop to memory, nice!
                
    global memobjects
    write(memobjects[ob],what,eloc,value,n)

    




def get_info(var,index=None):
    # Return the OB,TYPE,ADDR for the given variable.
    global locs

    # Look whether this variable is an alias for something else
    if var in varname_aliases:
        var = varname_aliases[var]

    # Otherwise, continue looking it up.
    if not var in locs:

        altvar = var.replace("_",".")
        if altvar in locs:
            var = altvar
        else:
            
            # Oops, this variable doesn't seem to exist!
            print("WARNING -- Variable %s not found!"%var)
            return None
        
        
    ob,what,arrn,loc  = locs[var]
    eloc = get_element_loc(loc,arrn,index,what)
    if eloc==None: return None
    return (ob,what,arrn,eloc)





def init(aliasfile="robot/alias_list.txt"):
    """
    Initialise the shared memory. This assumes that a robot process is already running
    so that a block of shared memory is allocated for us to connect to.
    """

    # For now, we read aliases from alias_list.txt. However, they are quite erratic and weird because
    # they were added in the time when the robot was controlled using Tcl (and nobody knows why they
    # didn't stick to logical variable naming). 
    add_aliases(aliasfile)

    
    global locs
    global memobjects
    
    memobjects = {}

    for ob in objects:

        # Try to attach to the robot memory
        try:
            memobj = sysv_ipc.SharedMemory(objects[ob]) #, [flags = 0, [mode = 0600, [size = 0 or PAGE_SIZE, [init_character = ' ']]]])
            memobjects[ob]= memobj
        except:
            print("Cannot access shared memory key OB_KEY. Robot process not running?")




def stop_shm():
    """
    Detaches from the shared memory. Run this when you are unloading the robot.
    You'll probably want to keep the shared memory open until the last moment though
    so that you can tell the robot to shut itself down through the shared memory
    (e.g. wshm('quit',1) )
    """
    # When you are done...

    global memobjects
    for ob in memobjects:
        memobjects[ob].detach()
    
            


def start_shm():
    """ 
    Initialises the shared memory object, connects to the shared memory blocks. This will fail
    if a robot process is not running because the shared memory has in that case not been allocated.
    """
    init()




def shm_status():
    """ 
    Returns the status of the shared memory.
    """
    status = ""
    if shm_connected():
        status += "connected"
    else:
        status += "shared memory objects not initialised (call start_shm())."
        
    return status




def shm_connected():
    """ Returns whether we are connected to the shared memory or not. """
    global memobjects
    return len(memobjects.keys())>0






def dump_shm():
    """ This aims to read all variables from the shared memory,
    as a kind of snapshot, and return them as a set of tuples. """

    global locs
    varnames = list(locs.keys())
    
    var_values = []
    for c in varnames:
        c = c.strip()
        var = c.split(' ')[0].strip()

        try:
            res = rshm(var)
            #print(var,str(res))
            var_values.append((var,res)) #[var]=str(res)

        except:
            continue
    
    return dict(var_values)
    








if True: # module initialisation

    global locs,memobjects

    # Read address information about the data
    locs = read_address_probe()

    memobjects = {}
