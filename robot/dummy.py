

# This is a dummy module that we can use when we are developing away from
# the robot computer and therefore we cannot import the actual robot code.
shm = {}
controller = 0
loaded = False
shm_conn = False

def status():
    return "Dummy robot status"

def load():
    print("Loading robot")
    global loaded,shm_conn
    loaded = True
    shm_conn = True

def unload():
    global loaded,shm_conn
    loaded = False
    print("Unloading robot")

def probe_process():
    return loaded
    
def get_controller():
    global controller
    return controller

def controller(i):
    global controller
    controller=i

def rshm(where):
    if where in shm.keys():
        return shm[where]
    else:
        return 0.0

def wshm(where,what):
    shm[where]=what

def start_loop():
    wshm("paused",0)
    wshm("slot_max",4)

def pause_loop():
    wshm("paused",1)

def stop_loop():
    wshm("paused",1)
    wshm('quit',1)


def start_log(fname,n):
    f = open(fname,'w')
    f.write('log from dummy robot')
    f.close()
    
def stop_log():
    pass

def bias_report():
    return "Bias report: N/A"

def zeroft():
    print("Zeroing FT")
    return None

def shm_connected():
    return shm_conn

def start_shm():
    shm_conn = True
    
