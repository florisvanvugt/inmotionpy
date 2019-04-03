# This is a really silly module. It just pretends to be a robot.

from __future__ import absolute_import


import os
import subprocess
import time
import sys

import time
import numpy as np


PERIOD = .0025 # the period of the main loop (in sec)




info = {'x':0.0,'y':0.0}

stiffness = 0
damping = 0


# This allows us to pre-program some future states for the robot
# On each pass of the loop, we will take one element of this list,
# which should be a dict that tells us the desired future state
# of particular variables.
future = []



PYTHON3 = (sys.version_info > (3, 0))
if PYTHON3: # this really is tested on Python 3 so please use that
    from tkinter import *

else: # python2
    print("## WARNING: don't use Python 2. This code is designed for Python 3.")
    from Tkinter import * # use for python2





from threading import Thread

def loop():
    global future
    while rshm('quit')!=1:
        time.sleep(.1*PERIOD)# wait a little
        if time.time()<info.get('next.tick.t',0):
            continue

        info['next.tick.t']=time.time()+PERIOD

        if len(future)>0:
            changes = future.pop(0)
            for k in changes: # Make the corresponding changes
                wshm(k,changes[k])
                #print(k,changes[k])
            if len(future)==0:
                print("Dummy: Exhausted future program.")

        # If we are supposed to be capturing, capture!
        if rshm('fvv_capture'):

            def append_to(c,r):
                """ Add the current value of some variable in shared memory,
                named c, to an array, stored in the shared memory under variable
                name r."""
                v = rshm(c)
                arr = info.get(r,[])
                arr.append(v)
                info[r]=arr
                return len(arr)

            for (c,r) in [('x','trajx'),
                          ('y','trajy'),
                          ('ft_world_x','recordfx'),
                          ('ft_world_y','recordfy'),
                          ('ft_world_z','recordfz')]:
                info['traj_count']=append_to(c,r)
            


def status():
    return ""



def update_popup():
    global popupinfo
    while not rshm('quit'):
        rx,ry=rshm('x'),rshm('y')
        if rx and ry:
            bbox = position_to_circle((rx,ry))
            popupinfo['canvas'].coords(popupinfo['robotpos'],*bbox)
            t = '{:0.2f},{:0.2f}'.format(rx,ry)
            popupinfo['canvas'].itemconfigure(popupinfo['postext'],text=t)
            popupinfo['canvas'].coords(popupinfo['postext'],*bbox[:2])
        time.sleep(.1) # update rate for the dummy window

POPUP_W,POPUP_H=400,200
POPUPCX,POPUPCY=POPUP_W/2,POPUP_H/2
POPUP_MAGNIF=500
POPUP_POS_RAD = 5 # radius in pixels of the marker for the current robot position

def rob2popup(pos):
    """ Convert robot coordinates to popup coordinates """
    x,y=pos
    return int((x*POPUP_MAGNIF)+POPUPCX),int((-y*POPUP_MAGNIF)+POPUPCY)

def position_to_circle(pos):
    x,y=rob2popup(pos)
    return x-POPUP_POS_RAD,y-POPUP_POS_RAD,x+POPUP_POS_RAD,y+POPUP_POS_RAD

def popup(master):
    global popupinfo
    window = Toplevel(master)
    #window.geometry('{}x{}'.format(POPUP_W,POPUP_H))
    window.title('Dummy robot sneak peek')
    canvas = Canvas(window, width=POPUP_W,height=POPUP_H,bg='black')
    canvas.pack()

    # Make a center marker
    bbox = position_to_circle((0,0))
    canvas.create_oval(bbox,fill='gray')
    canvas.create_line(POPUPCX,0,POPUPCX,POPUP_H, fill="gray", dash=(4, 4))
    canvas.create_line(0,POPUPCY,POPUP_W,POPUPCY, fill="gray", dash=(4, 4))

    # Make a marker for the current robot position, and a text label as well
    robotpos = canvas.create_oval([0,0,0,0],fill='red')
    txt = canvas.create_text(0,0,text='hello',fill='blue')
    popupinfo = {'window':window,
                 'canvas':canvas,
                 'robotpos':robotpos,
                 'postext':txt}
    Thread(target=update_popup).start()

def load():
    print("Loading robot...")
    Thread(target=loop).start()


def unload():
    print("Unloading robot...")
    info['quit']=1

def launch():
    print("Launching robot...")
    load()
    

def dump_shm():
    return info


def controller(n):
    print("Controller %d"%n)
    info['controller']=n


def start_log(fname,n):
    pass

def stop_log():
    pass


def zeroft():
    return


def bias_report():
    return "Bias report"



def stay():
    return

def stay_at(x,y):
    wshm('x',x)
    wshm('y',y)
    return


def wshm(var,v):
    # Write a value to the shared memory
    info[var]=v
    

def move_to(x,y,t):
    wshm('plg_movetime',0.0)
    wshm('plg_p2x',x)
    wshm('plg_p2y',y)
    wshm('plg_movetime',float(t))
    wshm('plg_moveto_done',0)
    wshm('plg_p1x',rshm('x'))
    wshm('plg_p1y',rshm('y'))

    info['moving'] = True
    info['movestart'] = time.time()
    info["movedur"]   = t

    print("Moving to %f,%f"%(x,y))
    future = preprogram_move_to(x,y,t)
    


def preprogram_move_to(x,y,t):
    # Now we create some future states that will determine where we predict the robot will go
    fut = []
    nsamp = int(t/PERIOD) # how many "periods" there are
    curpos  = {"x":rshm('x'),"y":rshm('y')} # current position
    targpos = {"x":x,"y":y} # target position
    
    for i in range(nsamp):
        f = {}
        # Proportion of move completed (between 0 and 1)
        tau = i/float(nsamp)
        #tau = float(t-info['movestart'])/info['movedur']

        # Minimum jerk trajectory
        posprop = -( 15*pow(tau,4)-6*pow(tau,5) -10*pow(tau,3))
    
        for dm in ['x','y']:
            p1,p2 = curpos[dm],targpos[dm]
            f[dm]=p1+(p2-p1)*posprop
        fut.append(f)

    fut.append({"plg_moveto_done":1}) # signal that the move is completed
    global future
    future = fut # "launch" this future, which will make it start "playing"
    #print(future)
    
    
    
def move_is_done():
    return rshm('plg_moveto_done')==1

# movedone = info['movestart']+info['movedur']
#return time.time()>movedone



    

def rshm(v):
    # Read shared memory (pretend to...)
    res = info.get(v,None)
    if v in ['x','y']:
        return res+np.random.normal(0,.0002) # add a little noise to make it look even more realistic

    # For forces, just pretend that they are random
    if v in ['ft_world_x','ft_world_y','ft_world_z']:
        return np.random.normal(0,1) # add a little noise to make it look even more realistic
    
    return res





def background_capture():
    wshm('traj_count',0)
    wshm('fvv_capture',1)
    for v in ['trajx','trajy','recordfx','recordfy','recordfz']:
        info[v]=[]


def stop_background_capture():
    wshm('fvv_capture',0)
    return retrieve_captured()


def retrieve_trajectory():
    """
    If you have called start_capture(),
    this retrieves the trajectory that was captured.
    """
    xs,ys=rshm('trajx'),rshm('trajy')
    #print(xs)
    #print(ys)
    n = rshm('traj_count')
    #print(n)
    return zip(xs[:n],ys[:n])
    


def retrieve_forces():
    """
    Retrieve captured forces.
    """
    fx,fy,fz=rshm('recordfx'),rshm('recordfy'),rshm('recordfz')
    n = rshm('traj_count')
    return zip(fx[:n],fy[:n],fz[:n])
    

def retrieve_captured():
    """ Retrieve captured trajectory and
    captured forces."""
    n = rshm('traj_count')
    xs,ys,fx,fy,fz = rshm('trajx'),rshm('trajy'),rshm('recordfx'),rshm('recordfy'),rshm('recordfz')
    return zip(xs[:n],ys[:n],fx[:n],fy[:n],fz[:n])





print("#\n#\n#\n#\n#\n# YOU ARE USING THE DUMMY ROBOT. THIS WON'T DO ANY ACTUAL MOVEMENT.\n#\n#\n#\n#\n#\n\n\n")
