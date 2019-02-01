# This is a really silly module. It just pretends to be a robot.

from __future__ import absolute_import


import os
import subprocess
import time
import sys

import time


info = {'x':0.0,'y':0.0}


def status():
    return ""


def load():
    print("Loading robot...")


def unload():
    print("Unloading robot...")
    info['quit']=1

def launch():
    print("Launching robot...")
    

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

    
def move_is_done():
    movedone = info['movestart']+info['movedur']
    return time.time()>movedone



def update_prediction():
    # Here we shamelessly predict where we will currently be
    if 'moving' in info and not move_is_done():
        # Currently moving! Let's predict our position

        for dm in ['x','y']:
                
            # Proportion of move completed
            prop = (time.time()-info['movestart'])/info['movedur']

            # Don't bother with minjerk trajectory, just linear!
            p1,p2 = rshm('plg_p1%s'%dm),rshm('plg_p2%s'%dm)
            info[dm]= p1+ (p2-p1)*prop
            
    

def rshm(v):
    # Read shared memory (pretend to...)
    return info.get(v,None)




from threading import Thread

def loop():
    while rshm('quit')!=1:
        time.sleep(.001)
        update_prediction()

Thread(target=loop).start()
