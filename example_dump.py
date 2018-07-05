
import time
import robot.interface as robot
import sys

import pickle
import datetime


PYTHON3 = (sys.version_info > (3, 0))
if PYTHON3:
    from tkinter import *
    from tkinter import messagebox as tkMessageBox

else: # python2
    from Tkinter import * # use for python2
    import tkMessageBox

import json
    
# Some parameters that specify how we draw things onto our window
w,h = 900,600
windowx,windowy= 600,200
cw,ch = w/2,h/2
robot_scale = 600
cursor_size = 10
target_size = 5

trace_size = 2 # size of the dots that show previous positions


robot.load()
robot.bias_force_transducers()
robot.status()



def rob_to_screen(x,y):
    # Convert robot coordinates into screen coordinates
    return (cw + x*robot_scale,ch - y*robot_scale)



def on_closing():
    global keep_going
    keep_going = False

    ## Write the captured info to a file (just JSON for now because it's easy to work on)
    global captured
    timestamp = datetime.datetime.now().strftime("%Y%d%m_%Hh%Mm%S")
    json.dump({"time":time.time(),
               "timestamp":timestamp,
               "shm_dump":robot.dump_shm(),
               "captured":captured},open("captured_%s.json"%timestamp,'w'))
    

def init_interface():
    
    # Set up the main interface scree
    global master
    global gui
    
    master = Tk()
    master.geometry('%dx%d+%d+%d' % (w, h, windowx,windowy))

    win = Canvas(master, width=w, height=h)
    win.grid(column=0,row=1,stick=(N,W,E,S))


    # Draw the background against which everything else is going to happen
    win.create_rectangle(0, 0, w, h, fill="black")
    minx,miny = rob_to_screen(-.4,-.2)
    maxx,maxy = rob_to_screen( .4,.3)
    win.create_rectangle(minx,miny,maxx,maxy, outline="blue")

    # Draw a little center point
    x,y = rob_to_screen(0,0)

    cross_size=10
    win.create_line(x-cross_size,y,x+cross_size,y,fill='green')
    win.create_line(x,y-cross_size,x,y+cross_size,fill='green')


    robot_pos = win.create_oval(cw,ch,cw,ch,fill="blue")
    coord_txt = win.create_text(cw,ch,text='NA,NA',fill='white',anchor='w')

    master.protocol("WM_DELETE_WINDOW", on_closing)

    gui = {}
    gui["win"]=win
    gui["robot_pos"]=robot_pos
    gui["coord_txt"]=coord_txt

    




def draw_robot():
    # Update the cursor that indicates the current position of the robot
    rx,ry = robot.rshm("x"),robot.rshm("y")
    #print(rx,ry)
    x,y = rob_to_screen(rx,ry)
    global gui
    win = gui["win"]
    robot_pos = gui["robot_pos"]
    coord_txt = gui["coord_txt"]
    win.coords(robot_pos,(x-cursor_size,y-cursor_size,x+cursor_size,y+cursor_size))
    win.itemconfig(coord_txt,text="  %.3f,%.3f"%(rx,ry))
    win.coords(coord_txt,(x,y))

    win.create_oval(x-trace_size,y-trace_size,x+trace_size,y+trace_size,fill='red')


def capture_position():
    # Capture the current position
    rx,ry = robot.rshm("x"),robot.rshm("y")

    global captured
    captured.append( (rx,ry) )
    


    
def routine():
    # The things we'll want to do all the time
    draw_robot()

    capture_position()

    

global captured
captured = []

init_interface()


keep_going = True

while keep_going:

    routine()

    master.update_idletasks()
    master.update()
    time.sleep(0.05) # frame rate of our GUI update and capture process

    
robot.unload()

