from tkinter import *

import time
import robot.interface as robot

import numpy as np



# Some parameters that specify how we draw things onto our window
w,h = 800,600
cw,ch = w/2,h/2
robot_scale = 700
cursor_size = 10
target_size = 5

robot_move_time = 3.

done_targets = []   # targets that are "done" (we have already moved to them)
targets = []        # targets we are to move to in the future
moving = False      # whether we are currently moving to a target


capturing = False         # whether we are capturing the trajectory currently
capture_start = None      # the time when we started capturing
trajectory = []           # the trajectory we have captured (if any)
CAPTURE_DURATION = 3.     # how long to capture for (in sec)
trajectory_display = None # the lines that we use to display on the screen

SMOOTHING_WINDOW_SIZE = 9 # how big a window to use for smoothing (see tools/smoothing for details about the effects)
SMOOTHING_WINDOW = np.hamming(SMOOTHING_WINDOW_SIZE)


replaying = False   # whether we are currently replaying something


robot.load()
robot.bias_force_transducers()
#robot.stay() # stay put
robot.status()





def rob_to_screen(x,y):
    # Convert robot coordinates into screen coordinates
    return (cw + x*robot_scale,ch - y*robot_scale)


def screen_to_rob(x,y):
    # Convert screen coordinates into robot coordinates
    return ( (x-cw)/float(robot_scale),
             (y-ch)/float(-robot_scale) )



def mouseclick(event):
    global capturing
    
    # When the mouse is clicked, show where we are
    canvas = event.widget
    x = canvas.canvasx(event.x)
    y = canvas.canvasy(event.y)
    print("Clicked",x,y)
    #print(canvas.find_closest(x, y))

    if not capturing and not replaying:
    
        # Add a target
        targ = win.create_rectangle(x-target_size,y-target_size,x+target_size,y+target_size, fill="red")

        rx,ry = screen_to_rob(x,y)
        print("Add target %f,%f"%(rx,ry))
        targets.append({"x":rx,"y":ry,"rect":targ})


    


def initiate_move():
    """ Initiate a move to the next target, assuming we are not already moving."""

    global targets
    global moving

    # Initiate move to the next target
    robot.controller(0) # Make this a null field first (because we will be updating the position)
    rx,ry=targets[0]["x"],targets[0]["y"]
    print("Initiate move to %f,%f"%(rx,ry))
    robot.move_to(rx,ry,robot_move_time)
    
    moving = True
        
    
        


def draw_trajectory():
    """ Draw the trajectory that we have captured. """
    global trajectory
    global trajectory_display
    
    if len(trajectory)==0:
        print("No trajectory to display")
        return
    else:
        print("Displaying trajectory with %i samples"%len(trajectory))

    # Remove the old trajectory lines from the canvas (if any)
    if trajectory_display!=None:
        win.delete(trajectory_display)
    
    coords = [ rob_to_screen(x,y) for (x,y) in trajectory[::10] ] # downsample the list a little and convert to screen coordinates
    
    trajectory_display = win.create_line(*coords,fill="green",width=2)
    










def smooth_window(x,window):
    """Smooth the data using a window with requested size.
    
    This method is based on the convolution of a scaled window with the signal.
    The signal is prepared by introducing reflected copies of the signal 
    (with the window size) in both ends so that transient parts are minimized
    in the begining and end part of the output signal.

    Arguments
        x: the input signal 
        window: the window function (for example take numpy.hamming(21) ) 

    output:
        the smoothed signal
        
    original source:  http://scipy-cookbook.readthedocs.io/items/SignalSmooth.html 
    adjusted by FVV to make Python3-compatible and ensure that the length of the output is the same
    as the input.
    """

    wl = len(window)
    if x.ndim != 1: raise ValueError( "smooth only accepts 1 dimension arrays.")
    if x.size < wl: raise ValueError("Input vector needs to be bigger than window size.")
    if wl<3: return x

    # Pad the window at the beginning and end of the signal
    s=np.r_[x[wl-1:0:-1],x,x[-2:-(wl+1):-1]]
    # Length of s is len(x)+wl-1+wl-1 = len(x)+2*(wl-1)
 
    ## Convolution in "valid" mode gives a vector of length len(s)-len(w)+1 assuming that len(s)>len(w) 
    y=np.convolve(window/window.sum(),s,mode='valid')
    
    ## So now len(y) is len(s)-len(w)+1  = len(x)+2*(wl-1) - len(w)+1
    ## i.e. len(y) = len(x)+len(w)-1
    ## So we want to chop off len(w)-1 as symmetrically as possible
    frontw = int((wl-1)/2)   # how much we want to chop off on the front
    backw  = (wl-1)-frontw   # how much we want to chop off on the back
    return y[frontw:-backw]


def smooth(x):
    """ Smooth the signal x using the specified smoothing window. """
    return smooth_window(np.array(x),SMOOTHING_WINDOW)

    
    

def routine_checks():
    """ Stuff we need to keep doing. """
    # Check if a move is completed, if so, have the robot stay put.
    global targets
    global moving
    global done_targets
    global capturing
    global replaying

    if replaying:
        if robot.replay_is_done():
            print("Detected end of replay.")
            replaying = False
            robot.stay()
        return
    
    
    if capturing:
        # If we are currently capturing a movement
        t = time.time()
        if t-capture_start > CAPTURE_DURATION:
            print("Capturing complete")
            robot.controller(0)
            raw_traj = list(robot.retrieve_trajectory()) # retrieve the captured trajectory from the robot memory

            # Smooth it
            x,y = zip(*raw_traj)
            xfilt,yfilt = smooth(x),smooth(y)

            global trajectory
            trajectory = list(zip(xfilt,yfilt))
            # Re-insert the filtered trajectory into the robot
            robot.prepare_replay(trajectory) # push the trajectory back to robot memory for replaying (and set the final positions appropriately)

            draw_trajectory()
            capturing = False
            
    
    if moving: # If we are currently in the middle of moving to a target

        if robot.move_is_done(): # check if we are done

            # Remove the last target
            if len(targets)>0: # this should always be true actually
                oldtarg = targets[0]
                if oldtarg["rect"]!=None:
                    win.itemconfig(oldtarg["rect"],fill="gray")
                    done_targets.append(oldtarg)
                #win.delete(oldtarg["rect"]) # remove rect from the interface
                targets = targets[1:] # remove this target from our stack

            moving = False

            # If now there is no more target to go to
            if len(targets)==0:
                robot.stay()

    if not moving and not capturing:
        if len(targets)>0:
            initiate_move()

    

def on_closing():
    #if messagebox.askokcancel("Quit", "Do you want to quit?"):
    #    master.destroy()
    global keep_going
    keep_going = False
    master.destroy()
    robot.unload()





    
    
def release():
    """ Release the robot (null field)."""

    global done_targets
    global targets
    global moving
    moving = False
    oldtargets = targets[:]
    targets = []
    
    # In case the robot might be actively moving, it will be good
    # to first hold it at the current position for a moment and then
    # release it, to make sure that it is still when released.
    robot.stay()
    t0 = time.time()
    while time.time()-t0 < 1.: # wait one second
        pass
    robot.controller(0) # null field

    # Remove the targets from the interface
    for targ in oldtargets+done_targets:
        win.delete(targ["rect"])
        
    targets = []
    




def capture():
    """ This will capture the robot position """
    global capturing
    global trajectory
    global capture_start

    if not capturing and not replaying:

        if moving:
            release() # we have to be in a null field to be capturing

        # Capture the trajectory
        trajectory = robot.start_capture()
        capture_start = time.time()
        capturing = True
        print("Initiated capturing")
        
    else:
        print("Already capturing or replaying!")
    



def to_start():
    """ Go to the starting point of the trajectory. """

    global targets
    
    if len(trajectory)==0:
        print("No trajectory to replay!")
        return

    release()

    global targets
    rx,ry = trajectory[0] # the starting point of the trajectory
    targets = [{"x":rx,"y":ry,"rect":None}]
    initiate_move()



def replay():
    """ Replay the captured trajectory """

    global replaying
    global trajectory
    
    if replaying:
        print("Already replaying.")
        return

    if len(trajectory)==0:
        print("No trajectory to replay!")
        return

    print("Starting replay now.")
    robot.start_replay()
    replaying = True



    
        
# Set up the main interface scree
    
master = Tk()
master.geometry('%dx%d+%d+%d' % (w, h, 500, 200))



buttonframe = Frame(master) #, padding="3 3 12 12")
buttonframe.grid(column=0, row=0)
releaseb = Button(buttonframe,text="null field",    command=release) .grid(column=0, row=0, sticky=W)
captureb = Button(buttonframe,text="capture (3sec)",command=capture) .grid(column=1, row=0, sticky=W)
startb   = Button(buttonframe,text="to start"      ,command=to_start).grid(column=2, row=0, sticky=W)
replayb  = Button(buttonframe,text="replay",        command=replay)  .grid(column=3, row=0, sticky=W)

win = Canvas(master, width=w, height=h)
win.bind("<Button-1>", mouseclick) # click callback
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
coord_txt = win.create_text(cw,ch,text='NA,NA',fill='blue',anchor='w')

master.protocol("WM_DELETE_WINDOW", on_closing)




def draw_robot():
    # Update the cursor that indicates the current position of the robot
    rx,ry = robot.rshm("x"),robot.rshm("y")
    #print(rx,ry)
    x,y = rob_to_screen(rx,ry)
    win.coords(robot_pos,(x-cursor_size,y-cursor_size,x+cursor_size,y+cursor_size))
    win.itemconfig(coord_txt,text="  %.3f,%.3f"%(rx,ry))
    win.coords(coord_txt,(x,y))
    global moving
    robcol = "green" if moving else "blue"
    win.itemconfig(robot_pos,fill=robcol)





    

keep_going = True
robot.wshm('plg_moveto_done',1)

while keep_going:

    draw_robot()
    routine_checks()

    master.update_idletasks()
    master.update()
    #time.sleep(0.01) # frame rate of our GUI update

    
