
import time
import pygame
from Tkinter import * # use for python2
import tkMessageBox
import tkFileDialog
#from tkinter import * # use for python3
import numpy as np
import random
import datetime

import robot
import pickle



# Controlling the subject screen
SUBJ_SCREENSIZE = (1920,1080)
SUBJ_SCREENPOS = (1600,0) # the offset (allows you to place the subject screen "on" the second monitor)



#N_HORIZ_VISUAL_TARGETS,N_VERTIC_VISUAL_TARGETS = 10,5
N_HORIZ_VISUAL_TARGETS,N_VERTIC_VISUAL_TARGETS = 3,3
TARGET_REPETITIONS = 2 # how often to present each target

N_CAPTURE = 50 # how many data points to capture (and average) for each target
CAPTURE_SLEEP = .005 # how long to sleep between captures

HORIZ_PAD = 400
VERTIC_PAD = 300


# The target circle radius
TARGET_RADIUS = 20

# When we draw a cursor following the robot handle position, this controls its radius
CURSOR_RADIUS = 10




# The little control window
CONTROL_WIDTH,CONTROL_HEIGHT= 700,400 # control window dimensions
CONTROL_X,CONTROL_Y = 800,500 # controls where on the screen the control window appears





def init_pygame():

    pygame.init()

    import os
    os.environ['SDL_VIDEO_WINDOW_POS'] = "%d,%d" % SUBJ_SCREENPOS # controls where the subject screen will appear

    global subjscreen
    subjscreen = pygame.display.set_mode(SUBJ_SCREENSIZE,pygame.NOFRAME)
    subjscreen.fill((0,0,0))
    pygame.display.flip()




    

def draw_target(target):
    (x,y) = target

    global subjscreen
    
    subjscreen.fill((0,0,0))
    updaterec = pygame.draw.circle(subjscreen,(255,0,0),(x,y),TARGET_RADIUS)
    pygame.display.flip()
    




def draw_cursor(sx,sy):
    global subjscreen,oldrect
    subjscreen.fill((0,0,0))

    updateold = None
    if oldrect!=None: # oldrect holds the rect where we last drew a cursor (we'll have to remove it)
        updateold = pygame.draw.rect  (subjscreen,(0,0,0),oldrect) # remove the previous cursor (we simply draw a rect over it because that is faster)
    updaterec = pygame.draw.circle(subjscreen,(255,255,0),(sx,sy),CURSOR_RADIUS)
    pygame.display.update([updaterec,updateold])
    oldrect = updaterec
    
    


    

def load_robot(e):
    robot.load()



def zeroft(e):
    robot.zeroft()

    bias = robot.bias_report()
    tkMessageBox.showinfo("Bias report", "Robot bias settings:\n\n" +" ".join([ str(b) for b in bias ]))
    

    
def unload_robot(e):
    stop_following(e)
    robot.unload()



def visual_calibrate(e):
    """
    Here we will present a series of visual targets (on the screen)
    and then ask the user to move to them, capture their position,
    and then later compute a regression to map screen coordinates to 
    robot coordinates.
    """

    targs = [ (int(x),int(y))
              for x in np.linspace(HORIZ_PAD, SUBJ_SCREENSIZE[0]-HORIZ_PAD, N_HORIZ_VISUAL_TARGETS)
              for y in np.linspace(VERTIC_PAD,SUBJ_SCREENSIZE[1]-VERTIC_PAD,N_VERTIC_VISUAL_TARGETS)
    ]
    
    global targets
    targets = []
    for _ in range(TARGET_REPETITIONS):
        random.shuffle(targs)
        targets += targs[:]

    global current_target,captured
    current_target = -1
    next_target(e)

    captured = []
    





def linregr(x,y):
    """ Find a linear regression, i.e. slope and intercept, to predict y based on x."""
    A = np.vstack([x, np.ones(len(x))]).T
    slope, interc = np.linalg.lstsq(A, y)[0]
    return (slope,interc)
    
    

def wrap_calibration():
    global targets,current_target
    global gui

    gui["target"].set("Targets complete")

    global captured
    scr_x,scr_y,rob_x,rob_y = zip(*captured)

    regrs = {}
    for (var,screenvar,robotvar) in [("x",scr_x,rob_x),
                                     ("y",scr_y,rob_y)]:
        sl,interc = linregr( robotvar, screenvar )
        regrs["slope.%s"%var]  = sl
        regrs["interc.%s"%var] = interc

    subjid = gui["subject.id"].get()

    timestamp = datetime.datetime.now().strftime("%Y%d%m_%Hh%Mm%S")
    
    fname = "calib_%s_%s.pickle27"%(subjid,timestamp)
    pickle.dump((captured,regrs),open(fname,'wb'))

    print("Robot -> Screen regression: ",regrs)
    
    global calib
    calib = regrs
    
    


def robot_to_screen(x,y):
    """ Given robot coordinates, find the screen coordinates. """
    global calib
    return (int(calib["interc.x"] + (x*calib["slope.x"])),
            int(calib["interc.y"] + (y*calib["slope.y"])))



def follow_robot():
    """ Draw the robot position on the screen, repeatedly."""
    global calib,master,gui,oldrect
    oldrect = None # just so that it is initialised
    
    gui["keep_going"] = True # set gui["keep_going"] to False to drop out of this and stop following the robot handle
    while gui["keep_going"]:
        rx,ry = robot.rshm('x'),robot.rshm('y')
        sx,sy = robot_to_screen(rx,ry)

        draw_cursor(sx,sy)
        
        master.update_idletasks()
        master.update()


        

def next_target(e):

    global targets,current_target
    current_target+= 1

    if current_target>=len(targets):
        wrap_calibration()
        follow_robot()
        return
        
    # Now wait for the subject to go there
    target = targets[current_target]
    print("Presenting target",target)
    draw_target(target)

    global gui
    gui["target"].set("target %i/%i"%(current_target+1,len(targets)))

    


def capture(e):
    capt = []
    for _ in range(N_CAPTURE):
        x,y = robot.rshm('x'),robot.rshm('y')
        capt.append((x,y))
        time.sleep(CAPTURE_SLEEP)
    
    # Capture the position
    allx,ally=zip(*capt)
    capx,capy=np.mean(allx),np.mean(ally)

    global captured,current_target,targets
    targx,targy=targets[current_target]
    captured.append( (targx,targy,capx,capy) )

    # Show in the interface what we captured
    global gui
    gui["position"].set("x=%.3f y=%.3f"%(capx,capy))
    
    next_target(e)


        

def load_calib(e):

    fname = tkFileDialog.askopenfilename(filetypes=[('pickles','.pickle27')])
    if fname!=None:
        print("Opening",fname)

        (captured,regrs) = pickle.load(open(fname,'rb'))
        global calib
        calib = regrs
        follow_robot()
        return
        

    
def stop_following(e):
    global gui
    gui["keep_going"]=False
    
    

def init_tk():
    global gui
    gui = {}
    
    master = Tk()
    master.geometry('%dx%d+%d+%d' % (CONTROL_WIDTH, CONTROL_HEIGHT, CONTROL_X, CONTROL_Y))
    master.configure(background='black')

    
    f = Frame(master,background='black')
    b1    = Button(f, text="Load robot",                background="green",foreground="black")
    b2    = Button(f, text="Zero FT (release handle)",  background="green",foreground="black")
    calibb= Button(f, text="Visual calibrate",          background="blue",foreground="white")
    stopb = Button(f, text="Stop following",            background="black",foreground="red")
    captb = Button(f, text="Capture",                   background="blue",foreground="white")
    loadb = Button(f, text="Load calib",                background="blue",foreground="white")
    b3    = Button(f, text="Unload robot"  ,            background="green",foreground="black")
    b4    = Button(f, text="Quit",                      background="red",foreground="black")
    b1.bind('<ButtonRelease-1>',load_robot)
    b2.bind('<ButtonRelease-1>',zeroft)
    calibb.bind('<ButtonRelease-1>',visual_calibrate)
    stopb.bind('<ButtonRelease-1>',stop_following)
    loadb.bind('<ButtonRelease-1>',load_calib)
    captb.bind('<ButtonRelease-1>',capture)
    b3.bind('<ButtonRelease-1>',unload_robot)
    b4.bind('<ButtonRelease-1>',endprogram)
    #b1.pack(side=LEFT)
    #b2.pack(side=LEFT)
    #b3.pack(side=LEFT)
    gui["position"] = StringVar()
    gui["target"]   = StringVar()
    gui["subject.id"] = StringVar()
    posl  = Label(f, textvariable=gui["position"],fg="white",bg="black")

    l    = Label(f, text="subject ID",fg="white",bg="black")
    subjid = Entry(f, textvariable=gui["subject.id"],fg="white",bg="black")
    
    targl = Label(f, textvariable=gui["target"],  fg="white",bg="black")

    row = 0
    f.grid         (column=0,row=row,padx=10,pady=10)
    row += 1
    b1.grid        (row=row,sticky=W,pady=10)
    row += 1
    b2.grid        (row=row,sticky=W,pady=10)
    row += 1
    l.grid         (row=row,column=0,sticky=W,pady=10)
    subjid.grid    (row=row,column=1,sticky=W,padx=10)
    loadb.grid     (row=row,column=2,sticky=W,padx=20)
    
    row +=1
    calibb.grid    (row=row,sticky=W,pady=10)
    captb.grid     (row=row,column=1,sticky=W,pady=10)
    targl.grid     (row=row,column=2,sticky=W,padx=10)
    posl.grid      (row=row,column=3,sticky=W,padx=10)
    stopb.grid     (row=row,column=4,sticky=W,padx=10)
    row += 1
    b3.grid        (row=row,sticky=W,pady=10)
    row += 1
    b4.grid        (row=row,sticky=W,pady=10)
    

    # Make some elements available for the future
    gui["position"].set("x=[] y=[]")
    gui["target"].set("Target NA/NA")
    gui["keep_going"]=False
    
    master.bind()

    return master
    #win.bind("<Button-1>", callback) # click callback
    



def init():
    init_pygame()
    init_tk()




def endprogram(e):
    stop_following(e)
    pygame.quit()
    sys.exit(0)
    
    



master = init_tk()
init_pygame()

master.mainloop()

#pygame.time.wait(3000)
#quit()



