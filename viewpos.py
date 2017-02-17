from tkinter import *

import time
import robot



# Some parameters that specify how we draw things onto our window
w,h = 800,600
cw,ch = w/2,h/2
robot_scale = 700
cursor_size = 10

def rob_to_screen(x,y):
    # Convert robot coordinates into screen coordinates
    return (cw + x*robot_scale,ch + y*robot_scale)


def screen_to_rob():
    # Convert screen coordinates into robot coordinates
    pass


def callback(event):
    # When the mouse is clicked, show where we are
    canvas = event.widget
    x = canvas.canvasx(event.x)
    y = canvas.canvasy(event.y)
    print(canvas.find_closest(x, y))


def on_closing():
    #if messagebox.askokcancel("Quit", "Do you want to quit?"):
    #    master.destroy()
    global keep_going
    keep_going = False
    master.destroy()
    robot.unload()







    
master = Tk()
master.geometry('%dx%d+%d+%d' % (w, h, 500, 200))


win = Canvas(master, width=w, height=h)
win.pack()

# Draw the background against which everything else is going to happen
win.create_rectangle(0, 0, w, h, fill="black")
minx,miny = rob_to_screen(-.4,-.1)
maxx,maxy = rob_to_screen( .4,.3)
win.create_rectangle(minx,miny,maxx,maxy, outline="blue")

robot_pos = win.create_oval(cw,ch,cw,ch,fill="blue")

master.protocol("WM_DELETE_WINDOW", on_closing)


def draw_robot():
    # Update the cursor that indicates the current position of the robot
    rx,ry = robot.rshm("x"),robot.rshm("y")
    print(rx,ry)
    x,y = rob_to_screen(rx,ry)
    win.coords(robot_pos,(x-cursor_size,y-cursor_size,x+cursor_size,y+cursor_size))






    

keep_going = True
    
robot.load()
robot.bias_force_transducers()
robot.status()

while keep_going:

    draw_robot()
    
    master.update_idletasks()
    master.update()
    time.sleep(0.01) # frame rate of our GUI update

    
