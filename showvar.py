from tkinter import * # python3
import robot.interface as robot
import time
import random

from tkinter import messagebox
 
robot.load()


root = Tk()
root.title("Variable display")
root.geometry('%dx%d+%d+%d' % (400, 600, 600, 200))

Grid.rowconfigure(root, 1, weight=1)
Grid.columnconfigure(root, 0, weight=1)
Grid.columnconfigure(root, 1, weight=1)

# These are the variables that we will check
variables = []
v = StringVar()
varbox = Listbox(root)
valbox = Listbox(root)


def addvariable():
    varname = v.get()

    # Get info about this variable
    varinfo = robot.get_info(varname)
    if varinfo==None:
        messagebox.showinfo("Unknown variable", "Unknown variable.")
        return

    if varname in variables: # we already have that variable
        messagebox.showinfo("Variable already in list", "That variable is already in the list.")
        return
    
    v.set("") # empty the original inbox
    variables.append(varname)
    # TODO: check that it's a real variable and that it's not already in our list
    #print(variables)
    #print("Click!")
    varbox.insert(END, varname)




def update_display_values():
    # For each variable, show its value
    
    valbox.delete(0,END) # remove everything
    i = 0
    for i,v in enumerate(variables):
        val = robot.rshm(v)
        #val = random.random()
        valbox.insert(END,str(val))
        if val<0:
            valbox.itemconfig(0, foreground="red")
        else:
            valbox.itemconfig(0, foreground="green")

        
    
b = Entry(root,textvariable=v)
b.grid(row=0,column=0)

b = Button(root, text="add", command=addvariable)
b.grid(row=0,column=1)

varbox.grid(row=1,column=0,sticky=N+E+S+W)
valbox.grid(row=1,column=1,sticky=N+E+S+W)




def on_closing():
    global keep_going
    keep_going = False
    root.destroy()
    robot.unload()

root.protocol("WM_DELETE_WINDOW", on_closing)



keep_going = True
#robot.wshm('plg_moveto_done',1)

while keep_going:

    update_display_values()

    root.update_idletasks()
    root.update()
    
    time.sleep(0.05) # frame rate of our GUI update

