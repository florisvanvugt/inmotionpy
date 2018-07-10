
import sys

PYTHON3 = (sys.version_info > (3, 0))
if PYTHON3:
    from tkinter import *
    from tkinter import messagebox as tkMessageBox
    from tkinter.font import *

else: # python2
    from Tkinter import * # use for python2
    from Tkinter.font import *
    import tkMessageBox


import robot.interface as robot
import time
import random


import tkinter.simpledialog as tkSimpleDialog # python 3


robot.load()

robot.bias_force_transducers()


capturing = True

root = Tk()
root.title("Variable display")
root.geometry('%dx%d+%d+%d' % (400, 600, 600, 200))
font = nametofont("TkDefaultFont")
font.configure(size=16)

# These are the variables that we will check
variables = []
v = StringVar()
varbox = Listbox(root)
valbox = Listbox(root)
varbox.configure(font=font)
valbox.configure(font=font)


def show_msg(msg):
    """ Show a message so that the text is selectable """
    win = Toplevel()
    win.geometry('%dx%d+%d+%d' % (300, 300, 200, 200))
    t = Text(win)
    t.insert(END,msg)
    t.config(state=DISABLED)
    t.pack(expand=True,fill='both')
    



def find_suggestions(varname):
    """ Find variables that contain a particular string."""
    suggestions = []
    for al in list(robot.varname_aliases.keys()):
        if al.find(varname)>-1:
            suggestions.append(al) #'- %s is an alias'%al)
    for l in list(robot.locs.keys()):
        if l.find(varname)>-1:
            suggestions.append(l) #'- %s is a variable'%l)
    return suggestions
    

    
def addvariable():
    varname = v.get()

    # Allow the use of wildcards -- if there is any asterisk anywhere,
    # just switch to searching mode. So it doesn't matter where you place
    # the asterisk.
    if varname.find("*")>-1:
        varname = varname.replace("*","")
        sugg = find_suggestions(varname)
        if len(sugg)>0:
            v.set("") # empty the original inbox
            for s in sugg:
                if not s in variables: # if not already there
                    variables.append(s)
                    varbox.insert(END,s)
        return
    
    # Get info about this variable
    varinfo = robot.get_info(varname)
    if varinfo==None:

        # Find some suggestions for variable names
        suggestions = [ '- %s'%s for s in find_suggestions(varname) ]

        msg = "Unknown variable '%s'"%varname
        if len(suggestions)>0:
            msg+="\nBut I have the following suggestions:\n"
            msg+="\n".join(suggestions)
        #messagebox.showinfo("Unknown variable", msg)
        show_msg(msg)
        return

    if varname in variables: # we already have that variable
        messagebox.showinfo("Variable already in list", "That variable is already in the list.")
        return
    
    v.set("") # empty the original inbox
    variables.append(varname)
    varbox.insert(END, varname)




def update_display_values():
    if capturing:

        # First, capture all the variables
        vals = [ robot.rshm(v) for v in variables ]
        
        # For each variable, show its value
        valbox.delete(0,END) # remove everything
        i = 0
        for i,val in enumerate(vals):
            #val = robot.rshm(v)
            if val==None:
                valbox.insert(END,"N/A")
                continue
            if type(val)is list:
                valbox.insert(END,"(array)")
                continue

            #val = random.random()
            valbox.insert(END,str(val))
            if val<0:
                valbox.itemconfig(i, foreground="red")
            else:
                valbox.itemconfig(i, foreground="green")



def toggle_capture():
    # Capture the state of variables
    global capturing
    capturing = not capturing;





def set_var(e):
    """ Attempt to set a variable (careful!) """
    idx, = varbox.curselection()
    varname = variables[ idx ]

    ## Get info about this variable
    varinfo = robot.get_info(varname)
    
    val = tkSimpleDialog.askstring("Set request", "Set %s to (CAREFUL!)"%varname)
    if val:
        print("Setting %s to %s"%(varname,val))
        val = robot.tryenc(val) ## trying to set encoding to something logical
        print("Converted to:")
        print("---")
        print(val)
        print("---")
        robot.wshm(varname,val)
              
            

def add(e): addvariable()
            
root.bind('<Return>',add)
e = Entry(root,textvariable=v)
e.grid(row=0,column=0)

b = Button(root, text="add", command=addvariable)
b.grid(row=0,column=1)

b = Button(root, text="start/stop", command=toggle_capture)
b.grid(row=1,column=1)

varbox.bind('<Double-Button-1>', set_var)
varbox.grid(row=2,column=0,sticky=N+E+S+W)
valbox.grid(row=2,column=1,sticky=N+E+S+W)

Grid.rowconfigure(root, 2, weight=1)
Grid.columnconfigure(root, 0, weight=1)
Grid.columnconfigure(root, 1, weight=1)




def on_closing():
    global keep_going
    keep_going = False
    root.destroy()
    robot.unload()


root.option_add("*Font",font)
root.protocol("WM_DELETE_WINDOW", on_closing)

e.focus_set()


#robot.controller(0) # null field, no dyn comp
#robot.controller(1) # null field, with dyn comp
#robot.start_curl(-15) # curl field, with dyn comp


#time.sleep(1)
#robot.start_damp(15) # viscosity field
robot.stay()



keep_going = True
#robot.wshm('plg_moveto_done',1)

while keep_going:

    update_display_values()

    root.update_idletasks()
    root.update()
    
    time.sleep(0.05) # frame rate of our GUI update

