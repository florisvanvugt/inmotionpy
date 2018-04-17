from tkinter import * # python3
import robot.interface as robot
import time
import random

from tkinter.font import *


robot.load()


root = Tk()
root.title("Variable display")
root.geometry('%dx%d+%d+%d' % (400, 600, 600, 200))
font = nametofont("TkDefaultFont")
font.configure(size=30)

Grid.rowconfigure(root, 1, weight=1)
Grid.columnconfigure(root, 0, weight=1)
Grid.columnconfigure(root, 1, weight=1)

# These are the variables that we will check
variables = []
v = StringVar()
varbox = Listbox(root)
valbox = Listbox(root)



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
    # For each variable, show its value
    
    valbox.delete(0,END) # remove everything
    i = 0
    for i,v in enumerate(variables):
        val = robot.rshm(v)
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


root.option_add("*Font",font)
root.protocol("WM_DELETE_WINDOW", on_closing)



keep_going = True
#robot.wshm('plg_moveto_done',1)

while keep_going:

    update_display_values()

    root.update_idletasks()
    root.update()
    
    time.sleep(0.05) # frame rate of our GUI update

