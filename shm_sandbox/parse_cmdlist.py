

# cmdlist.tcl is a file that specifies which variables are in the shared memory and in what order.

f = open('../robot/cmdlist.tcl','r')
cmdl = f.read()
f.close()



# A more readable table of commands
cmds = []

# Not good code at all; just so that we can keep going!
for ln in cmdl.split("\n"):

    ln = ln.strip()

    if len(ln)==0 or ln[0]=="#":
        continue

    if ln == "set cmdlist {" or ln =="}":
        continue

    if ln[0]=="{" and ln[-1]=="}":
        elts = ln[1:-1].split()

        if len(elts)==4:
            elts[0] = elts[0][3:]
            cmds.append(" ".join(elts))

        else:
            print("ERROR",ln)
        



f = open('cmdlist.txt','w')
f.write("\n".join(cmds))
f.close()


