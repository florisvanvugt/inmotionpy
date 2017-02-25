
"""

This tries to connect to the shared memory and dumps all variables that we know of.

"""

from shm import *
start_shm()


f = open('alias_list.txt','r')
cmds = f.readlines()
f.close()


var_values = []
for c in cmds:
    c = c.strip()
    var = c.split(' ')[0].strip()

    try:
        res = rshm(var)
        #print(var,str(res))
        var_values.append((var,res)) #[var]=str(res)
        
    except:
        continue
    

for v,r in var_values:
    print(v,r)

    

stop_shm()
