

# Let's read the robot log and do some exploration.

import subprocess
import struct
import numpy as np
import pandas as pd
import os



def read_trajectories(traj_f):

    fieldsize = 8  # bytes per variable
    variables = [("sample",          "d"), # yep, unfortunately even integers get stored as doubles
                 ("pos.x",           "d"),
                 ("pos.y",           "d"),
                 ("vel.x",           "d"),
                 ("vel.y",           "d"),
                 ("ft.world.x",      "d"),
                 ("ft.world.y",      "d"),
                 ("motor.force.x",   "d"),
                 ("motor.force.y",   "d"),
                 ("trial.phase",     "d"), # originally called fvv.trial.phase
                 ("trial",           "d"), # originally called fvv.trial.no
                 ("ft.world.z",      "d"),
                 ("grasp.force",     "d"),
    ]
    n_variables = len(variables)


    f = open(traj_f, "rb")
    #buff = f.read(6)

    samples = []

    entry_format = "".join(list(zip(*variables))[1])
    field_names = list(zip(*variables))[0]

    onelines = []

    keepGoing = True
    while keepGoing:
        # Read the contents

        desired_length = 8*n_variables
        oneline = f.read(desired_length)
        onelines.append(oneline)
        if len(oneline)!=desired_length:
            keepGoing=False
        else:
            sample = struct.unpack(entry_format,oneline)
            samples.append(sample)

    #print "End of file"
    f.close()



    #data= np.array(samples[0], dtype=np.dtype(variables))
    #data= np.array(samples[1], dtype=np.dtype(variables))

    d = np.array(samples, dtype=np.dtype(variables))
    data = pd.DataFrame(d)


    # data[['sample','fvv.trial.phase','fvv_trial.no']] = data[['sample','fvv.trial.phase','fvv_trial.no']].astype(int)
    return data






