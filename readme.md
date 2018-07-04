# Inmotion2-Python Robot

This is an attempt to make a simple interface for the InMotion2 Robot at the Motor Control Lab at McGill. It uses Python instead of Tcl, but keeps C for the code that actually controls the robot directly. All of this is built for **Python 3**.


This branch is intended to work with Xenomai 2.6 under Linux 3.18.20.



## Requirements

Python 3 with the following modules:

* `subprocess` (might already be included in your Python installation by default).
* `sysv_ipc` (for native Python access to the shared memory)

I think these can be straight-forwardly be installed using `pip <module-name>`.

That's it! 



## Usage

Full documentation can be found through Doxygen (to generate it type `make doxygen` in the root folder).

The robot C scripts are included in the subdirectory `robot/` and these need to be compiled. This, as well as some smaller administrative tasks, can be done by invoking the following command from the prompt (from the parent directory to `robot/`):

```
make
```

You can then write a script that controls the robot. A simple example is here:


```python
import robot.interface as robot

robot.load()

robot.move_to(.2,.1,5.) # move to point (x=.2,y=.1) in t=5 seconds.
time.wait(6) # wait roughly until the movement completes

robot.unload()
```



## Logging

Start writing a binary log to file using `robot.start_log('log.txt',n)` where `n` is the number of columns (this has to match the way you have defined columns in your `robot/pl_ulog.c`). Stop logging using `robot.stop_log()`. Logs are written in binary format and you can unpack them easily if you know the order in which columns are written (and how many there are).



## Files

I recommend keeping all the robot C and Python code in `robot/` and put experiment-specific stuff in the parent directory.

* `robot/` -- the C code for the robot (as well as some Python scripts for shared memory access).
* `robot/interface.py` -- the main robot module.
* `robot/shm.py` -- infrastructure for accessing the shared memory.

Example programs:

* `example_simple.py` -- loads the robot and prints the position to the screen, repeatedly.
* `example_proto.py` -- loads the robot and reads position and moves it to a different position.
* `example_viewpos.py` -- example of a GUI interface in which you see the robot handle position and can click to move it to new locations.
* `example_log.py` -- writes an example log file.
* `example_replay.py` -- captures a trajectory and then plays it back.
* `dump_shm.py` -- this reads all variables it knows about from the shared memory together with their value (great for taking a "snapshot" of the current config).
* `readlog/readlog.py` -- this is an example script that can read a robot log.
* `shm_sandbox/shm_ext.py` -- old-style shm module (can be loaded instead of the preferred `shm`) which mimicks previous Tcl code by communicating with the C program `shm`.

Note that you also need a build environment where robot code can be built. In other words, this won't simply work at your home computer, because you will need libraries that are installed, for example in `/opt` on the robot computer.




## Shared memory (shm)

The robot is controlled by a C program whose code is in `robot/`. Since we use a Python script to run the experiment itself (e.g. present trials, switch on or off force fields, etc.) these two programs need to be able to talk to each other. This happens through the shared memory, which is currently implemented as *System V* ([doc](http://fscked.org/writings/SHM/shm-5.html)).

A number of C objects are put in the shared memory, and these are defined in `robot/robdecls.h` and `robot/userdecls.h`. The previous Tcl code for the robot invoked another C script and then asked it to return particular variables in the shared memory. The current Python implementation reads the shared memory directly.

The interface is fairly straightforward. Let's say you want, within Python, to read or write to a variable `plg_stiffness` defined in the structure `Ob` in `robdecls.h`. From within Python, you can access this variable with `rshm('plg_stiffness')` or write a value to it using `wshm('plg_stiffness,4000)`. That's all there is to it!

If you want to access variables that are embedded in objects, say variable `x` that is a variable declared in `pos` (of type `xy`) in `Ob`, then you can access this variable using `rshm('pos_x')`. In other words, what would be dots in C are turned into underscores when calling the variables within the Python interface (this is, again, done for backwards compatibility with the previous Tcl implementation).

If you want to access arrays, you need to specify which element of an array you want. For example, if you want to read the 4-th element from an array `ft_bias`, then you call `rshm('ft_bias',3)`. Similarly for writing, but careful, the index you are assigning to comes as the last argument, i.e. if you want to set the aforementioned element to `8.9` then you call `wshm('ft_bias',8.9,3)`.



### Under the hood

The shared memory system itself in Python is quite easy thanks to the module `sysv_ipc` ([doc](http://semanchuk.com/philip/PythonIpc/)). It allows you to access the chunk of memory and read or write data with a particular offset. The tricky thing is to find out where in that chunk particular variables are, because you can't call them by name but only by byte offset. In other words, you need to figure out how C stores its objects. It would have been ideal if the shared memory were allocated in a more structured fashion (i.e. to have specifications where particular variables are to be found in the shared memory chunk). Since we don't have that, one solution is to have a middle-man C script that we can talk to that will give us these variables or write to them on demand. But in order to move towards the specification-approach I mentioned before, I decided for the following "intermediate" solution. 

A python script `parse_robdecls.py` parses the `robdecls.h` script and pulls out the type definitions. For a few types of interest, the script lists their variables and generates a C script that will tell us the memory location offsets of the subtypes (see the `Makefile` in `robot/` for details). These memory locations are then stored in a file `field_addresses.txt` which the Python shared memory module reads in and uses to find the byte offsets for variables in the shared memory chunk.

However, some of the naming of variables in the previous Tcl implementation was quite erratic. For example, a variable was called from the shared memory by the name `have_csen` which actually referred to the C varialbe `csen.have`. Why these words were sometimes flipped (and sometimes not) remains a mystery to me but for backwards compatibility I decided to create infrastructure that could deal with these, until the time that a gentle soul spends an afternoon recoding these names properly. Until that time, aliases such as these are defined in `robot/alias_list.txt` and these are read by the Python script at run time.



### Some examples

If your robot process is running, then shared memory chunks will be allocated and Python can access them. Here is an example:

```python
from shm import *
start_shm()
st = rshm('plg_stiffness')
print(st)
```

If you are looking up a variable, `get_info()` can be useful, as it gives information about where Python thinks the data is to be found.
```python
get_info('plg_stiffness')
```





## TODO

- [ ] Clean up the code that kills the robot if it already exists during launch.







## InMotion2 programmer's notes

This comes originally from a file called `ABOUT` in the original robot code. It looked useful.

InMotion2 programmer's notes

Copyright 2003-2005 Interactive Motion Technologies, Inc.
Cambridge, MA, USA
http://www.interactive-motion.com
All rights reserved

Wed Aug  4 16:13:50 EDT 2004

There is a web page with links to documents describing
the InMotion2 system at:

```
/opt/imt/robot/crob/notes/index.html
```

These are the C files that make up the
IMT Robot control LKM (Linux Kernel Module).

Headers

* `robdecls.h` - contains most of the LKM's declarations
* `rtl_inc.h` - RTLinux system include
* `uei_inc.h` - UEI specific
* `userfn.h` - user definable LKM functions

Implementation

* `main.c` - the main loop
* `sensact.c` - sensors and acutators
* `math.c` - matrix ops
* `uei.c` - UEI PowerDAQ i/o
* `fifo.c` - fifo i/o between LKM and user space
* `write.c` - fifo printf stuff.
* `slot.c` - slot handling
* `ulog.c` - user definable logging functions (FVV Feb 2017: no longer found this file)
* `uslot.c` - user definable slot functions (FVV Feb 2017: no longer found this file)
* `isaft.c` - ATI ISA force transducer
* `pc7266.c` - US Digital incremental encoders

Note: in earlier versions of the software, `sensact.c` was called
`control.c` , and `ulog.c` and `uslot.c` were together in userfn.c .

The RTLinux programming model dictates that the programmer code a main
control loop running in a Linux Kernel Module (LKM) thread.

The LKM is invoked by loading it with the Linux command, `/sbin/insmod`.
When an LKM is done loading, its `init_module()` is called.  This is
akin to `main()` being called when a user-mode C program is run.

`init_module()` sets up some data structures,
creates a new thread with `pthread_create()`,
and starts running our `start_routine()` in the new thread.

`start_routine()` sets up a periodic timer to wake up at the
sampling frequency we request (let's say, 200 Hz),
and invokes the `main_loop()`.

`main_loop()` will run until the LKM is unloaded.  This is the order of
tasks that take place during each sample in the main loop:

* check exit conditions (late and quit)
* read sensors

* read references (from a file)
* compute control outputs
* check safety
* write control outputs
* write log data
* write display data

* wait for next tick

The main loop is controlled by a "paused" variable.  If the loop is
paused, none of the control tasks are performed.  No data is read or
written, to or from the robot arm, user tasks, or system display.
This is a software on/off switch.  When the loop is paused, all we do
is wait for the next tick and check to see if the loop is still
paused.

Some minor time data structures do get modified, telling you how long
the loop has been running, but no data goes to or from the robot while
the loop is paused.



Optional sections of the main loop are controlled by function call
variables.  These are specified in the main loop with functions calls 
of the form:

```
call_xxx();
```

For instance, you might want to customize the logfile writing function
called in the main loop.  This function is call_write_log().

`call_write_log()` looks like this:

```
if (func.write_log)
func.write_log();
```

This means, if the variable func.write_log is non-zero, then call it.
All the members of the struct func are pointers to functions.
In userfn.c, there is a line:

```
func.write_log = write_data_fifo_sample_fn;
```

which means that when call_write_log() is invoked above, the function
write_data_fifo_sample_fn() will be called.  If userfn.c said:

```
func.write_log = NULL;
```

Then call_write_log() would return without doing anything.
You can modify userfn.c to make func.write_log point to whatever
function you choose, and in this way, you can modify the behavior of
the main loop without actually modifying main.c

For further information, see the html documentation at:

```
/opt/imt/robot/crob/notes/index.html
```
