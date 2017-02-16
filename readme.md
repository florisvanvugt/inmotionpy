# Simple Robot

This is an attempt to make a simple interface for the InMotion2 Robot at the Motor Control Lab at McGill.

I plan to use Python instead of Tcl, but keep using C for the code that actually controls the robot directly.





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
