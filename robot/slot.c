// slot.c - code to handle/schedule slot controllers
// part of the robot.o Linux Kernel Module

// InMotion2 robot system software for RTLinux

// Copyright 2003-2005 Interactive Motion Technologies, Inc.
// Cambridge, MA, USA
// http://www.interactive-motion.com
// All rights reserved

#include "rtl_inc.h"
#include "ruser.h"
#include "robdecls.h"

// args: id, init, term, incr, fn, tag.
// (init, term, incr) order a la C for loop.
// args must be unsigned.
//
// if you want backwards stepping, do the math yourself,
// with term - i
//
// to do 10 samples:
// slot(sid, 0, 10, 0, fnid);
//
// to do 10 seconds:
// slot(sid, 0, 10*ob->Hz, 1, fnid);
//
// to do until done:
// slot(sid, 0, 1, 0, fnid);
// (and set ob->slot.i when you're done).

// set ob->slot_max by hand.
// if you have one slot, with id=0, set slot_max to 1.

// chaining slot controllers?
// when you are finished, replace yourself with a new one?

// call ob->slot_fns[ob->slot[id].fnid](id)
// if slot is active.
//
// note the if statement comparing i < term.
// this means that if i==0 and term==10, [0,10)
// the interval is half-closed, i goes from 0 to 9,
// in usual C for loop fashion.
// if you want a different range, you can get it by setting
// i and term as you wish, or by hacking your controller.

void
do_slot(void)
{
    u32 id;
    u32 i;
    u32 zero_force;
    u32 ran_slot;

    // if we don't run any slots, make sure that motor_force
    // is zeroed here, so that it doesn't have vestigal values from a
    // previous cycle's safety damping etc.

    ran_slot = 0;
    zero_force = 0;

    // if slot_max is too bix, then don't run slots
    if (ob->slot_max > 8) {
	ob->slot_max = 0;
    }

    // for each slot[i]
    for (id = 0; id < ob->slot_max; id++) {
	// don't assign intermediate values to ob->slot[id].i
	i = ob->slot[id].i;
	// did user ask to stop?
	if (ob->slot[id].running == 0) {
	    // don't run it.
	    continue;
	}
	// is fnid outside the slot_fns array?
	if (ob->slot[id].fnid >= 32) {
	    // yes, something is wrong.
	    zero_force = 1;
	    break;
	}
	// if there is a function for it in slot_fns
	if (ob->slot_fns[ob->slot[id].fnid]) {
	    // call it with id as parameter.
	    ob->slot_fns[ob->slot[id].fnid](id);
	    ran_slot++;
	}

	// post-increment i
        i += ob->slot[id].incr;

	// when you get to the end of the slot,
	// let it run until canceled, with i = term.
	if (i > ob->slot[id].term) {
	    i = ob->slot[id].term;
	    // count samples after term,
	    // in case you want to increase stiffness or something.
	    ob->slot[id].termi++;
	}

	ob->slot[id].i = i;
    }
    if (zero_force || !ran_slot) {
	set_zero_torque();
    }
}

void
stop_slot(u32 id)
{
    memset(&(ob->slot[id]), 0, sizeof(Slot));
}

void
stop_all_slots(void)
{
    u32 id;

    for (id=0; id<8; id++)
	stop_slot(id);
}
