// InMotion2 robot system software for RTLinux

// Copyright 2005 Interactive Motion Technologies, Inc.
// Cambridge, MA, USA
// http://www.interactive-motion.com
// All rights reserved

// rwarp.c - warp the cursor using the robot.
// makes the robot act like a 1-button mouse,
// warping the cursor and generating
// button1 press and release on grasp squeeze and release.
// uses Xlib XWarpPointer for motion
// and XTEST XTestFakeButtonEvent for button.

// assumes robot lkm is already loaded, a la shm
// may be killed safely, a la shm

// cc -Wall -o rwarp rwarp.c -L/usr/X11R6/lib -lX11 -lXtst

// X
#include <X11/Xlib.h>
#include <X11/extensions/XTest.h>

// make sure you run "go" to load lkm's first!

// mbuffs
// TODO: delete #include "/usr/src/linux-2.4.18-rtl3.1/rtlinux-3.1/include/mbuff.h"
// #include "/usr/src/rtlinux-3.2-pre3/include/mbuff.h"

#include "rtl_inc.h"

// // primitive typedefs
#include "ruser.h"

// robot decls
#include "robdecls.h"

// EXIT
#include <stdlib.h>

// signals
#include <signal.h>

Ob *ob;
Robot *rob;
Daq *daq;

int ob_shmid;
int rob_shmid;
int daq_shmid;

Display *display;

XEvent event;

void do_atexit(int sig)
{
    // TODO: delete mbuff_detach("ob", ob);
    // TODO: delete mbuff_detach("rob", rob);
    // TODO: delete mbuff_detach("daq", daq);
    shmdt(ob);
    shmdt(rob);
    shmdt(daq);
    // fprintf (stderr, "rwarp: exiting.\n");

    // don't call the atexit stuff again!
    _exit(0);
}

#define RELEASED 0
#define PRESSED 1

s32 main() {

    u32 screenw, screenh;
    s32 warpx, warpy;
    s32 lastx, lasty;
    f64 tablew, tableh, wscale, hscale, bigscale;

    u32 grasp_state;
    f64 grasp_force;

    Window root;

    // attach to robot linux kernel module data structures
    // assume that the lkm is already loaded.
    // TODO: delete ob = (Ob *) mbuff_attach("ob", (int)(sizeof(Ob)));
    ob_shmid = shmget(OB_KEY, sizeof(Ob), 0666);
    if (ob_shmid == -1) {
	fprintf (stderr, "rwarp: shmget() failed.\n");
	fprintf (stderr, "robot kernel process likely not running.\n");
	exit(EXIT_FAILURE);
    }
    ob = (Ob *)shmat(ob_shmid, NULL, 0);
    // TODO: delete rob = (Robot *) mbuff_attach("rob", (int)(sizeof(Robot)));
    // TODO: delete daq = (Daq *) mbuff_attach("daq", (int)(sizeof(Daq)));
    rob_shmid = shmget(ROB_KEY, sizeof(Robot), 0666);
    rob = (Robot *)shmat(rob_shmid, NULL, 0);
    daq_shmid = shmget(DAQ_KEY, sizeof(Daq), 0666);
    daq = (Daq *)shmat(daq_shmid, NULL, 0);

    // make sure it's sampling.
    ob->paused = 0;

    // do_atexit calls mbuff_free.
    // leaving mbuffs allocated is a bad thing.
    (void) signal(1, (__sighandler_t) do_atexit);
    (void) signal(2, (__sighandler_t) do_atexit);
    (void) signal(15, (__sighandler_t) do_atexit);
    (void) atexit((void *) do_atexit);

    // init X display data structures
    display = XOpenDisplay(NULL);

    if (display == 0){
	    fprintf(stderr, "could not open display");
	    exit(EXIT_FAILURE);
    }

    root = DefaultRootWindow(display);

    tablew = .4;
    tableh = .4;

    screenw = DisplayWidth(display, 0);
    screenh = DisplayHeight(display, 0);

    // if bigscale is bigger, you have to move the arm less.
    bigscale = 2;
    // flip y, and scale width to table height to make motions square
    wscale = screenw / tablew * bigscale;
    hscale = -wscale;

    lastx = lasty = 0;
    grasp_state = RELEASED;

    // loop, each cycle reads arm position and grasp sensor.
    // if position has moved, send motion event.
    // if grasp has crossed threshold, send button event.
    for (;;) {

	// first, handle motion.

	warpx = warpy = 0;
	if (ob->have_planar) {
	    warpx = ob->pos.x * wscale + screenw / 2;
	    warpy = ob->pos.y * hscale + screenh / 2;
	} else if (ob->have_wrist) {
	    warpx = ob->wrist.pos.fe * wscale + screenw / 2;
	    warpy = ob->wrist.pos.aa * hscale + screenh;
	}

	if (warpx < 0) warpx = 0;
	if (warpy < 0) warpy = 0;
	if (warpx > screenw) warpx = screenw;
	if (warpy > screenh) warpy = screenh;

	// no warp if mouse doesn't move;
	if (!(warpx == lastx && warpy == lasty)) {

	    lastx = warpx;
	    lasty = warpy;

	    // print
	    // printf("x = %f, y = %f, warpx %d, warpy %d, g = %f\n",
		// ob->pos.x, ob->pos.y, warpx, warpy, rob->grasp.force);

	    // warp
	    // XWarpPointer(display, src_w, dest_w, src_x, src_y,
	    //		src_width, src_height, dest_x, dest_y);
	    XWarpPointer(display, None, root, 0, 0,
			    0, 0, warpx, warpy);

	    XFlush(display);
	}

	// handle grasp
	// no events if no state change.

	// grasp_force = rob->grasp.force;
	grasp_force = daq->m_adcvolts[0][rob->grasp.channel];

	if (grasp_state == RELEASED) {
	    if (grasp_force > rob->grasp.press) {
		// printf("generate buttonpress %f\n", grasp_force);
		grasp_state = PRESSED;
		// press
		XTestFakeButtonEvent(display, 1, True, CurrentTime);

		XFlush(display);
	    }
	} else if (grasp_state == PRESSED) {
	    if (grasp_force < rob->grasp.release) {
		// printf("generate buttonrelease %f\n", grasp_force);
		grasp_state = RELEASED;
		// release
		XTestFakeButtonEvent(display, 1, False, CurrentTime);
		XFlush(display);
	    }
	}

	usleep(20000); // 50/sec
    }

    // not reached
    return 0;
}
