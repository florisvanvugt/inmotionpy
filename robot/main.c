// main.c - InMotion2 main loop
// part of the robot.o Linux Kernel Module

// InMotion2 robot system software for RTLinux

// Copyright 2003-2005 Interactive Motion Technologies, Inc.
// Cambridge, MA, USA
// http://www.interactive-motion.com
// All rights reserved


/** \mainpage Porting the robot code
 * The aim of this project is to port the Inmotion-2 robot operating
 * code to a Linux 3.x kernel and Xenomai 2.6.
 */



// uncomment this for McGill environment
#define _ISMCGILL

#include "rtl_inc.h"

RT_TASK thread;
RT_TASK_INFO thread_info;

#include "ruser.h"

//#include "userdecls.h"

#include "robdecls.h"

#include "userfn.h"



// Xenomai has 1 ns resolution.  Multipliers: seconds 10^0,
// millisecond 10^-3, microsecond 10^-6, nanoseconds 10^-9.  

// sample tick 200 Hz, or 5 ms or 5,000,000 ns.

// tick rate can be reset with ob->restart.Hz and restart_init();

// Todo: delete #define BILLION (1000*1000*1000) // Xenomai resolution is 1 ns
// Todo: delete #define TIMER_PERIOD 5000 // 5000 us per Xenomai tick
// Todo: delete #define ROB_HZ 200  // We want to run 200 times a second
// Todo: delete #define ROB_PERIOD (BILLION/(TIMER_PERIOD*ROB_HZ))  // Delay this
						    // number of ticks
						    // each loop.
#define STACK_SIZE 8192
#define STD_PRIO 1

// ob storage definition
// the ob structure contains globals

Func func;

int ob_shmid;
int rob_shmid;
int daq_shmid;
int prev_shmid;
int game_shmid;
int moh_shmid;
int dyncmp_shmid;

Ob *ob;
Robot *rob;
Daq *daq;
Prev *prev;
Game *game;
Moh *moh;
Dyncmp_var *dyncmp_var;




/// Close the devices that are used for communication
void
cleanup_devices()
{
	// shut down devices here.
	uei_aio_close();
}




static u32 cleanup_module_in_progress = 0;
static u32 cleanup_module_main_loop_done = 0;



/**
  @brief cleanup_signal - stops our current process, quitting the robot
  
  cleanup_module may be called when the main loop is busy.
  if this happens, we must ensure that data structures are not deleted
  while they are in use.  for this reason, the main loop checks
  cleanup_module_in_progress, and sets cleanup_module_main_loop_done
  to let cleanup_module know that it is done, and cleanup may proceed
  safely.
*/

void
cleanup_signal(s32 sig)
{
  // TODO: Is the new circumstance analogous to old? Do we keep same
  // waiting architecture here? Would rt_task_delete() do the trick?
    s32 ret;

    dpr(3, "cleanup_module because of signal %d\n",sig);

    if (ob->quit) {
        rt_task_sleep(100);
    }
    // tell main_loop to stop doing i/o.
    cleanup_module_in_progress = 1;

    write_zero_torque();

    // delay for at least 2 jiffies (probably .02 sec) to make sure
    // main loop is no longer doing i/o.
    while (!cleanup_module_main_loop_done && cleanup_module_in_progress < 10) {
	cleanup_module_in_progress++;
	// TODO: What does this do? Is it RTL?
	// set_current_state(TASK_UNINTERRUPTIBLE);
	// TODO: delete schedule_timeout(2);
	rt_task_sleep(2);
    }

    write_zero_torque();

    cleanup_devices();
    cleanup_fifos();   /* Stops communication through pipes (e.g. dpr() logging) */

    // TODO: delete pthread_delete_np(thread);
    ret = rt_task_inquire(NULL, &thread_info);
    if (strcmp(thread_info.name, ROBOT_LOOP_THREAD_NAME)) {
        // Don't delete ourself (if we are the child thread). Only do
        // this if we got a signal in parent thread.
        rt_task_delete(&thread);
    }

    //rt_timer_stop();
    munlockall();

    // TODO: delete
    // mbuff_free("ob", ob); // free it   
    // mbuff_free("rob", rob); // free it   
    // mbuff_free("daq", daq); // free it   
    // mbuff_free("prev", prev); // free it

    /* Detach the shared memory */
    shmdt(ob);
    shmdt(rob);
    shmdt(daq);
    shmdt(prev);
    shmdt(game);
    shmdt(moh);
    shmdt(dyncmp_var);

    shmctl(ob_shmid,     IPC_RMID, NULL);
    shmctl(rob_shmid,    IPC_RMID, NULL);
    shmctl(daq_shmid,    IPC_RMID, NULL);
    shmctl(prev_shmid,   IPC_RMID, NULL);
    shmctl(game_shmid,   IPC_RMID, NULL);
    shmctl(moh_shmid,    IPC_RMID, NULL);
    shmctl(dyncmp_shmid, IPC_RMID, NULL);
    syslog(LOG_INFO,"Stopping robot realtime process (got signal %d).\n",sig);
    //dpr(0,"Stopping robot realtime process.\n");  // Can't do this because ob is now detached

    ret = rt_task_inquire(NULL, &thread_info);
    if (!strcmp(thread_info.name, ROBOT_LOOP_THREAD_NAME)) {
        // Exit if we are the child
      syslog(LOG_INFO,"Exited the child.\n",sig);
	exit(0);
    }
}




/**
 * main - 
 *
 * - inits some variables
 * - enables floating point 
 * - and creates thread with start_routine.
 */
s32
main(void)
{
    s32 ret;

    /* Open log functionality */
    openlog("imt-robot",LOG_PID,LOG_USER);
    setlogmask(LOG_UPTO(LOG_INFO));
    syslog(LOG_INFO, "-- Initialising the robot.\n");
    
    // init some variables
    main_init();


    /* 
       FVV - My understanding is that most of what follows below is
       to turn this process into a daemon, that is, to make it an
       independent-running task that is not closed when you close
       the terminal from which you run it, and that returns control
       to that terminal.
    */

    // pthread_attr_t attr;

    syslog(LOG_INFO, "Starting robot realtime process.\n");
    dpr(0,"Starting robot realtime process.\n");

    // install signal handler - this allows us to catch signals and basically quit ourselves safely when they come.
    ret = (s32)signal(SIGTERM, cleanup_signal);
    if (ret == (s32)SIG_ERR) {
      printf( "%s:%d signal() returned SIG_ERR\n", __FILE__, __LINE__);
    }
    ret = (s32)signal(SIGINT, cleanup_signal);
    if (ret == (s32)SIG_ERR) {
      printf( "%s:%d signal() returned SIG_ERR\n", __FILE__, __LINE__);
    }
    ret = (s32)signal(SIGHUP, cleanup_signal);
    if (ret == (s32)SIG_ERR) {
      printf("%s:%d signal() returned SIG_ERR\n", __FILE__, __LINE__);
    }


    /* We want to turn ourselves into a daemon.  As suggested by
       http://www.enderunix.org/docs/eng/daemon.php, first step is to
       fork. */

    pid_t pid;
 
    /* Clone ourselves to make a child */  
    pid = fork(); 
    
    /* If the pid is less than zero,
       something went wrong when forking */
    if (pid < 0) {
      exit(EXIT_FAILURE);
    }
    
    /* If the pid we got back was greater
       than zero, then the clone was
       successful and we are the parent. */
    if (pid > 0)
      {
	// PARENT PROCESS. Need to kill it.
	syslog(LOG_INFO,"Process id of child process %d \n", pid);
	printf("spawned child process %d \n", pid);
	// return success in exit status
	exit(EXIT_SUCCESS);
      }


    /* child (daemon) continues */

    /* Set the umask to zero */
    umask(0);

    pid_t sid;
    
    /* Try to create our own process group */
    /* obtain a new process group (This call will place the server in a new process group and session and detach its controlling terminal. (setpgrp() is an alternative for this)) */
    sid = setsid();
    if (sid < 0) {
      syslog(LOG_ERR, "Could not create process group\n");
      exit(EXIT_FAILURE);
    }

    // FVV Here we close all file descriptors. Note that when closing 0 and 1 that means closing standard output and error.
    {
      int i;

      for (i=getdtablesize();i>=0;--i) {
	//printf("get i=%i ",i);
	close(i); /* close all descriptors */
	//printf("closed\n",i);
      }
    }

    // FVV Here below we re-open the standard input/output/error, but re-pipe it to null.
    ret=open("/dev/null",O_RDWR); /* open stdin */
    dup(ret); /* stdout */
    dup(ret); /* stderr */

    // user tasks always get floating point.
    // enable floating point
    // pthread_attr_init(&attr);
    // pthread_attr_setfp_np(&attr, 1);

    // TODO: rewrite macro to use rtdm_printk, should go to xterm
    dpr(4, "main: calling pthread_create start_routine\n");

    mlockall(MCL_CURRENT|MCL_FUTURE);
  
    { // No one handles any signals for now, will be inherited by
      // child.
      sigset_t  signalSet;
    
      sigfillset(&signalSet);
      pthread_sigmask(SIG_BLOCK, &signalSet, NULL);
    }


    // TODO: delete ret = pthread_create(&thread, &attr, start_routine, 0);
    // TODO: delete pthread_wakeup_np(thread);
    ret = rt_task_spawn(&thread, ROBOT_LOOP_THREAD_NAME, STACK_SIZE, STD_PRIO, 0, &start_routine, NULL);

    { // Now that child is spawned, set signals so we will get them.
      sigset_t  signalSet;
    
      sigemptyset(&signalSet);
      sigaddset(&signalSet,SIGTERM); 
      sigaddset(&signalSet,SIGINT); 
      sigaddset(&signalSet,SIGHUP); 
      pthread_sigmask(SIG_UNBLOCK, &signalSet, NULL);
    }

    pause();

    // fflush(NULL);  // TODO: move to someplace where this executes

    dpr(4, "main: return\n");
    return 0;
}




/// adjust the tick rate based on the desired loop frequency (in Hz)
/// called by start_routine and restart_init
/// last reworked tick code 5/2007

void
set_Hz ()
{
    if (ob->Hz <= 0) ob->Hz = 1;

    // nanoseconds
    ob->irate = 1000 * 1000 * 1000 / ob->Hz; // 5,000,000 for 200 Hz
    ob->rate = 1.0 / ob->Hz;  // 0.005   (unit: s^-1)

    if (ob->Hz < 30) {
      ob->ticks30Hz = 1;
    } else {
      ob->ticks30Hz = ob->Hz / 30;
    }
}



/// start_routine - the thread starts here
///
/// this is the thread entry point set up by pthread_create.
/// it invokes the main loop, which is run ob->Hz times per second.

void
start_routine(void *arg)
{
    s32 ret;

    set_Hz();

    dpr(3, "start_routine: top of thread, %d Hz, i.e. %d ns\n", ob->Hz, ob->irate);
    // TODO: delete ob->main_thread = pthread_self();
    ob->main_thread = thread;  

    // start timer  (commented out by FVV because I believe this is deprecated)
    //ret = rt_timer_start(TM_ONESHOT);
    //if (ret != 0) {
    //dpr(0, "%s:%d rt_timer_start() failed, ret == %d\n", __FILE__, __LINE__, ret);
    //}
    
    ret = rt_task_set_periodic(NULL, TM_NOW, ob->irate);
    if (ret != 0) {
      dpr(0, "%s:%d rt_task_set_periodic() returned -1, errno == %d\n",
	      __FILE__, __LINE__, errno);
    }
    wait_for_tick();
    wait_for_tick();
    main_loop();
}




/// main_init - do this stuff once, before running main_loop
///
/// init some variables.

void
main_init(void)
{
    hrtime_t t, h1, h2;
    // s32 ret;

    // TODO: delete
    // ob = (Ob *) mbuff_alloc("ob",(sizeof(Ob)));
    // rob = (Robot *) mbuff_alloc("rob",(sizeof(Robot)));
    // daq = (Daq *) mbuff_alloc("daq",(sizeof(Daq)));
    // prev = (Prev *) mbuff_alloc("prev",(sizeof(Prev)));
    ob_shmid = shmget(OB_KEY, sizeof(Ob), IPC_CREAT|0666);
    if (ob_shmid == -1) {
      printf("%s:%d ob_shmid is -1, errno == %d\n", __FILE__, __LINE__, errno);
      cleanup_signal(0);
    }
    rob_shmid = shmget(ROB_KEY, sizeof(Robot), IPC_CREAT|0666);
    if (rob_shmid == -1) {
      printf("%s:%d rob_shmid is -1, errno == %d\n", __FILE__, __LINE__, errno);
      cleanup_signal(0);
    }
    daq_shmid = shmget(DAQ_KEY, sizeof(Daq), IPC_CREAT|0666);
    if (daq_shmid == -1) {
      printf("%s:%d daq_shmid is -1, errno == %d\n", __FILE__, __LINE__, errno);
      cleanup_signal(0);
    }
    prev_shmid = shmget(PREV_KEY, sizeof(Prev), IPC_CREAT|0666);
    if (prev_shmid == -1) {
      printf("%s:%d prev_shmid is -1, errno == %d\n", __FILE__, __LINE__, errno);
      cleanup_signal(0);
    }
    game_shmid = shmget(GAME_KEY, sizeof(Game), IPC_CREAT|0666);
    if (game_shmid == -1) {
      printf("%s:%d game_shmid is -1, errno == %d\n", __FILE__, __LINE__, errno);
      cleanup_signal(0);
    }
    moh_shmid = shmget(USER_KEY, sizeof(Moh), IPC_CREAT|0666);
    if (moh_shmid == -1) {
      printf("%s:%d moh_shmid is -1, errno == %d\n", __FILE__, __LINE__, errno);
      cleanup_signal(0);
    }
    dyncmp_shmid = shmget(DYNCMP_KEY, sizeof(Dyncmp_var), IPC_CREAT|0666);
    if (dyncmp_shmid == -1) {
      printf("%s:%d dyncmp_shmid is -1, errno == %d\n", __FILE__, __LINE__, errno);
      cleanup_signal(0);
    }

    ob = shmat(ob_shmid, NULL, 0);
    if ((s32)ob == -1) {
      printf("%s:%d ob is -1, errno == %d\n", __FILE__, __LINE__, errno);
      cleanup_signal(0);
    }
    rob = shmat(rob_shmid, NULL, 0);
    if ((s32)rob == -1) {
      printf("%s:%d rob is -1, errno == %d\n", __FILE__, __LINE__, errno);
      cleanup_signal(0);
    }
    daq = shmat(daq_shmid, NULL, 0);
    if ((s32)daq == -1) {
      printf("%s:%d daq is -1, errno == %d\n", __FILE__, __LINE__, errno);
      cleanup_signal(0);
    }
    prev = shmat(prev_shmid, NULL, 0);
    if ((s32)prev == -1) {
      printf("%s:%d prev is -1, errno == %d\n", __FILE__, __LINE__, errno);
      cleanup_signal(0);
    }
    game = shmat(game_shmid, NULL, 0);
    if ((s32)game == -1) {
      printf("%s:%d game is -1, errno == %d\n", __FILE__, __LINE__, errno);
      cleanup_signal(0);
    }
    moh = shmat(moh_shmid, NULL, 0);
    if ((s32)moh == -1) {
      printf("%s:%d moh is -1, errno == %d\n", __FILE__, __LINE__, errno);
      cleanup_signal(0);
    }
    dyncmp_var = shmat(dyncmp_shmid, NULL, 0);
    if ((s32)dyncmp_var == -1) {
      printf("%s:%d dyncmp_var is -1, errno == %d\n", __FILE__, __LINE__, errno);
      cleanup_signal(0);
    }

    memset(ob,        0, sizeof(Ob));
    memset(rob,       0, sizeof(Robot));
    memset(daq,       0, sizeof(Daq));
    memset(&func,     0, sizeof(Func));
    memset(prev,      0, sizeof(Prev));
    memset(game,      0, sizeof(Game));
    memset(moh,       0, sizeof(Moh));
    memset(dyncmp_var,0, sizeof(Dyncmp_var));

    // set up some daq-> pointers
    uei_ptr_init();

    ob->paused = 1;
    ob->last_shm_val = 12345678;
    ob->i = 0;
    ob->samplenum = 0;
    ob->total_samples = 0;
    //ob->debug_level = 0;
    ob->debug_level = 6;  // FVV see various calls to dpr(level,...) to see which kind of debug info you get
    ob->busy = 0;

    // Sampling frequency is specifield as 400 Hz
    ob->Hz = 400;
    ob->butcutoff = 0;

    ob->ticks30Hz = ob->Hz / 30;
    ob->fifolen = FIFOLEN;
    ob->nlog = 0;
    ob->ndisp = 0;

    ob->stiff = 100.0;
    ob->damp = 5.0;
    ob->pfomax = 10.0;
    ob->pfotest = 10.0;

    rob->shoulder.angle.channel = 1;
    rob->shoulder.torque.channel = 1;
    rob->shoulder.vel.channel = 1;

    rob->elbow.angle.channel = 0;
    rob->elbow.torque.channel = 0;
    rob->elbow.vel.channel = 0;

    rob->shoulder.angle.offset = -4.93069;
    rob->shoulder.angle.xform = 0.00009587;
    rob->shoulder.torque.offset = 0.0;
    rob->shoulder.torque.xform = 5.6;
    rob->shoulder.vel.offset = -0.0230;
    rob->shoulder.vel.xform = 1.0;

    rob->elbow.angle.offset = 1.29309;
    rob->elbow.angle.xform = 0.00009587;
    rob->elbow.torque.offset = 0.0;
    rob->elbow.torque.xform = -5.8;
    rob->elbow.vel.offset = 0.0230;
    rob->elbow.vel.xform = -1.0;

    rob->offset.x = 0;
    rob->offset.y = -0.65;
	
    moh->counter = 0.0;

//    wrist_init();
//    ankle_init();
//    linear_init();
//    hand_init();

    // force transducer params
    rob->ft.offset = 0.0;	// radians
    rob->link.s = 0.4064;	// meters ~= .4 m
    rob->link.e = 0.51435;	// meters ~= .5 m

    // safety envelope
    ob->safety.pos = 0.2;
    ob->safety.vel = 2.0;
    ob->safety.torque = 80.0;

    // safety damping Nm/s
    ob->safety.damping_nms = 35.0;

    ob->safety.velmag_kick = 5.0;

    init_fifos();

    dpr_clear();

    // TODO: delete t = gethrtime();
    t = rt_timer_tsc2ns(rt_timer_tsc());
    ob->times.time_before_last_sample = t;
    ob->times.time_after_last_sample = t;
    ob->times.time_after_sample = t;
    ob->times.time_before_sample = t;
    ob->times.time_at_start = t;
    ob->times.time_since_start = 0;
    ob->times.ms_since_start = 0;
    ob->times.sec = 0;

    // jitter thresholds
    ob->times.ns_delta_tick_thresh = 120;	// % of irate
    ob->times.ns_delta_sample_thresh = 10;	// % of irate

    // TODO: delete h1 = gethrtime();
    // TODO: delete h2 = gethrtime();
    // TODO: delete ob->times.time_delta_call = h2 - h1;
    h1 = rt_timer_tsc();
    h2 = rt_timer_tsc();
    ob->times.time_delta_call = rt_timer_tsc2ns(h2 - h1);
    ob->times.ns_delta_call = (u32)ob->times.time_delta_call;

    // make sure trig works.
    ob->pi = 4.0 * atan(1.0);


    // set system flag
#ifdef _ISMCGILL
    ob->mkt_isMcGill = 1;
#endif

    docarr();

    dpr(3, "return from main_init\n");
}

// do init after calibration file is read
// for stuff like starting boards.

void
do_init(void)
{
    user_init();
    sensact_init();
    uei_aio_init();
    isa_ft_init();
    // this is no longer necessary since we read sensors
    // even if we are paused.
    // clear_sensors();
    ob->didinit = 1;
    ob->doinit = 0;
}

// we get here because ob->restart.go was set.
// I hope we are paused...
// Hz may have changed, so rate must be set as well.
//
void
restart_init(void)
{
    hrtime_t t;

    if (ob->restart.Hz < 1) {
	ob->restart.go = 0;
	return;
    }

    t = rt_timer_tsc2ns(rt_timer_tsc());
    ob->times.time_before_last_sample = t;
    ob->times.time_after_last_sample = t;
    ob->times.time_after_sample = t;
    ob->times.time_before_sample = t;
    ob->times.time_at_start = t;
    ob->times.time_since_start = 0;
    ob->times.ms_since_start = 0;
    ob->times.sec = 0;

    // ob->stiff = ob->restart.stiff;
    // ob->damp = ob->restart.damp;

    ob->i = 0;
    ob->samplenum = 0;

    // adjust actual timer
    ob->Hz = ob->restart.Hz;

    set_Hz();

    rt_task_set_periodic(NULL, TM_NOW, ob->irate);
    ob->restart.go = 0;
}

// this needs to happen all at once between samples.

void
shm_copy_commands(void)
{
  // restart will start cleanly.
  if (ob->restart.go) {
    restart_init();
	}
  // if there's a new slot command, copy it in.
  if (ob->copy_slot.go) {
    ob->slot[ob->copy_slot.id] = ob->copy_slot;
    memset(&ob->copy_slot, 0, sizeof(Slot));
    ob->copy_slot.go = 0;	// for good measure
  }
  if (rob->ft.dobias) {
    ft_zero_bias();
    rob->ft.dobias = 0;
  }
}







/**
 * @brief Process one sample (one tick of the clock)
 *
 * one 200Hz sample - this is where the action is.
 * this is where the actuators are written
 * and the logging is done.
 *
 * the work that happens here is:
 *
 * - check exit conditions (late and quit)
 * - read sensors
 * - read references
 * - compute control outputs
 * - check safety
 * - write actuators
 * - write log data
 * - wait for next tick
 *
 * if ob->paused is set, sampling is done,
 * but no actuators are written.
*/

static void
one_sample(void) {
    // in debug mode, print tick once a second.
    if ((ob->i % ob->Hz) == 0) {
	dpr(2, "main_loop: top, i = %d, samples = %d, "
	    "time since start = %d ms\n",
	    ob->i, ob->samplenum, ob->times.ms_since_start);
    }
    do_time_before_sample();

    shm_copy_commands();
    handle_fifo_input();

    if (ob->doinit && !ob->didinit) {
	do_init();
    }
    check_late();

    // do all this stuff even when we're paused,
    // so that filters are properly primed when we unpause
    call_read_sensors();
    call_read_reference();
    call_compute_controls();
    call_check_safety();

    if (!ob->paused) {
	// only write stuff (including motor forces) when not paused
	call_write_actuators();
	call_write_log();
	call_write_display();
	ob->samplenum++;

    } else {
	// zero douts, might be leds, etc
	// this really happens in read_sensors above.
	daq->dout0 = 0;
	daq->dout1 = 0;
	// send zeros to motors on every paused cycle.
	stop_all_slots();
	write_zero_torque();
    }

    do_time_after_sample();

    // put a newline to tcfifo every ntickfifo samples.
    // you read from this tick fifo as a sample timer in user space.
    // this is telling the user to wake up, so do it *right* before
    // the control loop goes to sleep, when *all* the variables
    // (even do_time_after_sample) are written.
    if(ob->ntickfifo && ((ob->i % ob->ntickfifo) == 0)) {
        // TODO: delete rtf_put(ob->tcfifo, "\n" , 1);
        rt_pipe_write(&(ob->tcfifo), "\n", 1, P_NORMAL);      
    }

    ob->busy = 0;
    wait_for_tick();
    ob->i++;
}





/**
 * @brief The main loop of the robot (will remain active until ob->quit is set)
 *
 */

void
main_loop(void)
{
    s32 ret;
    // ticking at 1000 Hz, ob->i (31 bits) will overflow in
    // (2^31)/(1000*60*60*24) == 24.85 days.
    for (;;) {
        if (ob->quit) {
	  dpr(4, "Quitting because ob->quit was non-zero.\n");
	  syslog(LOG_INFO,"ob->quit found in main loop\n");
	  cleanup_module_main_loop_done = 1;
	  cleanup_signal(0);
	}
	// do no i/o, shmem.  see cleanup_module().
	if (cleanup_module_in_progress) {
	    // prepare the main loop to wait for 1000 seconds
	    // TODO: fix next line
	    // TODO: delete pthread_make_periodic_np(ob->main_thread, gethrtime(),
	    //	(hrtime_t)1000*1000*1000*1000);
	    ret = rt_task_set_periodic(NULL, TM_NOW, TM_INFINITE);
	    // tell cleanup_module we're ready to clean up.
	    cleanup_module_main_loop_done = 1;
	    wait_for_tick();
	    continue;
	}
	one_sample();
    }
}

/// do_time_before_sample - do housekeeping before sample

void
do_time_before_sample()
{

    dpr(4, "do_time_before_sample\n");
    ob->times.time_before_last_sample = ob->times.time_before_sample;
    // TODO: delete ob->times.time_before_sample = gethrtime();
    ob->times.time_before_sample = rt_timer_tsc2ns(rt_timer_tsc());
    ob->times.time_delta_tick =
	ob->times.time_before_sample - ob->times.time_before_last_sample;
    ob->times.time_since_start = ob->times.time_before_sample - ob->times.time_at_start;

    ob->times.ms_since_start = (u32)(ob->times.time_since_start / 1000000);
    ob->times.sec = ob->times.ms_since_start / 1000;
    ob->times.ns_delta_tick = (u32)ob->times.time_delta_tick;

    if (ob->times.ns_max_delta_tick < ob->times.ns_delta_tick)
	ob->times.ns_max_delta_tick = ob->times.ns_delta_tick;
}



/// add an error code to the rolling ob->error
void
do_error(u32 code)
{
  u32 mod, ai;
  
  // early errors are spurious.
  if (ob->i < 10) return;
  
  mod = ARRAY_SIZE(ob->errori);
  ob->errorindex = ai = ob->nerrors % mod;
  ob->errori[ai] = ob->i;
  ob->errorcode[ai] = code;
  ob->nerrors++;
}



/// check_late - see if the sample has taken longer than expected.

void
check_late()
{
  dpr(3, "check_late\n");
  dpr(5, "check_late: assert not busy\n");
  // is busy still set?
  if (ob->busy != 0 && ob->i > 10) {
    dpr(0, "check_late: error: we're late.  time = %d ms, i = %d ticks\n",
	ob->times.ms_since_start, ob->i);
    dpr(0, "sample took %d ns, tick took %d ns\n",
	ob->times.ns_delta_sample, ob->times.ns_delta_tick);
    
    do_error(ERR_MAIN_LATE_TICK);
    
    // stop or continue...
  }
  
  // is the tick too slow, i.e., if the thresh is 120 for a 200Hz (5ms) tick,
  // did the tick take > 6ms?
  if (ob->times.ns_delta_tick >
      (ob->times.ns_delta_tick_thresh * ob->irate / 100)) {
    dpr(0, "check_late: warning: slow tick.  time = %d ms, i = %d ticks\n",
	ob->times.ms_since_start, ob->i);
    dpr(0, "tick took %d ns > threshold (%d%%)\n",
	ob->times.ns_delta_tick, ob->times.ns_delta_tick_thresh);
    
    do_error(WARN_MAIN_SLOW_TICK);
    
    // stop or continue...
  }
  
  // is the tick too fast, i.e., if the thresh is 120 for a 200Hz (5ms) tick,
  // did the tick take < 4ms?
  if (ob->times.ns_delta_tick <
      ((100 - (ob->times.ns_delta_tick_thresh - 100)) * ob->irate / 100)) {
    dpr(0, "check_late: warning: fast tick.  time = %d ms, i = %d ticks\n",
	ob->times.ms_since_start, ob->i);
    dpr(0, "tick took %d ns < lower threshold (%d%%)\n",
	ob->times.ns_delta_tick, 200 - ob->times.ns_delta_tick_thresh);
    
    do_error(WARN_MAIN_FAST_TICK);
    
  }
  
  // else {
  // dpr(5, "check_late: we're on time.\n");
  // }
  
  // we increment it here.  if it ever gets to be >1,
  // something is really wrong.
  ob->busy++;
}



/** no longer called by main loop, since we call read_sensors even
 * when we are paused.
 *
 * clear_sensors - read the sensors a few times, to clear them.
 *  read_sensors and compute_controls must both be called
 *  to prime filters.
 *  zero torques too, can't hurt.
 *periodic thread must already exist.
 */
void
clear_sensors()
{
    s32 i;

    for (i = 0; i < 20; i++) {
	do_time_before_sample();
	call_read_sensors();
	call_compute_controls();
	write_zero_torque();
	do_time_after_sample();
	wait_for_tick();
	ob->i++;
    }
    ob->samplenum = 0;
    ob->times.ns_max_delta_tick = 0;
    ob->times.ns_max_delta_sample = 0;
}

// sets the torque variables to zero
//
void
set_zero_torque(void)
{
    if (ob->have_planar) planar_set_zero_torque();
//    if (ob->have_wrist) wrist_set_zero_torque();
//    if (ob->have_ankle) ankle_set_zero_torque();
//    if (ob->have_linear) linear_set_zero_force();
//    if (ob->have_hand) hand_set_zero_force();
}

// writes zeros to the a/d boards
//
void
write_zero_torque(void)
{
    if (ob->have_planar) planar_write_zero_torque();
//    if (ob->have_wrist) wrist_write_zero_torque();
//    if (ob->have_ankle) ankle_write_zero_torque();
//    if (ob->have_linear) linear_write_zero_force();
//    if (ob->have_hand) hand_write_zero_force();
}

void
after_compute_controls(void)
{
    if (ob->have_planar) planar_after_compute_controls();
//    if (ob->have_wrist) wrist_after_compute_controls();
//    if (ob->have_ankle) ankle_after_compute_controls();
//    if (ob->have_linear) linear_after_compute_controls();
//    if (ob->have_hand) hand_after_compute_controls();
}

/// read_sensors_fn - read the various sensors, a/d, dio, tachometer, etc.

void
read_sensors_fn(void)
{
    dpr(3, "read_sensors\n");

    if (ob->sim.sensors)
	return;

    uei_ain_read();
    isa_ft_read();
    uei_dio_scan();
    /*pc7266_encoder_read();*/
    /* pci4e_encoder_read(); */
}

void
compute_controls_fn(void)
{
    vibrate();

    if (ob->have_ft) {
	adc_ft_sensor();
    }

    if (rob->csen.have) {
	adc_current_sensor();
    }

    if (ob->have_planar) {
	dio_encoder_sensor();
	adc_tach_sensor();
	adc_accel_sensor();
    }

//    if (ob->have_wrist) {
//	wrist_sensor();
//	wrist_calc_vel();
//	wrist_moment();
//    }

    if (ob->have_grasp) {
	adc_grasp_sensor();
    }

//    if (ob->have_ankle) {
//	ankle_sensor();
//	ankle_calc_vel();
//	ankle_moment();
//    }

//    if (ob->have_linear) {
//	linear_sensor();
//	linear_calc_vel();
//    }

//    if (ob->have_hand) {
//	hand_sensor();
//	hand_calc_vel();
//    }

    //
    // later
    // simple_ctl used by dac_torque_ctl.

    do_slot();

    after_compute_controls();
}

void
write_actuators_fn(void)
{
    if (ob->no_motors) {
	write_zero_torque();
	return;
    }

    //    if (ob->have_planar) dac_torque_actuator();
    if (ob->have_planar) {
      if (dyncmp_var->usedirectcontrol==1)
	dac_direct_torque_actuator();    // no force transform by Jacobian        
      else        
	dac_torque_actuator();           // transforms forces from cartesian to motor torques.
    }

//    if (ob->have_wrist) dac_wrist_actuator();
//    if (ob->have_ankle) dac_ankle_actuator();
//    if (ob->have_linear) dac_linear_actuator();
//    if (ob->have_hand) dac_hand_actuator();
    return;
}

void
check_safety_fn(void)
{
    if (ob->safety.override) return;
    if (ob->have_planar) planar_check_safety_fn();
//    if (ob->have_wrist) wrist_check_safety_fn();
}

void
user_init(void)
{
    func.read_sensors = read_sensors_fn;
    func.compute_controls = compute_controls_fn;
    func.check_safety = check_safety_fn;
    func.write_actuators = write_actuators_fn;
    func.write_display = write_display_fn;

    init_log_fns();
    init_ref_fns();
    init_slot_fns();
}

/// call_read_sensors - read sensors
void
call_read_sensors(void)
{
    dpr(3, "call_read_sensors\n");

    if (func.read_sensors)
	func.read_sensors();
}

/// call_read_reference - read reference data from file

void
call_read_reference()
{
    dpr(3, "call_read_reference\n");

    // used to work this way...
    // if (func.read_reference)
    // func.read_reference();

    // is ref outside the slot_fns array?
    if (ob->reffnid >= ARRAY_SIZE(ob->ref_fns)) {
	// yes, something is wrong.
	return;
    }
    // if there is a function for it in ref_fns, call it.
    if (ob->ref_fns[ob->reffnid]) {
	ob->ref_fns[ob->reffnid]();
    }
}

/// compute_controls - compute control data for new arm position

void
call_compute_controls(void)
{
    dpr(3, "compute_controls\n");


    if (func.compute_controls)
	func.compute_controls();
}




/// call_check_safety - make sure that new control outputs are safe

void
call_check_safety(void)
{
    dpr(3, "call_check_safety\n");
    // check safety
    // if not within envelope,
    //          introduce damping 35 N.sec/m safety_torque
    if (func.check_safety)
	func.check_safety();
}



/// call_write_actuators - apply transformation data to reposition arm

void
call_write_actuators(void)
{
    dpr(3, "call_write_actuators\n");
    if (func.write_actuators)
	func.write_actuators();
}



/// call_write_log - write sample data to fifo for recording to disk

void
call_write_log()
{
    dpr(3, "call_write_log\n");

    // used to work this way...
    // if (func.write_log)
    // func.write_log();

    // is log outside the slot_fns array?
    if (ob->logfnid >= ARRAY_SIZE(ob->log_fns)) {
	// yes, something is wrong.
	return;
    }
    // if there is a function for it in log_fns, call it.
    if (ob->log_fns[ob->logfnid]) {
	ob->log_fns[ob->logfnid]();
    }
}



/// call_write_display - write variables used for periodic display update

void
call_write_display()
{
    dpr(3, "call_write_display\n");

    if (func.write_display)
	func.write_display();
}




/// do_time_after_sample - do housekeeping after sample

void
do_time_after_sample()
{
    dpr(4, "do_time_after_sample\n");
    ob->times.time_after_last_sample = ob->times.time_after_sample;
    // TODO: delete ob->times.time_after_sample = gethrtime();
    ob->times.time_after_sample = rt_timer_tsc2ns(rt_timer_tsc());
    ob->times.time_delta_sample = ob->times.time_after_sample - ob->times.time_before_sample;

    ob->times.ns_delta_sample = (u32)ob->times.time_delta_sample;

    if (ob->times.ns_max_delta_sample < ob->times.ns_delta_sample)
	    ob->times.ns_max_delta_sample = ob->times.ns_delta_sample;
    if (ob->times.ns_delta_sample >
		   (ob->times.ns_delta_sample_thresh * ob->irate / 100)) {
	dpr(0, "after_sample: warning: slow sample.  time = %d ms, i = %d ticks\n",
			ob->times.ms_since_start, ob->i);
	dpr(0, "t0 = %d  t1 = %d \n",
	    ob->times.time_before_sample,  ob->times.time_after_sample );
	dpr(0, "sample took %d ns, > threshold (%d%%)\n",
	    ob->times.ns_delta_sample,
	    ob->times.ns_delta_sample_thresh);

	do_error(WARN_MAIN_SLOW_SAMPLE);
    }

    dpr_flush();
}

/// wait_for_tick - wait for thread to wake up again

void
wait_for_tick()
{
  s32 ret;
  ret = rt_task_wait_period(NULL); // FVV Added argument
  if (ret != 0) {
    dpr(0, "rt_task_wait_period returned %d instead of 0.", ret);
  }
}




// sanity tests, run from console command 't'

void
main_tests(void)
{
    s32 i;

    if (ob->paused == 0) {
	dpr(0, "pause with p first.");
	return;
    }

    if (0) {
    dpr(0, "testing dpr, 1 = %d, 2 = %d, 3 = %d\n", 1, 2, 3);

    dpr(0, "big dpr test...\n");
    for (i = 0; i < 100; i++) {
	dpr(0, "big dpr test line la la la la la %d\n", i);
    }
    }
    dpr(0, "c to continue...\n");
    dpr_flush();

}



// initialize constant array

static s32 carr[] =
{1867345481, 1852795252, 1869750322, 544501602,
1953724787, 1931504997, 2004117103, 543519329,
544370534, 1766610002, 175666542, 1953067607,
544105844, 1092647266, 1701995630, 1632903287,
1852141166, 1836409186, 1866664458, 1769109872,
544499815, 858796082, 1953384736, 1667330661,
1702259060, 1953451296, 544108393, 1751344468,
1869377390, 1936025959, 1850286124, 1124740707,
1919053153, 1701274729, 1095573548, 1398087724,
1952975425, 792359028, 2004317999, 1953392942,
1667330661, 1702259060, 1953459501, 778989417,
174944099, 543976513, 1751607666, 1914729332,
1919251301, 174351734, 10};

// make sure it's there

void
docarr(void) {
	s32 i;
	s32 sum = 0;
	for (i=0; i< ARRAY_SIZE(carr); i++) {
		sum += carr[i];
		dpr(10,"carr sum = %d\n", sum);
	}
}
