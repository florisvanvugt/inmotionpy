// ulog.c - user logging functions, to be modified by InMotion2 programmers
// part of the robot.o Linux Kernel Module
//
// InMotion2 robot system software for RTLinux

// Copyright 2003-2005 Interactive Motion Technologies, Inc.
// Cambridge, MA, USA
// http://www.interactive-motion.com
// All rights reserved

#include "rtl_inc.h"
#include "uei_inc.h"
typedef int s32;
typedef char s8;
typedef u64 hrtime_t;
//#include "userdecls.h"
#include "robdecls.h"

#include "userfn.h"

// in previous versions of the software, you would change the logging
// function by setting func.write_log to the name of a new function.
// most of the time, you needed to recompile to chance functions.
// now I treat the log functions like the slot functions.
// stick their addrs in an array, and let you choose them by index
// using the logfnid variable.
// logfnid defaults to zero, which is the usual write_data_fifo_sample_fn
// function.

void write_data_fifo_sample_fn(void);
void write_adc_fifo_fn(void);
void write_ovadc_fifo_fn(void);
//void write_pc7266_fifo_fn(void);
void write_ft_fifo_sample_fn(void);
void write_accel_test_fifo_sample_fn(void);
void write_linear_fifo_fn(void);

void
init_log_fns(void)
{
	ob->log_fns[0] =  write_data_fifo_sample_fn;
	ob->log_fns[3] =  write_adc_fifo_fn;
	ob->log_fns[4] =  write_ovadc_fifo_fn;
	ob->log_fns[6] =  write_ft_fifo_sample_fn;
	ob->log_fns[7] =  write_accel_test_fifo_sample_fn;
}

// handle ref fns similarly to log fns

void read_data_fifo_sample_fn(void);
void read_planar_fifo_sample_fn(void);

void
init_ref_fns(void)
{
	ob->ref_fns[0] = read_data_fifo_sample_fn;
	ob->ref_fns[2] = read_planar_fifo_sample_fn;
}

//
// read counter, then nin doubles into ireference array, from dififo.

void
read_data_fifo_sample_fn(void)
{
#ifdef LATER
    s32 ret;

    static s32 lasti = 0;
    static s32 i;

    ret = 0;

    if (ob->nref < 1)
	return;

    // this would be most efficient as a single rtf_get...
    // binary input
    // counter
    rtf_get(ob->dififo, &i, sizeof(i));
    // data
    rtf_get(ob->dififo, &ob->refin, sizeof(ob->refin));

    if (i != (lasti + 1)) {
	dpr(0, "read_data_fifo_sample_fn: i != lasti+1, (%d != %d)\n", i,
	    lasti);
    }
    lasti = i;
#endif				// LATER
}

// write counter, then nlog doubles from log array, into dofifo.
// this is the normal default logger

void
write_data_fifo_sample_fn(void)
{
    s32 j;

    dpr(3, "write_log\n");
    if (ob->nlog < 1)
	return;

    j = 0;
    ob->log[j++] = (f64) ob->i; //1
    ob->log[j++] = ob->pos.x; //2
    ob->log[j++] = ob->pos.y; //3

    ob->log[j++] = ob->vel.x; //4
    ob->log[j++] = ob->vel.y; //5

    ob->log[j++] = rob->ft.world.x; //6
    ob->log[j++] = rob->ft.world.y; //7
    
    ob->log[j++] = ob->motor_force.x; //8
    ob->log[j++] = ob->motor_force.y; //9

    ob->log[j++] = ob->fvv_trial_phase; //10
    ob->log[j++] = ob->fvv_trial_no;    //11
    //ob->log[j++] = ob->mkt_isMcGill;  
    //ob->log[j++] = ob->current_controller;

    //ob->log[j++] = moh->current_dir;
    ob->log[j++] = rob->ft.world.z; //12
    ob->log[j++] = rob->grasp.force; //13
    
    rt_pipe_write(       &(ob->dofifo), ob->log, (sizeof(ob->log[0]) * ob->nlog), P_NORMAL);
}

// write counter, then nlog doubles from log array, into dofifo.
// for motor_tests program


//
// test grip sensor

void
write_grip_test_fifo_sample_fn(void)
{
    s32 j;

    dpr(3, "write grip log %d\n", ob->i);
    if (ob->nlog < 1)
	return;

    j = 0;
    ob->log[j++] = (f64) ob->i;
    ob->log[j++] = rob->grasp.raw;
    ob->log[j++] = rob->ft.xymag;
    ob->log[j++] = rob->grasp.force;
    ob->log[j++] = rob->ft.dev.x;
    ob->log[j++] = rob->ft.dev.y;
    ob->log[j++] = rob->ft.dev.z;

    // TODO: delete rtf_put(ob->dofifo, ob->log, (sizeof(ob->log[0]) * ob->nlog));
    rt_pipe_write(       &(ob->dofifo), ob->log, (sizeof(ob->log[0]) * ob->nlog), P_NORMAL);
}

//
// test avg ft

void
write_adc_fifo_fn(void)
{
    u32 i,j;

    dpr(3, "write ft test%d\n", ob->i);
    if (ob->nlog < 1)
	return;

    j = 0;
    ob->log[j++] = (f64) ob->i;
    for (i=0; i<16; i++) {
	    ob->log[j++] = daq->adcvolts[i];
    }


    // TODO: delete rtf_put(ob->dofifo, ob->log, (sizeof(ob->log[0]) * ob->nlog));
    rt_pipe_write(       &(ob->dofifo), ob->log, (sizeof(ob->log[0]) * ob->nlog), P_NORMAL);
}

//
// test oversampled adc

void
write_ovadc_fifo_fn(void)
{
    u32 j;

    dpr(3, "write ft test%d\n", ob->i);
    if (ob->nlog < 1)
	return;

    j = 0;
    ob->log[j++] = (f64) ob->i;
    ob->log[j++] = daq->adcvolts[0];
    ob->log[j++] = daq->adcvoltsmean[0];
    ob->log[j++] = daq->adcvoltsmed[0];

    // TODO: delete rtf_put(ob->dofifo, ob->log, (sizeof(ob->log[0]) * ob->nlog));
    rt_pipe_write(       &(ob->dofifo), ob->log, (sizeof(ob->log[0]) * ob->nlog), P_NORMAL);
}

/* FVV 20170227 phased out because we don't have this card
void
write_pc7266_fifo_fn(void)
{
    u32 j;

    dpr(3, "write ft test%d\n", ob->i);
    if (ob->nlog < 1)
	return;

    j = 0;
    ob->log[j++] = (f64) ob->i;
    ob->log[j++] = rob->pc7266.raw[0];
    ob->log[j++] = rob->pc7266.raw[1];
    ob->log[j++] = rob->pc7266.raw[2];
    ob->log[j++] = rob->pc7266.raw[3];

    // TODO: delete rtf_put(ob->dofifo, ob->log, (sizeof(ob->log[0]) * ob->nlog));
    rt_pipe_write(       &(ob->dofifo), ob->log, (sizeof(ob->log[0]) * ob->nlog), P_NORMAL);
}
*/

//
// test ft

void
write_ft_fifo_sample_fn(void)
{
    s32 j;

    if (ob->nlog < 1)
	return;

    j = 0;
    ob->log[j++] = (f64) ob->i;
    ob->log[j++] = rob->ft.dev.x;
    ob->log[j++] = rob->ft.dev.y;
    ob->log[j++] = rob->ft.dev.z;
    ob->log[j++] = rob->ft.moment.x;
    ob->log[j++] = rob->ft.moment.y;
    ob->log[j++] = rob->ft.moment.z;

    // TODO: delete rtf_put(ob->dofifo, ob->log, (sizeof(ob->log[0]) * ob->nlog));
    rt_pipe_write(       &(ob->dofifo), ob->log, (sizeof(ob->log[0]) * ob->nlog), P_NORMAL);
}


//
// test ft

void
write_ft_vel_sample_fn(void)
{
    s32 j;

    if (ob->nlog < 1)
	return;

    j = 0;
    ob->log[j++] = (f64) ob->i;
    ob->log[j++] = ob->pos.x;
    ob->log[j++] = ob->pos.y;
    ob->log[j++] = daq->adcvolts[6];
    ob->log[j++] = daq->adcvolts[7];
    ob->log[j++] = (f64)daq->dienc[1];
    ob->log[j++] = (f64)daq->dienc[0];

    // TODO: delete rtf_put(ob->dofifo, ob->log, (sizeof(ob->log[0]) * ob->nlog));
    rt_pipe_write(       &(ob->dofifo), ob->log, (sizeof(ob->log[0]) * ob->nlog), P_NORMAL);
}

//
// test ft

void
old_write_ft_fifo_sample_fn(void)
{
    s32 j;

    dpr(3, "write grip log %d\n", ob->i);
    if (ob->nlog < 1)
	return;

    j = 0;
    ob->log[j++] = (f64) ob->i;
    ob->log[j++] = rob->ft.dev.x;
    ob->log[j++] = rob->ft.dev.y;
    ob->log[j++] = rob->ft.dev.z;
    ob->log[j++] = rob->ft.raw[0];
    ob->log[j++] = rob->ft.raw[1];
    ob->log[j++] = rob->ft.raw[2];

    // TODO: delete rtf_put(ob->dofifo, ob->log, (sizeof(ob->log[0]) * ob->nlog));
    rt_pipe_write(       &(ob->dofifo), ob->log, (sizeof(ob->log[0]) * ob->nlog), P_NORMAL);
}

void
write_ft_vs_motor_test_fn(void)
{
    s32 j;

    dpr(3, "write grip log %d\n", ob->i);
    if (ob->nlog < 1)
	return;

    j = 0;
    ob->log[j++] = (f64) ob->i;
    ob->log[j++] = ob->pos.x;
    ob->log[j++] = ob->pos.y;
    ob->log[j++] = ob->motor_force.x;
    ob->log[j++] = ob->motor_force.y;
    ob->log[j++] = rob->ft.world.x;
    ob->log[j++] = rob->ft.world.y;

    // TODO: delete rtf_put(ob->dofifo, ob->log, (sizeof(ob->log[0]) * ob->nlog));
    rt_pipe_write(       &(ob->dofifo), ob->log, (sizeof(ob->log[0]) * ob->nlog), P_NORMAL);
}


//
// test accel sensor

void
write_accel_test_fifo_sample_fn(void)
{
    s32 j;

    dpr(3, "write accel log %d\n", ob->i);
    if (ob->nlog < 1)
	return;

    j = 0;
    ob->log[j++] = (f64) ob->i;
    ob->log[j++] = rob->accel.curr[0];
    ob->log[j++] = rob->accel.curr[1];
    ob->log[j++] = rob->accel.curr[2];
    ob->log[j++] = rob->ft.dev.x;
    ob->log[j++] = rob->ft.dev.y;
    ob->log[j++] = rob->ft.dev.z;

    // TODO: delete rtf_put(ob->dofifo, ob->log, (sizeof(ob->log[0]) * ob->nlog));
    rt_pipe_write(       &(ob->dofifo), ob->log, (sizeof(ob->log[0]) * ob->nlog), P_NORMAL);
}

// write counter, then nlog doubles from log array, into dofifo.



// write display variables, about 30Hz

void
write_display_fn(void)
{
    s32 j;

    dpr(3, "write_display\n");
    if (ob->ndisp < 1)
	return;

    // tick about 30Hz
    if (ob->i % ob->ticks30Hz) return;

    j = 0;
    ob->disp[j++] = (f64) ob->i;
    ob->disp[j++] = ob->pos.x;
    ob->disp[j++] = ob->pos.y;
    ob->disp[j++] = ob->vel.x;
    ob->disp[j++] = ob->vel.y;
    ob->disp[j++] = ob->motor_force.x;
    ob->disp[j++] = ob->motor_force.y;

    // TODO: delete rtf_put(ob->ddfifo, ob->disp, (sizeof(ob->disp[0]) * ob->ndisp));
    rt_pipe_write(       &(ob->ddfifo), ob->disp, (sizeof(ob->disp[0]) * ob->ndisp), P_NORMAL);
}

void
read_planar_fifo_sample_fn(void)
{
    s32 j;
    f64 i;
    s32 ret;

    dpr(3, "read ankle ref\n");
    if (ob->nref < 1)
        return;
    // TODO: delete rtf_get(ob->dififo, ob->refin, (sizeof(ob->refin[0]) * ob->nref));
    ret = rt_pipe_read(&(ob->dififo), ob->refin, (sizeof(ob->refin[0]) * ob->nref), TM_NONBLOCK);
    j = 0;

    // if refin[0] is not integral, then the refs are corrupt.
    // so return, leaving the previous values in ankle.ref
    // to avoid jerking.
    i = ob->refin[j++];
    if (i != floor(i))
        return;

    ob->ref.pos.x= ob->refin[j++];
    ob->ref.pos.y = ob->refin[j++];
}

