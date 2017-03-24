// write.c - fifo printf and debug printf
// part of the robot.o Linux Kernel Module

// InMotion2 robot system software for RTLinux

// Copyright 2003-2005 Interactive Motion Technologies, Inc.
// Cambridge, MA, USA
// http://www.interactive-motion.com
// All rights reserved

// printf to rtl fifo

#include <stdarg.h>
#include "rtl_inc.h"
#include "uei_inc.h"
typedef int s32;
typedef char s8;
typedef u64 hrtime_t;
#include "robdecls.h"

// general purpose fifo printf

void
fprf(RT_PIPE *fifo, const s8 *fmt, ...)
{
    va_list ap;
    s32 len, ret;

    static s8 fprf_buffer[FIFOLEN];

    // TODO: delete if (fifo < 0)
	// TODO: delete return;

    va_start(ap, fmt);
    len = vsnprintf(fprf_buffer, FIFOLEN, fmt, ap);
    va_end(ap);

    // TODO: delete ret = rtf_put(fifo, fprf_buffer, len);
    ret = rt_pipe_write(fifo,  fprf_buffer, len, P_NORMAL);
    // for hairy debug
    // printk(fprf_buffer);
} 

// maybe this should be malloc'd, but we shouldn't be printing
// more than 16k bytes of error per cycle.

static s8 dprbuf[FIFOLEN];    // Buffer that holds the debug messages
static s8 *dprbufp = dprbuf;  // Pointer to the end of the buffer

static void dpr1(const s8 *, ...);
static void dpr2(const s8 *, va_list);


/**
 * print debug messages
 * if level > ob->debug_level
 * save them up in dprbuf until they are dpr_flushed.
 * it's easier to use helper functions, so that dpr can "call itself."
 */

void
dpr(s32 level, const s8 *format, ...)
{
    va_list args;

    //if (level!=NULL) {
      // FVV commented these next lines as it seems to cause a segfault
    //syslog(LOG_INFO,"ob->debug_level %d.\n",ob->debug_level); 
    //syslog(LOG_INFO,"level %d.\n",level); 
    //syslog(LOG_INFO,"ob->i %d.\n",ob->i); 
    //syslog(LOG_INFO,"ob->Hz %f.\n",ob->Hz); 

    if (level > ob->debug_level)
      return;
    if (level >= 1) {
      if (ob->Hz && ob->i) {
	if (ob->i % ob->Hz != 0)
	  return;
      }
    }

    /* If the buffer is almost full, flush it to prevent overflow. */
    // note that we can't use sprintf in the kernel, hmmm.
    if ((dprbufp - dprbuf) > (FIFOLEN - 1024)) {
	dpr1("\n<<dprbuf almost full>>\n");
	dpr_flush();
    }
    // if (level > 0) dpr1("%d:", level);

    

    //dpr1(format, args);
    /* Just for prettiness, add a prefix that will indicate the
       debug level. */
    //if (level!=NULL) {
    dpr1("[%d] ",level);
    if (ob->times.ms_since_start) {
      dpr1("%10d ms ",ob->times.ms_since_start);
    } else {
      dpr1("              ");
    }
    int i;
    for (i=0; i<=level; i++) 
      dpr1("-");
    //dpr1(dbg);
    //}
    dpr1(" ");

    va_start(args, format);
    dpr2(format, args);
    va_end(args);
}




// dpr1 helper function takes a normal arglist

static void
dpr1(const s8 *format, ...)
{
    va_list args;
    s32 len;

    va_start(args, format);
    len = vsprintf(dprbufp, format, args);
    dprbufp += len;
    va_end(args);
}



/// dpr2 helper function takes a va_list

static void
dpr2(const s8 *format, va_list args)
{
    s32 len;

    len = vsprintf(dprbufp, format, args);
    dprbufp += len; // update the pointer that points to the end of the buffer
}



/// clear the debug buffer

void
dpr_clear()
{
  dprbufp = dprbuf; // Point to the beginning of the debug buffer
  dprbuf[0] = (intptr_t) NULL; // otherwise throws pointer conversion error
}



/**
 * @brief write out the buffer for messages (dprbuf)
 *
 * This will call rt_pipe_write and push any messages to the corresponding /dev/rtpX output
 * device pipe.
 */

void
dpr_flush()
{
    s32 len;
    // TODO: delete if proves unneeded    s32 ret;
    s32 ret; // to stop warning  TODO: delete (previously this was set to zero, perhaps to quelch error messages?)

    /* Determine the length of the buffer to-be-flushed, quit if nothing */
    len = dprbufp - dprbuf;
    if (len <= 0)
	return;

    // fprf(ob->eofifo, "\n\ndpr_flush: writing %d chars\n", len);

    // TODO: delete ret = rtf_put(ob->eofifo, dprbuf, len);
    ret = rt_pipe_write( &(ob->eofifo), dprbuf, len, P_NORMAL);
    dpr_clear();
    if (ret != len) {
      syslog(LOG_INFO,"%s:%d %d return from rt_pipe_write()\n", __FILE__, __LINE__, ret);
      fprf(&(ob->eofifo), "dpr_flush: ret = %d, len = %d\n", ret, len);
    }
    // usleep(100*1000);

    // fprintf(stderr, "dpr_flush: done\n");
}
