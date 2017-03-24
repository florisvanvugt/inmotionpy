// fifo.c - perform i/o to real time fifos
// part of the robot.o Linux Kernel Module

// InMotion2 robot system software for RTLinux

// Copyright 2003-2005 Interactive Motion Technologies, Inc.
// Cambridge, MA, USA
// http://www.interactive-motion.com
// All rights reserved

#include "rtl_inc.h"
#include "ruser.h"
#include "robdecls.h"
#include "pipes.h"

// setup_fifos - create fifos for doing i/o between user mode
// and the linux kernel robot control module.
// TODO: delete // uses the traditional stdin/stdout/stderr numbering, with 3 for command-in


#define POOLSIZE 1000000 /* Not sure what this setting should be */


void
init_fifos(void)
{
    int ret;
    static char ibuffer[FIFOLEN];

    // make sure they're not in use.
    cleanup_fifos();

    // data input from user space
    // TODO: delete ob->dififo = 0;
    // TODO: delete rtf_create(ob->dififo, ob->fifolen);
    ret = rt_pipe_create(&(ob->dififo), DIFIFO_NAME, DIFIFO_MINOR, POOLSIZE);
    if (ret < 0) {
      printf("%s:%d %d return from rt_pipe_create()\n", __FILE__, __LINE__, ret);
      cleanup_signal(0);
    }

    // data output to user space
    // TODO: delete ob->dofifo = 1;
    // TODO: delete rtf_create(ob->dofifo, ob->fifolen);
    ret = rt_pipe_create(&(ob->dofifo), DOFIFO_NAME, DOFIFO_MINOR, POOLSIZE);
    if (ret < 0) {
      printf("%s:%d %d return from rt_pipe_create()\n", __FILE__, __LINE__, ret);
      cleanup_signal(0);
    }

    // error output to user space
    // TODO: delete ob->eofifo = 2;
    // TODO: delete rtf_create(ob->eofifo, ob->fifolen);
    ret = rt_pipe_create(&(ob->eofifo), EOFIFO_NAME, EOFIFO_MINOR, POOLSIZE);
    if (ret < 0) {
      printf("%s:%d %d return from rt_pipe_create()\n", __FILE__, __LINE__, ret);
      cleanup_signal(0);
    }

    // command input from user space
    // TODO: delete ob->cififo = 3;
    ob->ci_fifo_buffer = ibuffer;
    // TODO: delete rtf_create(ob->cififo, ob->fifolen);
    // TODO: delete rtf_create_handler(ob->cififo, fifo_input_handler);
    ret = rt_pipe_create(&(ob->cififo), CIFIFO_NAME, CIFIFO_MINOR, POOLSIZE);
    if (ret < 0) {
      printf("%s:%d %d return from rt_pipe_create()\n", __FILE__, __LINE__, ret);
      cleanup_signal(0);
    }

    // display data to user space
    // TODO: delete ob->ddfifo = 4;
    // TODO: delete rtf_create(ob->ddfifo, ob->fifolen);
    ret = rt_pipe_create(&(ob->ddfifo), DDFIFO_NAME, DDFIFO_MINOR, POOLSIZE);
    if (ret < 0) {
      printf("%s:%d %d return from rt_pipe_create()\n", __FILE__, __LINE__, ret);
      cleanup_signal(0);
    }

    // tick data to user space
    // TODO: delete ob->tcfifo = 5;
    // TODO: delete rtf_create(ob->tcfifo, ob->fifolen);
    ret = rt_pipe_create(&(ob->tcfifo), TCFIFO_NAME, TCFIFO_MINOR, POOLSIZE);
    if (ret < 0) {
      printf("%s:%d %d return from rt_pipe_create()\n", __FILE__, __LINE__, ret);
      cleanup_signal(0);
    }

    // cleanup_fifos uses this
    // TODO: delete ob->nfifos = ob->tcfifo + 1;

    syslog(LOG_INFO, "init_fifos: done\n");

}

// clean up fifos created above

void
cleanup_fifos(void)
{
    // TODO: delete fs32 i;
    int ret;

    dpr_flush(); // flush anything that may be in the pipelines
    
    // TODO: delete for (i = 0; i < ob->nfifos; i++)
    // TODO: delete 	rtf_destroy(i);
    ret = rt_pipe_delete(&(ob->dififo));
    if (ret < 0) {
      dpr(0, "%s:%d %d return from rt_pipe_delete()\n", __FILE__, __LINE__, ret);
    }
    ret = rt_pipe_delete(&(ob->dofifo));
    if (ret < 0) {
      dpr(0, "%s:%d %d return from rt_pipe_delete()\n", __FILE__, __LINE__, ret);
    }
    ret = rt_pipe_delete(&(ob->eofifo));
    if (ret < 0) {
      dpr(0, "%s:%d %d return from rt_pipe_delete()\n", __FILE__, __LINE__, ret);
    }
    ret = rt_pipe_delete(&(ob->cififo));
    if (ret < 0) {
      dpr(0, "%s:%d %d return from rt_pipe_delete()\n", __FILE__, __LINE__, ret);
    }
    ret = rt_pipe_delete(&(ob->ddfifo));
    if (ret < 0) {
      dpr(0, "%s:%d %d return from rt_pipe_delete()\n", __FILE__, __LINE__, ret);
    }
    ret = rt_pipe_delete(&(ob->tcfifo));
    if (ret < 0) {
      dpr(0, "%s:%d %d return from rt_pipe_delete()\n", __FILE__, __LINE__, ret);
    }

    syslog(LOG_INFO, "Cleaned up FIFOs (cleanup_fifos)\n");

}


/// fifo_input_handler
/// gets invoked when the command fifo has input ready.
///
/// overwrite previous buffer for now.  (does not append)

void
fifo_input_handler(void)
{
    s32 ret;

    // TODO: delete     ret = rtf_get(fifo, ob->ci_fifo_buffer, ob->fifolen);
    ret = rt_pipe_read(&(ob->cififo), &ob->ci_fifo_buffer, ob->fifolen, TM_NONBLOCK);
    if (ret != -EWOULDBLOCK && ret < 0) {
      dpr(0,          "%s:%d %d return from rt_pipe_read()\n", __FILE__, __LINE__, ret);
      syslog(LOG_INFO,"%s:%d %d return from rt_pipe_read() which I believe is %d\n", __FILE__, __LINE__, ret, -ESRCH);
      if (ob->times.ms_since_start) {
	//dpr1("%10d ms ",);
	//syslog(LOG_INFO,"ob->cififo %d\n", &(ob->cififo));
	syslog(LOG_INFO,"time is %d ms\n", ob->times.ms_since_start);
      }

      /*
      RT_TASK_INFO thread_info;
      ret = rt_task_inquire(NULL, &thread_info);
      syslog(LOG_INFO,"thread_info %s status %d page faults %d",thread_info.name,thread_info.status,thread_info.pagefaults);
      */
      
      cleanup_signal(0);  // Stop functioning, something went wrong.
    }

    if (ret>0) {
      syslog(LOG_INFO,"%s:%d %d return from rt_pipe_read()\n", __FILE__, __LINE__, ret);
    }

    // append null to string
    ob->ci_fifo_buffer[ob->fifolen] = 0;
    if (ret >= 0 && ret < ob->fifolen)
	ob->ci_fifo_buffer[ret] = 0;

    return;
}



/// Print what information was received over the FIFO input.
void
print_fifo_input(void)
{
    if (&(ob->ci_fifo_buffer[0]) != 0) {
	prf(&(ob->eofifo), "\n##### fifo input: %s\n", ob->ci_fifo_buffer);
    }
}




/// handle_fifo_input - act on data in the command input fifo.
///
/// the command input fifo buffer is filled by an async input handler,
/// but we check the command input buffer once per sample.

void
handle_fifo_input()
{
    // int ret;

  fifo_input_handler(); // Under RTLinux this was called by RTLinux,
			  // now we call it as part of the polling.
                          // TODO: fix this architectural history.
  switch (ob->ci_fifo_buffer[0]) {
  case 0:			// no input (this gets called every sample)
    return;
    
  case 'c':
    dpr(0, "continuing thread...\n");
    ob->paused = 0;
    clear_sensors();
    break;
    
  case 'g':
    dpr(0, "trying aout...\n");
    // test_uei_write();
    break;
    
  case '\n':
  case 'h':
    dpr(0, "help:\n");
    dpr(0, "c - continue sampling\n");
    dpr(0, "h - this help\n");
    dpr(0, "i - init system and restart\n");
    dpr(0, "k - unload module (kill thread)\n");
    dpr(0, "p - pause sampling\n");
    dpr(0, "t - sanity test (while paused)\n");
    dpr(0, "^C to exit iofifo\n");
    break;
    
  case 'i':
    dpr(0, "init - restarting thread...\n");
    ob->paused++;
    main_init();
    break;
    
  case 'k':
    dpr(0, "killing thread...\n");
    // TODO: delete unload_module();
    cleanup_signal(0);
    break;
    
  case 'p':
    dpr(0, "pausing thread...\n");
    ob->paused++;
    break;
    
  case 't':
    main_tests();
    break;
    
  default:
    print_fifo_input();
    break;
  }
  // prompt
  dpr(0, "\n> ");
  ob->ci_fifo_buffer[0] = 0;
}
