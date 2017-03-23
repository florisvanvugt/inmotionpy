// pwrdaq_dummy1.c - InMotion2 main loop
// dummy version of pwrdaq.c for when the pwrdaq load fails
// when there is no uei board.
//
// InMotion2 robot system software for RTLinux
//
// Copyright 2003-2005 Interactive Motion Technologies, Inc.
// Cambridge, MA, USA
// http://www.interactive-motion.com
// All rights reserved

// this is caled dummy1 because dummy1.o is the plain output of cc,
// dummy.o has gone through ld -r and is ready to be loaded as an lkm.

#ifdef __KERNEL__

#undef __NO_VERSION__

//#include <linux/config.h>
#include <generated/autoconf.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>

#include <linux/mm.h>
#include <linux/proc_fs.h>

#include <linux/device.h>

MODULE_DESCRIPTION("dummy pwrdaq module");
MODULE_AUTHOR("IMT interactive-motion.com");
MODULE_LICENSE("GPL");  // This is a trivial module, being GPL keeps
			// us from tainting the kernel.

int pd_board = 0;
int num_pd_boards = 0;

static int fent_out(char *buffer, char **start, off_t offset, 
			int length) {
	int len;
	static int count=1;
	static char bufo[40];
	if(offset>0)
		return 0;

	len=sprintf(bufo,
		    "This is a dummy pwrdaq module for use when an actual \n"
		    "powerdaq board is not present.\n");
	count++;
	*start=bufo;
	return len;
}		

int powerdaq_dummy_init_module (void) {
  //create_proc_info_entry("pwrdaq", 0, NULL, fent_out);
  //tentry = proc_create("pwrdaq", 0, NULL, fent_out);
  proc_create("pwrdaq", 0, NULL, fent_out);
  printk(KERN_INFO "Hello world from powerdaq dummy module.\n"); // this goes on dmesg trace
  return 0;
}
void powerdaq_dummy_cleanup_module (void) {
  remove_proc_entry("pwrdaq", NULL);
  printk(KERN_INFO "Cleaning up powerdaq dummy module.\n");
  return;
}

module_init(powerdaq_dummy_init_module);
module_exit(powerdaq_dummy_cleanup_module);

int pd_ain_set_config(int board, u32 a, u32 b, u32 c) { return -1; }
int pd_ain_set_channel_list(int board, u32 num_entries, u32 list[]) { return -1; }
int pd_ain_sw_start_trigger(int board) { return -1; }
int pd_ain_set_enable_conversion(int board, int enable) { return -1; }
int pd_ain_get_samples(int board, int max_samples, u16* buffer, u32* samples) { return -1; }
int pd_ain_reset(int board) { return -1; }
int pd_ain_sw_cl_start(int board) { return -1; }
int pd_ain_reset_cl(int board) { return -1; }
int pd_aout_reset(int board) { return -1; }
int pd_aout_put_value(int board, u32 dwValue) { return -1; }
int pd_din_read_inputs(int board, u32 *pdwValue) { return -1; }
int pd_dout_write_outputs(int board, u32 val) { return -1; }
int pd_ao32_writex(int board, u32 dwDACNum, u16 wDACValue, int dwHold, int dwUpdAll) { return -1; }
int pd_ao32_reset(int board) { return -1; }
int pd_ao32_write(int board, u16 wChannel, u16 wValue) { return -1; }
int pd_ao32_write_hold(int board, u16 wChannel, u16 wValue) { return -1; }
void pd_process_pd_ain_get_samples(int board, int bFHFState) { return; }
void pd_init_pd_board(int board) { return; }

#endif // KERNEL
