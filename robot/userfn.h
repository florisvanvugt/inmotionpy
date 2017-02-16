// userfn.h - includes for userfn.c

// InMotion2 robot system software for RTLinux

// Copyright 2003-2005 Interactive Motion Technologies, Inc.
// Cambridge, MA, USA
// http://www.interactive-motion.com
// All rights reserved

void read_data_fifo_sample_fn(void);
void write_data_fifo_sample_fn(void);
void write_motor_test_fifo_sample_fn(void);
void write_display_fn(void);
void write_actuators_fn(void);
void check_safety_fn(void);
void compute_controls_fn(void);
void output_controls_fn(void);
void init_log_fns(void);
void init_slot_fns(void);
