// robdecls.h - data declarations for the InMotion2 robot software system
//

// InMotion2 robot system software for RTLinux

// Copyright 2003-2005 Interactive Motion Technologies, Inc.
// Cambridge, MA, USA
// http://www.interactive-motion.com
// All rights reserved

#ifndef ROBDECLS_H
#define ROBDECLS_H

//#include "ruser.h"
#include <math.h>

// close to zero, for double compares.
#define EPSILON 0.0000001

#define ROBOT_LOOP_THREAD_NAME "IMT Robot Control Loop"

#define ARRAY_SIZE(a) (sizeof (a) / sizeof ((a)[0]))
#define MAX(a,b) ((a) > (b) ? (a) : (b))
// u8/s8 etc. are defined in types.h

// I tripped over an f32, so I'm removing them
// typedef float f32;
#define f32 woops!
typedef double f64;

#define FIFOLEN 0x4000   // i.e. 16384


// For trajectory replaying, this is the maximum length of the trajectory position array.
#define TRAJECTORY_BUFFER_SIZE 3000  // should match the number of items in ob->trajx and ob->trajy


// errors, see main.c and uei.c
// if you change this, you MUST change tools/errpt

enum {
	ERR_NONE=0,
	ERR_MAIN_LATE_TICK,
	WARN_MAIN_SLOW_TICK,
	WARN_MAIN_SLOW_SAMPLE,
	ERR_UEI_NSAMPLES,
	ERR_UEI_RET,
	ERR_UEI_BOARD_RANGE,
	ERR_UEI_BAD_ARRAY_PTRS,
	WARN_MAIN_FAST_TICK,
	ERR_AN_HIT_STOPS,
	ERR_AN_SHAFT_SLIP_LEFT,
	ERR_AN_SHAFT_SLIP_RIGHT,
	ERR_PL_ENC_KICK,
	ERR_LAST
};

// math

// 2d
typedef struct xy_s {
	f64 x;
	f64 y;
} xy;

// 3d
typedef struct xyz_s {
	f64 x;
	f64 y;
	f64 z;
} xyz;

// shoulder/elbow pair
typedef struct se_s {
	f64 s;
	f64 e;
} se;

// 2x2 matrix
typedef struct mat22_s {
    f64 e00;
    f64 e01;
    f64 e10;
    f64 e11;
} mat22;

#ifdef NOTDEF
// 2x2 box center/w/h
typedef struct box_cwh_s {
	xy c;
	xy wh;
} box_cwh;


// 2x2 box x1y1/x2y2
typedef struct box_xy_s {
	xy p1;
	xy p2;
} box_xy;
#endif // NOTDEF

// performance metrics
typedef struct pm_s {
    f64 active_power;       // pm2a
    f64 min_jerk_deviation; // pm2b
    f64 min_jerk_dgraph; // graph
    f64 dist_straight_line; // pm3
    f64 max_dist_along_axis; // pm4
    u32 npoints;            // npts
    u32 five_d;             // planarwrist adaptive
} PM;


// linear types

typedef struct linear_mattr_s {    // linear motor attributes
    u32 enc_channel;	// counter card encoder channel
    f64 disp;		// displacement
    f64 devfrc;		// output forces
    f64 xform;		// xform from force to voltage
    f64 volts;		// output voltage
    f64 test_volts;	// test voltage
    u32 ao_channel;	// ao board output voltage channel
    u32 limit_channel;
    f64 limit_volts;
} linear_MAttr;

typedef struct linear_gears_s {        // linear gear ratios from motor to world
    f64 ratio;
} linear_gears;

typedef struct linear_s {   // linear
    linear_MAttr motor;
    linear_gears gears;
    u32 uei_ao_board_handle;
} Linear;

typedef struct linear_ob_s {    // linear world coordinates
    f64 pos;
    f64 vel;
    f64 fvel;
    f64 force;
    f64 back;	// back wall for adap
    f64 norm;	// normalized posn for adap
    f64 offset;
    f64 ref_pos;
    f64 stiff;
    f64 damp;
    f64 force_bias;
    f64 pfomax;		// pfo
    f64 pfotest;	// pfo
    u32 adap_going_up;
} linear_ob;

typedef struct linear_prev_s {   // previous linear world coordinate parameters
    f64 pos;
    f64 vel;
    f64 fvel;
} linear_prev;


// globals
// robot attributes

// motor attributes
typedef struct mattr_s {
    // input
    u32 channel;
    f64 xform;
    f64 offset;

    // output
    f64 raw;	// raw data
    f64 rad;	// radians
    f64 deg;	// degrees
    f64 cal;	// calibration value in radians
} MAttr;

// motor
typedef struct motor_s {
    MAttr angle;
    MAttr vel;
    MAttr torque;
} Motor;

// Force Transducer
typedef struct ft_s {
    u32 flip;	// ft is installed upside down
    u32 vert;	// ft is installed vertically as on integrated planar wrist
    u32 channel[6];	// adc channels

    f64 offset;	// ft rotation in radians

    f64 curr[6];	// values after conversion to world space
    f64 prev[6];	

    f64 filt[6];	// butterworth of curr
    f64 prevf[6];

    xyz world;	// filtered values world space
    xyz dev;	// filtered values device space
    xyz moment;	// moment values

    f64 xymag;	// force magnitude in xy

    u32 dobias;		// copy current raw to bias
    f64 raw[6];		// raw unbiased (for calibration)
    f64 cooked[6];	// voltages after bias is applied
    f64 bias[6];	// bias voltages
    f64 scale[6];	// scale and cal matrixes
    f64 cal[6][6];	//   supplied with each ft by manufacturer

    f64 avg[6];		// various filtering choices
    f64 but[6];
    f64 sg[6];
    f64 sghist[6][64];
    f64 bsrawhist[6][16];
    f64 bsfilthist[6][16];
    f64 bs[6];
} Ft;

// ATI ISA Force Transducer
typedef struct isaft_s {
	u16 cpf;	// (12-bit) counts per force (default units)
	u16 cpt;	// (12-bit) counts per torque (default units)
	u16 units;	// the default units for the calibration
				// (1=lbf,lbf-in; 2=N,N-mm; 3=N,N-m;
				// 4=Kg,Kg-cm; 5=Klbf,Klbf-in)

	s32 iraw[8];	// raw 12 bit data 
	f64 raw[8];	// raw voltage
} ISAFt;

// IMT Grasp sensor
typedef struct grasp_s {
	u32 channel;	// adc channel
	f64 raw;	// volts before bias
	f64 bias;	// bias
	f64 cal;	// calibration (raw * gain)
	f64 gain;
	f64 force;
	f64 press;	// press voltage (hwm)
	f64 release;	// release voltage (lwm)
} Grasp;

// Accelerometer
typedef struct accel_s {
	u32 channel[3];	// adc channels

	f64 raw[3];	// voltages after bias is applied

	f64 curr[3];	// raw values after conversion to world space
	f64 prev[3];	

	f64 filt[3];	// butterworth of curr
	f64 prevf[3];

	xyz world;	// filtered values world space
	xyz dev;	// filtered values device space

	f64 bias[3];	// bias voltages
	f64 xform;	// xform
} Accel;

typedef struct csen_s {
	u32 have;		// have up to 8
	u32 channela[8];	// a channel
	u32 channelc[8];	// c channel
	f64 rawa[8];		// a component in
	f64 rawc[8];		// c component in
	f64 xforma[8];		// xform from raw to a
	f64 xformc[8];		// xform from raw to c
	f64 kt[8];		// kt:  current to torque
	f64 kt_biasa[8];	// kt bias a
	f64 kt_biasc[8];	// kt bias c
	f64 alpha[8];		// alpha
	f64 curr[8];		// output current
	f64 torque[8];		// output torque
} CSen;

// robot
typedef struct robot_s {
  s8 tag[8];	// unused
  
  f64 ain_07;
  u32 ain_07_channel;
  
  Motor shoulder;
  Motor elbow;
  
  se link;
  
  xy offset;
  
  Ft ft;
  
  ISAFt isaft;
  
  Grasp grasp;
  
  Accel accel;
  
  CSen csen;
  
  Linear linear;
  
} Robot;

// time vars

typedef struct time_s {
  /* FVV 20190404 changed the data type for the variables below from hrtime_t to SRTIME, following Xenomai docs.*/
  SRTIME time_at_start;
  SRTIME time_before_sample;
  SRTIME time_after_sample;
  SRTIME time_before_last_sample;
  SRTIME time_after_last_sample;
  SRTIME time_delta_sample;
  SRTIME time_delta_tick;
  SRTIME time_delta_call;
  SRTIME time_since_start;
  
  u32 ns_delta_call;
  u32 ns_delta_tick;
  u32 ns_delta_tick_thresh;
  u32 ns_delta_sample;
  u32 ns_delta_sample_thresh;
  u32 ms_since_start;
  u32 sec;
  
  u32 ns_max_delta_tick;
  u32 ns_max_delta_sample;
} Timev;

// data that gets copied to ob atomically, when you set go

typedef struct restart_s {
    u32 go;			// write this last!
    u32 Hz;			// when you're ready to go,
    u32 ovsample;
    f64 stiff;			// copy all this to ob.
    f64 damp;
} Restart;

typedef struct ref_s {
    xy pos;
    xy vel;
} Ref;

typedef struct max_s {
    xy vel;
    xy motor_force;
    se motor_torque;
} Max;

// this needs to be extended for > 2d position descriptions

typedef struct box_s {
    xy point;
    f64 w;	// width
    f64 h;	// height
} box;

typedef struct slot_s {
    u32 id;			// id (for copy_slot)

    // a la for loop
    u32 i;			// the incremented index
    u32 incr;			// amount to increment each sample
    u32 term;			// termination
    u32 termi;			// index incremented after termination
    				// for making controls stiffer, etc.

    box b0;			// initial position
    box b1;			// final position
    box bcur;			// current position

    f64 rot;			// slot rotation in radians
    u32 fnid;			// index into slot fn * table
    u32 running;		// this slot is running
    u32 go;			// go (for copy_slot)
} Slot;



// safety envelope
typedef struct safety_s {
  f64 pos;

  // The positions below specify the minimum and maximum
  // "safe" values for x and y. Once the robot gets outside
  // these, the safety features activate.
  f64 minx;
  f64 maxx;
  f64 miny;
  f64 maxy;
  
  f64 vel;
  f64 torque;
  f64 ramp;
  f64 damping_nms;
  f64 velmag_kick;
  u32 override;

  xy motor_force; // if the safety motor force has applied, this is what its values are
  
  u32 planar_just_crossed_back;
  u32 was_planar_damping;
  u32 active; // whether safety damping is currently active
  u32 has_applied; // This counts the number of frames where the safety boundary was enforced
  u32 damp_ret_ticks;
  f64 damp_ret_secs;
} Safety;




#define N_UEIDAQ_BOARDS 4
#define UEI_SAMPLES 16
#define DAQ_ACQUIRE 1
#define DAQ_RELEASE 0


// contains s32 data from the daq
typedef struct daq_s {
    s8 tag[8];	// unused
    s32 n_ueidaq_boards;	// number of UEI boards in system
    s32 uei_board[4];    // mapping from logical (array) to physical (bus)
    s32 ain_handle[4];   // Handles returned by PdAcquireSubsystem
    s32 aout_handle[4];
    s32 din_handle[4];   // Handles returned by PdAcquireSubsystem
    s32 dout_handle[4];
    s32 ao8_handle[4];
    // TODO: delete u16 subdevid[4];     // sub device ID to identify board
    u32 adapter_type[4];     // adapter type to identify board

    s32 ain_cl_size;		// channel list size
    s32 ain_ret;		// return from pd_ain_get_samples()
    u32 ain_got_samples;	// number of samples returned
    u32 ain_cfg;		// SINGLE_ENDED?  5/10V?
    f64 ain_bias_comp[4]; 	// bias voltage compensation ~ .018

    // in the shoulder/elbow robot:
    // 2 dio vars are used for encoder input
    // up to 16 adc are used for tach input, etc.
    // 2 dac are used for torque output to motors

// up to 4 boards * 16 samples each.
// single dim arrays are easier to handle here.

    // two-dimensional arrays
    u16 m_dienc[4][2];		// digital inputs
    u16 m_dout_buf[4];		// digital outputs
    u16 m_adc[4][16];	// analog inputs
    u16 m_dac[4][16];	// analog outputs

    f64 m_adcvoltsmean[4][16];	// adc mean
    f64 m_adcvoltsmed[4][16];	// adc median

    f64 m_adcvoltsprev[4][16];	// adc filter stuff
    f64 m_adcvoltsfilt[4][16];
    f64 m_adcvoltspfilt[4][16];

    f64 m_adcvolts[4][16];
    f64 m_dacvolts[4][16];

    // one dimensional pointers, see main..c:main_init()
    u16 *dienc;
    u16 *dout_buf;
    u16 *adc;
    u16 *dac;

    f64 *adcvolts;
    f64 *dacvolts;

    f64 *adcvoltsmean;
    f64 *adcvoltsmed;

    f64 *adcvoltsprev;
    f64 *adcvoltsfilt;
    f64 *adcvoltspfilt;

    // the two digital output bits on the junction box
    u16 dout0;
    u16 dout1;

    // di status bits
    u16 distat[2];

    // digital inputs, used for reading planar encoders
    u16 prev_dienc[2];		// for calculating velocity
    s32 dienc_vel[2];		// raw velocity
    s32 prev_dienc_vel[2];	// for calculating accelleration
    s32 dienc_accel[2];		// raw accelleration

    u32 diovs;	// encoder oversampling
} Daq;

// this is the dynamic compensation structure REWT
typedef struct dyncmp_var_s {
  
  u32 usedirectcontrol;        // variable set in by dyncomp.c
  
  xy control_force;
  se anglevelocity;            // REWT: added angle velocity in robot coords used in dyncomp.c
  se  Corriolis;               // calculated in dyncomp
  mat22 Bofq;                  // robot mass matrix
  mat22 Mofq;                  // task space mass matrix (J^-T B J^-1)

} Dyncmp_var;


// the main object struct

typedef struct ob_s {
  s8 tag[8];			// unused
  /* TODO: delete pthread_t main_thread; */	// main thread (currently only thread)
  RT_TASK main_thread;
  
  u32 doinit;			// run init code if set.
  u32 didinit;		// so we only init once
  u32 quit;                   // quit program if set.
  
  u32 i;			// loop iteration number.
  u32 fasti;			// fast tick count for ft
  u32 samplenum;		// sample number.
  
  u32 total_samples;		// stop after this many
  
  u32 Hz;			// samples per second
    				// following are derived.
  f64 rate;			// 1.0/(samples per second),
  u32 irate;			// BILLION/(samples per second),
    				// i.e. nanoseconds per sample
  
				// for oversampled ft sampling
  u32 ovsample;		// oversampling multiplier
    				// set ovsample, the following are derived.
  u32 fastHz;			// fast samples per second
  f64 fastrate;		// 1.0/(samples per second),
  u32 fastirate;		// BILLION/(samples per second),
    				// i.e. nanoseconds per sample
  
  u32 ticks30Hz;		// number of samplenum ticks in 30Hz,
    				// for display stuff
  
  f64 stiff;			// stiffness for controller
  f64 damp;			// damping
  f64 curl;			// curl
  
  xy const_force;		// constant force control
  
  f64 side_stiff;		// side stiffness for adapative controller
  
  f64 pfomax;			// preserve force orientation max
  f64 pfotest;	   	        // pfo test value
  
  u32 busy;			// sample is not in sleep wait.
  u32 paused;			// tick clock, but don't write actuators
  u32 fault;			// control loop triggered fault
  s32 stiffener;		// a stiffness % increment
				// 0 is normal stiffness
				// -100 is no stiffness
				// 100 is double, 200 is triple
			        // generalizing slot_term_stiffen
  s32 stiff_delta;		// how much to add to stiffener each tick
  u32 no_motors;		// never write torques
  
  Restart restart;		// copied into Ob on restart.
  
  Timev times;		// timing vars
  
  u32 have_tach;		// we have a tachometer
  u32 have_ft;		// we have a non-ISA force transducer
  u32 have_isaft;		// we have an ISA force transducer
  u32 have_accel;		// we have an acceleromter
  u32 have_grasp;		// we have a grasp sensor
  u32 have_planar;		// we have a planar robot
  u32 have_wrist;		// we have a wrist robot
  u32 have_ankle;		// we have an ankle robot
  u32 have_linear;		// we have an linear robot
  u32 have_hand;		// we have a hand robot
  u32 have_planar_incenc;	// we have a planar with incr encoders 
  u32 have_planar_ao8;	// we have a planar with ao8 output
  
  Slot copy_slot;		// for input from shm
  Slot slot[8];		// slot control
  PM pm;			// performance metrics for adaptive
  void (*slot_fns[32])(u32); 	// array of slot functions
  
  u32 slot_max;		// max number of slots;
  
  f64 pi;			// pi (make sure trig works)
  
  u32 nlog;			// number of items to write out
  f64 log[32];		// array of items to write out
  void (*log_fns[32])(void); 	// array of log functions
  u32 logfnid;		// which log function in array
  
  u32 ndisp;			// number of items to write out
  f64 disp[32];		// array of items to write out
  
  u32 nref;			// number ofdouble inforcex,double inforcey,int method, double diagD) items to read in
  f64 refin[32];		// array of items to read
  void (*ref_fns[32])(void); 	// array of ref functions
  u32 reffnid;		// which log function in array
  
  RT_PIPE dififo;	       	// data in (like stdin)
  RT_PIPE dofifo;	       	// data out (stdout)
  RT_PIPE eofifo;	       	// error out (stderr)
  RT_PIPE cififo;	       	// command in
  RT_PIPE ddfifo;	       	// display data out
  RT_PIPE tcfifo;             // tick data out
  // TODO: delete s32 nfifos;			// number of fifos
  
  u32 ntickfifo;		// do the tcfifo output
  
  u32 fifolen;		// fifo buffer size
  s8 *ci_fifo_buffer;		// pointer to command input fifo buffer
    				// (handled differently from other fifos)
  
  Safety safety;		// safety zone variables
  
  u32 vibrate;		// random vibration factor for testing
  s32 xvibe;			// vibration components
  s32 yvibe;
  
  xy pos;			// world position
  xy tach_vel;		// world velocities from hardware tach
  xy ftach_vel;		// filtered tach_vel
  xy soft_vel;		// world velocities from position
  xy fsoft_vel;		// filtered soft_vel
  xy vel;			// assigned from one of the vels
  xy motor_force;		// world forces sent to motors, from controller
  se motor_torque;		// device torques from motor_force
  se motor_volts;		// device volts from motor_torque
  xy back;			// back wall for adap
  xy norm;			// normalized posn for adap
  
  f64 velmag;			// magnitude of the velocity
  
  xy soft_accel;		// accel derived from position
  f64 soft_accelmag;		// accel magnitude
  
  Ref ref;			// references for logfile playback
  
  se theta;			// encoder angles
  se thetadot;		// angular velocity
  se fthetadot;		// filtered thetadot
  
  linear_ob linear;
  
  u32 test_raw_torque;	// raw torque test mode;
  se raw_torque_volts;	// raw volts for testing
  
  //Sim sim;			// simulate sensors
  
  f64 sin_period;		// for the sinewave generator
  f64 sin_amplitude;		// amplitude
  u32 sin_which_motor;	// shoulder, elbow, neither, both
  
  u32 butcutoff;		// butterworth cutoff W(n) * 100
    				// for 200 Hz, 15Hz cutoff, use 15
    				// (2 * 15 Hz cutoff) / 200 Hz
  
  Max max;			// maxima
  
    				// see main:do_error() and calls to it.
  u32 errnum;			// error this sample (couldn't call it errno)
  u32 nerrors;		// cumulative number of errors
  u32 errori[128];
  u32 errorcode[128];
  u32 errorindex;		// index into error arrays
  
  // scr is for random debugging
  f64 scr[64];		// scratch registers
  // game is for when programs game processes to communicate
    // with each other while the lkm is loaded.
  f64 game[64];		// game registers

  f64 plg_debug;                 // for debugging
  f64 plg_ftzero[6];             // for zeroing the ATI force transducer
  u32 plg_ftzerocount;
  f64 plg_curlmat[4];            // 2x2 damping matrix for velocity-dependent curl force field
  f64 plg_stiffness;             // stiffness coefficient
  f64 plg_damping;               // damping coefficient
  f64 plg_p1x, plg_p1y;          // robot (p1x,p1y) for moving from
  f64 plg_p2x, plg_p2y;          // robot (p2x,p2y) for moving to
  f64 plg_movetime;              // move in this time
  u32 plg_counterstart;          // counter time when started
  u32 plg_moveto_done;           // we have reached the moveto pt
  f64 plg_last_fX;               // for ramping down forces gradually to zero
  f64 plg_last_fY;
  f64 plg_channel_width;         // width (m) of force channel
  
  u32 ba_accumforce;             // force accumulator state
  f64 mkt_mvtangle;              // current movement deflection angle in radians
  f64 mkt_finline;               // inline component of movement
  f64 mkt_fortho;                // orthogonal component of movement
  
  f64 ba_fx;                     //forces declaration 
  f64 ba_fy;                     //forces declaration 
  f64 ba_position;               //flag for hand position
  
  u32 mkt_isMcGill;              // true for McGill environment

  u32 fvv_trial_phase;           // the current phase in the trial
  u32 fvv_trial_no;              // the current trial number
  u32 fvv_move_done;             // whether the subject has completed their movement

  f64 fvv_robot_center_x;        // the robot center (x coordinate)
  f64 fvv_robot_center_y;        // the robot center (y coordinate)

  f64 fvv_force_fade;            // a multiplier for the force, used to fade the force

  f64 fvv_robot_min_y;           // the lower Y edge of the workspace
  f64 fvv_min_dist;              // the minimum distance that a subject must travel before we start determining the movement end based on the peak velocity
  f64 fvv_vel;                   // the maximum Y speed recorded so far
  f64 fvv_max_vel;               // the maximum Y speed recorded so far

  f64 fvv_vmax_x;                // the x coordinate of the maximum velocity point
  f64 fvv_vmax_y;                // the y coordinate of the maximum velocity point

  f64 fvv_final_x;                // the x coordinate of the final position
  f64 fvv_final_y;                // the y coordinate of the final position
  
  u32 fvv_trial_timer;           // a timer for the move phase (so we can cut off the trial when it takes too long)

  u32 fvv_vel_low_timer;         // times how long we are on low Y-velocity (below some percentage of the maximum velocity
  u32 fvv_subject_move_phase;    // a flag to indicate that this is the subject's active move phase

  u32 fvv_capture;               // whether we should capture or not
  
  u32 fvv_workspace_enter;       // 0 when outside of workspace, 1 when entered the workspace
  xy fvv_curl_force;	// world forces sent from the curl field controller (for debug)

  
  /* The variables below here are used for trajectory replaying */
  u32 traj_n_samps;            // how many samples the trajectory we have loaded contains
  u32 traj_count;               // how far we are currently in reproducing the trajectory
  f64 trajx[3000]; // used to store trajectories (x position values)
  f64 trajy[3000]; // used to store trajectories (y position values)
  f64 recordfx[3000];           // used to store force traces (x values)
  f64 recordfy[3000];           // used to store force traces (y values)
  f64 recordfz[3000];           // used to store force traces (z values)
  f64 traj_final_x;             // the final position of the replay, x
  f64 traj_final_y;             // idem, y
  u32 replay_done;              // whether replaying is done
  f64 replay_damping;           // damping to use during replaying
  f64 replay_stiffness;         // stiffness to use during replaying

  s32 debug_level;		// for dpr
  
  u32 last_shm_val;		// sanity check
} Ob;

typedef struct pcolor {
  float red;
  float green;
  float blue;
} Pcolor;

typedef struct game {
  // Shared memory variables for talking to saturn (space) game 3D engine.
  u32 go;  // Client sets this to indicate that space should look for changed data.

  f64 planet_color_red[9];   // For setting planet color, 0 is moving
			     // planet, 1 is N, 2 is NE, etc.  Between
			     // 0.0 and 1.0
  f64 planet_color_green[9];
  f64 planet_color_blue[9];
  u32 planet_color_go[9];    // Go bit for setting a planet's color.
  f64 planet_default_red;
  f64 planet_default_green;
  f64 planet_default_blue;

  f64 planet_rotation_aa[9]; // For setting a planet's angle, in
			     // radians.
  f64 planet_rotation_ps[9];
  f64 planet_rotation_fe[9];
  u32 planet_rotation_go[9]; // Go bit for setting a planet's angles.

  u32 repetition;            // Numerator in corner text display.
  u32 total_repetitions;     // Denominator in corner text display.
  u32 repetition_go;         // Go bit for changing text.

  u32 key_spacebar;          // Is set to 1 when the spacebar is
			     // pressed.  Clear after reading.
  u32 key_quit;              // Is set to 1 when q or esc is
			     // pressed.  Clear after reading.
  u32 key_b;                 // Is set to 1 when b is
			     // pressed.  Clear after reading.

  u32 target;                // For setting which planet is the
			     // target.  0 is center, 1 N, 2 NE, etc.
  u32 target_go;             // Go bit for setting target.
  u32 target_hit;            // Is set to 1 when the target is hit.
			     // Clear after reading.
  f64 feSlack;               // Thresholds for target hit criteria.
  f64 psSlack;
  f64 aaSlack;
  f64 planarSlack;
  u32 slack_go;              // Go bit for setting thresholds.

  u32 active_target;         // Whether we are hunting a target.
  u32 active_target_go;

  u32 quit_server;           // Tell 3D display to quit.
} Game;

// values of data from previous sample, for filters etc.
// these get filled after read_sensors is done.

typedef struct prev_s {
  s8 tag[8];	// unused
  xy pos;
  xy vel;
  
  se theta;
  se thetadot;
  se fthetadot;
  
  xy tach_vel;
  xy ftach_vel;
  
  linear_prev linear;

} Prev;

// functions that the user can over-ride

typedef struct func_s {
    s8 tag[8];	// unused
    void (*read_sensors) (void);
    void (*read_reference) (void);
    void (*compute_controls) (void);
    void (*check_safety) (void);
    void (*write_actuators) (void);
    void (*write_display) (void);
} Func;



typedef struct Moh_s {
  f64 pointer; 	
  
  u32 pixelx[36];
  u32 pixely[36];
  f64 realx[36];
  f64 realy[36];
  f64 xcenter;
  f64 ycenter;
  f64 ffs;
  f64 hand_place;
  f64 stiffness;
  f64 move_flag;
  f64 servo_flag;
  f64 counter;
  f64 current_dir;
  f64 last_pointX;
  f64 last_pointY;
  f64 fX;
  f64 fY;
  
} Moh;



// 0x494D5431 is 'IMT1'
#define OB_KEY   0x494D5431
#define ROB_KEY  0x494D5432
#define DAQ_KEY  0x494D5433
#define PREV_KEY 0x494D5434
#define GAME_KEY 0x494D5435
#define USER_KEY 0x494D5436
#define DYNCMP_KEY 0x494D5437

extern Ob *ob;
extern Daq *daq;
extern Prev *prev;
extern Robot *rob;
extern Game *game;
extern Dyncmp_var *dyncmp_var;
extern Moh *moh;

extern Func func;

void check_safety_fn(void);
void planar_check_safety_fn(void);
void wrist_check_safety_fn(void);

void user_init(void);

// these call the _fn functions that the user sets up in user_init
void call_read_sensors(void);
void call_read_reference(void);
void call_compute_controls(void);
void call_check_safety(void);
void call_write_actuators(void);
void call_write_log(void);
void call_write_display(void);

// slot.c
// void load_slot(u32, u32, u32, u32, void (*)(u32), s8 *);
void load_slot(u32, u32, u32, u32, u32, s8 *);
void do_slot(void);
void stop_slot(u32);
void stop_all_slots(void);

// dyncomp.c
void dynamics_compensation (double, double, int, double );

// main.c
void do_time_before_sample(void);
void handle_fifo_input(void);
void check_quit(void);
void do_error(u32);
void check_late(void);
void read_sensors_fn(void);
void clear_sensors(void);
void print_sample_times(void);
void do_time_after_sample(void);
void wait_for_tick(void);

s32 init_module(void);
// void cleanup_module(void);
void cleanup_signal(s32);

void unload_module(void);
void start_routine(void *);
void shm_copy_commands(void);
void main_tests(void);
void main_init(void);
void main_loop(void);

void docarr(void);

// math.c
xy jacob2d_x_p2d(mat22, se);
mat22 jacob2d_x_j2d(mat22, mat22);
mat22 jacob2d_inverse(mat22);
mat22 jacob2d_transpose(mat22);
xy xy_polar_cartesian_2d(se, se);
mat22 j_polar_cartesian_2d(se, se);
xy rotate2d(xy, f64);
xy xlate2d(xy, xy);
f64 xform1d(f64, f64, f64);
s32 ibracket(s32, s32, s32);
f64 dbracket(f64, f64, f64);
se preserve_orientation(se, f64);
f64 butter(f64, f64, f64);
f64 butstop(f64 *, f64 *);
f64 apply_filter(f64, f64 *);
f64 delta_radian_normalize(f64);
f64 radian_normalize(f64);
f64 min_jerk(f64, f64);
f64 i_min_jerk(u32, u32, f64);
f64 xasin(f64 x);
f64 xacos(f64 x);

// uei.c
void uei_ptr_init(void);
void uei_aio_init(void);
void uei_aio_close(void);
void uei_ain_read(void);
void uei_aout_write(f64, f64);
void test_uei_write(void);
void uei_dio_scan(void);
void uei_dio_write_led(u16);
s32 uei_dout_write_masked(s32, u32, u32);
void uei_din_read(s32, u32 *);
void uei_dout01(s32);
void uei_aout32_test(void);
void uei_aout32_write(s32, u16, f64);
void cleanup_devices(void);

// isaft.c
void isa_ft_init(void);
void isa_ft_read(void);

// pc7266.c
/* FVV 20170227 Removed because we don't have this card
void pc7266_init(void);
f64 pc7266_read_ch(u32);
void pc7266_reset_all_ctrs(void);
s32 pc7266_safe_check(void);
void pc7266_encoder_read(void);
void pc7266_calib(void);
*/

// pci4e.c
/* FVV 20170227 Removed because we don't have this card
void pci4e_init(void);
void pci4e_close(void);
void pci4e_reset_all_ctrs(void);
s32 pci4e_safe_check(void);
void pci4e_encoder_read(void);
void pci4e_calib(void);
*/

// sensact.c
void sensact_init(void);
void dio_encoder_sensor(void);
void adc_tach_sensor(void);
void adc_ft_sensor(void);
void fast_read_ft_sensor(void);
void ft_zero_bias(void);
void adc_grasp_sensor(void);
void adc_accel_sensor(void);
void adc_current_sensor(void);

void dac_torque_actuator(void);
void dac_direct_torque_actuator(void);
void set_zero_torque(void);
void write_zero_torque(void);
void vibrate(void);
void do_max(void);
void planar_set_zero_torque(void);
void planar_write_zero_torque(void);
void planar_after_compute_controls(void);

// pl_ulog.c
void init_log_fns(void);
void init_ref_fns(void);

// pl_uslot.c
void init_slot_fns(void);

// fifo.c
void init_fifos(void);
void cleanup_fifos(void);
// TODO: delete s32 fifo_input_handler(u32);
void fifo_input_handler(void);
void print_fifo_input(void);

void set_ob_variable(void);
s32 get_option_x64 (s8 **, u64 *);
s8 *get_options_x64 (s8 *, s32 , u64 *);

void dpr(s32 level, const s8 *format, ...);
void dpr_clear(void);
void dpr_flush(void);

void fprf(RT_PIPE *, const s8 *, ...);
#define prf fprf
#include <stdio.h>
#include <unistd.h>
#include <syslog.h>

#endif				// ROBDECLS_H


