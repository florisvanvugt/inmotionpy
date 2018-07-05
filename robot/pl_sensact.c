// sensact.c - sensors and actuators
// part of the robot.o Linux Kernel Module

// convert from raw data taken from sensor inputs to useful formats
// convert from from useful formats to raw data to be sent to actuators

// InMotion2 robot system software for RTLinux

// Copyright 2003-2005 Interactive Motion Technologies, Inc.
// Cambridge, MA, USA
// http://www.interactive-motion.com
// All rights reserved

#include "rtl_inc.h"
#include "ruser.h"
#include "robdecls.h"

#include "userfn.h"

#define As rob->shoulder.angle
#define Ae rob->elbow.angle
#define Ts rob->shoulder.torque
#define Te rob->elbow.torque
#define Vs rob->shoulder.vel
#define Ve rob->elbow.vel
#define G rob->grasp

// called after imt2.cal is read, to range check some shm vars
void
sensact_init(void) {
    u32 i;

    if (!ob->have_planar) return;
    // din
    As.channel = ibracket(As.channel, 0, 7);
    Ae.channel = ibracket(Ae.channel, 0, 7);

    // ain07
    rob->ain_07 = 0.0;
    rob->ain_07_channel = ibracket(rob->ain_07_channel, 0, 63);
    // adc
    Vs.channel = ibracket(Vs.channel, 0, 63);
    Ve.channel = ibracket(Ve.channel, 0, 63);
    G.channel = ibracket(G.channel, 0, 63);
    for (i=0; i<6; i++) {
	rob->ft.channel[i] = ibracket(rob->ft.channel[i], 0, 63);
    }
}

static void
planar_apply_safety_damping(void)
{
  //f64 dx, dy;
  //f64 xramp, yramp;
  //f64 sramp;

  f64 fX = 0.0;
  f64 fY = 0.0;
  
  if (!ob->have_planar) return;

  ob->safety.active = 1; // Mark that the safety is activated now
  
  //dx = fabs(ob->pos.x - ob->safety.pos);
  //dy = fabs(ob->pos.y - ob->safety.pos);
  
  // set up a ramp at the damping edge
  //xramp = yramp = 1.0;
  // feather the edge of the safety zone.
  // safety.ramp is the width of the feathered edge.
  // don't divide by zero.
  //sramp = ob->safety.ramp;
  //if (sramp > .000001) {
  //if (dx < sramp)
  //xramp = dx / sramp;
  //if (dy < sramp)
  //yramp = dy / sramp;
  //}
  
  fX = -ob->safety.damping_nms * ob->vel.x; // * xramp; // FVV switched off the ramping for now (more complex)
  fY = -ob->safety.damping_nms * ob->vel.y; // * yramp;
  
  // Also keep a record of the safety motor forces we have applied
  ob->safety.motor_force.x = fX;
  ob->safety.motor_force.y = fY;

  // FVV 20180420 I considered applying dynamics compensation here,
  // however it seems to not be such a great idea, because who knows
  // exactly what it will do at the edges of the workspace? It's not designed
  // for that. And when safety damping applies then we just want to get
  // those forces, nothing else. We don't care about isotropy at that point anymore.
  
  //#ifdef dyn_comp 
  //dynamics_compensation(fX,fY,3,1.0);
  //# else
  ob->motor_force.x = fX;
  ob->motor_force.y = fY; // have to make sure these are transformed into torques later!
  //#endif
  
}

void
planar_check_safety_fn(void)
{
  // If no safety motor forces were applied...
  ob->safety.motor_force.x = 0;
  ob->safety.motor_force.y = 0;
  ob->safety.active        = 0;
  
  if (!ob->have_planar) return;
  // this isn't really for overriding safety.
  // it's for turning off the safety damping zone, which
  // is sometimes necessary for debugging.
  if (ob->safety.override) return;

  f64 vel_tot = sqrt( pow(ob->vel.x,2) + pow(ob->vel.y,2) );
  
  if (   ob->pos.x <=  ob->safety.minx
	 || ob->pos.x >=  ob->safety.maxx
	 || ob->pos.y <=  ob->safety.miny
	 || ob->pos.y >=  ob->safety.maxy

	 || vel_tot >= ob->safety.vel
	 //|| ob->vel.x <= -ob->safety.vel
	 //|| ob->vel.x >=  ob->safety.vel
	 //|| ob->vel.y <= -ob->safety.vel
	 //|| ob->vel.y >=  ob->safety.vel
	 
	 // || ob->motor_volts.s <= -ob->safety.volts
	 // || ob->motor_volts.s >= ob->safety.volts
	 // || ob->motor_volts.e <= -ob->safety.volts
	 // || ob->motor_volts.e >= ob->safety.volts
	 ) {
    planar_apply_safety_damping();
    ob->safety.was_planar_damping = 1;
    ob->safety.has_applied += 1;

    // New: disable whatever controller is active -- because otherwise unexpected things may happen
    ob->copy_slot.id = 0;
    ob->copy_slot.fnid = 0; // should be null field
    ob->copy_slot.running = 0;
    ob->copy_slot.go = 1;
    
  } else {
    
    if (ob->safety.was_planar_damping) {
      ob->safety.planar_just_crossed_back = 1;
      ob->safety.was_planar_damping = 0;
    }
    
    
    //return;
    // if the velocity is impossibly fast,
    // it's probably a bad encoder, send no force.
    if (   ob->vel.x <= -5.0
	   || ob->vel.x >= 5.0
	   || ob->vel.y <= -5.0
	   || ob->vel.y >= 5.0
	   ) {
      ob->motor_force.x = 0.0;
      ob->motor_force.y = 0.0;
    }
    
  }
  
}

// when you come out of the damping field, have the voltage ramp up from
// zero to 100% in safety.damp_ret_secs seconds

se
planar_back_from_safety_damping(se volts)
{
    f64 x;
    static u32 total_ticks = 0;

    if (ob->safety.planar_just_crossed_back) {
	if (ob->safety.damp_ret_secs > 10.0 || ob->safety.damp_ret_secs <= 0.0)
	    ob->safety.damp_ret_secs = 2.0;
	ob->safety.planar_just_crossed_back = 0;
	total_ticks = ob->safety.damp_ret_secs * ob->Hz;
	ob->safety.damp_ret_ticks = total_ticks;
    }

    if (ob->safety.damp_ret_ticks <= 0) {
	ob->safety.damp_ret_ticks = 0;
	return volts;
    }

    if (total_ticks <= 0) {
	    total_ticks = 0;
	    ob->safety.damp_ret_ticks = 0;
	    return volts;
    }

    // x must be f64, between zero and one.
    x = (f64)(total_ticks - ob->safety.damp_ret_ticks)
	    / total_ticks;

    volts.s = volts.s * x;
    volts.e = volts.e * x;
    ob->safety.damp_ret_ticks--;

    return volts;
}

// convert the encoder angles from dio, to x/y position.
// Gurley Model VB Interpolating Decoder

// conversion A[se].xform from digital input to radians is 0.00009587.
// 0.00009587 == (pi * 2) / (2^^16)
// A[se].offset are calibration values reflecting the rotational position
// of the encoders in the motor housing.

void
dio_encoder_sensor(void)
{
    se raw;

    se pr;
    xy pos;

    if (!ob->have_planar) return;

    //if (ob->sim.sensors) {
    //ob->pos = ob->sim.pos;
    //return;
    //}

    //if (ob->have_planar_incenc && rob->pci4e.have) {
    // new for planar pci4e
    //raw.s = As.raw = (f64)rob->pci4e.enc[As.channel];
    //raw.e = Ae.raw = (f64)rob->pci4e.enc[Ae.channel];
    //} else {
    // old standard va's on pd2mf
    raw.s = As.raw = (f64)daq->dienc[As.channel];
    raw.e = Ae.raw = (f64)daq->dienc[Ae.channel];
    //}

// take the two raw shoulder/elbow values from the sensors
// and return an x/y coordinate pair
// applying polar to cartesian,
// polar offset xforms,
// and cartesian offset xforms.

    // translate
    // composite
    // cartesian

    // shoulder
    pr.s = As.rad = radian_normalize(xform1d(raw.s, As.xform, As.offset));
    As.deg = As.rad * 180. / M_PI;

    // elbow
    pr.e = Ae.rad = radian_normalize(xform1d(raw.e, Ae.xform, Ae.offset));
    Ae.deg = Ae.rad * 180. / M_PI;

    // these calibrations are negative of the encoder angle readings.
    As.cal = radian_normalize(-xform1d(raw.s, As.xform, 0.0));
    Ae.cal = radian_normalize(-xform1d(raw.e, Ae.xform, 0.0));

    pos = xy_polar_cartesian_2d(pr, rob->link);

    pos.x -= rob->offset.x;
    pos.y -= rob->offset.y;

    ob->pos = pos;
}

void
adc_pot_sensor(void) {
}

// get velocity data, either from a tach on the adc,
// or by calculating it from successive x/y encoder positions.

// if you have a tach,
//
// V[se].offset is the voltage produced by the tachometers at zero velocity.
// This offset may be aquired by a calibration procedure with the tach
// at rest.

// L1 is the shoulder link (upper arm, upper motor).
// L2 is the elbow link (forearm, lower motor).
// the shoulder is above the elbow, like on a person's body (usually).
// note that in many of our hardware docs,
// motor#1 is elbow and motor#2 is shoulder.
// (that is, they are reversed.  oof.)

// theta is the motor's position in radians
// thetadot is the motor's angular velocity in radians/second.

// V = J * thetadot
// [Vx]   [-L1*sin(theta1)  -L2*sin(theta2) ]   [theta1dot]
// [  ] = [                                 ] * [         ]
// [Vy]   [ L1*cos(theta1)   L2*cos(theta2) ]   [theta2dot]     

// thetadot = (theta - prevtheta) * sampfreq

void
adc_tach_sensor(void)
{
    se theta;
    se pr;

    mat22 J;
    xy V;

    f64 Hz;
    f64 dtick;

    if (!ob->have_planar) return;

    theta.s = As.rad;
    theta.e = Ae.rad;
    /* REWT change regarding shoulder angle:
       change made to obtain a theta.s value the varies around 0 
       and does not jump if it reaches 2*pi
    */


    if (theta.s<3.14159265358979) theta.s += 6.28318530717959;
    theta.s -= 6.28318530717959;

    ob->theta = theta;

    // later...
    // base Hz on measured delta_tick instead of constant Hz
    dtick = ob->times.time_delta_tick;
    if (dtick < 1.) dtick = 1.0;
    Hz = (1000.*1000.*1000.) / dtick;

    // if > 5x or < x/5, something is probably wrong.
    if ((Hz > (5 * ob->Hz)) || (Hz < (ob->Hz / 5)))
	Hz = ob->Hz;

    // uncomment this to do it the old constant way.
    Hz = (f64)ob->Hz;

    J = j_polar_cartesian_2d(ob->theta, rob->link);

    {
	f64 dtheta;
	// if no have_tach, use angles and butterworth

	// there is an initial spike, this is filtered out by ignoring
	// the first few cycles in the main loop

	// encoder values are [0..2pi].  To calculate velocity when
	// encoder crosses between 2pi and 0, we need -pi..pi,
	// that's delta_radian_normalize().

	dtheta = (ob->theta.s - prev->theta.s);
	ob->thetadot.s = delta_radian_normalize(dtheta) * Hz;
	pr.s = ob->fthetadot.s = butter(ob->thetadot.s, prev->thetadot.s, prev->fthetadot.s);

	dtheta = (ob->theta.e - prev->theta.e);
	ob->thetadot.e = delta_radian_normalize(dtheta) * Hz;
	pr.e = ob->fthetadot.e = butter(ob->thetadot.e, prev->thetadot.e, prev->fthetadot.e);

	// return this if no tach.
	ob->fsoft_vel = jacob2d_x_p2d(J, pr);
	V = ob->fsoft_vel;
    }

    {
	// or, if no have_tach, use x/y
	// no filter here

	ob->soft_vel.x = (ob->pos.x - prev->pos.x) * Hz;
	ob->soft_vel.y = (ob->pos.y - prev->pos.y) * Hz;
    }

    if (ob->have_tach) {
	// have_tach
	// 10 volts = 32k

	// convert raw tach voltages to radial velocity
	Vs.raw = daq->adcvolts[Vs.channel];
        // these calibrations are negative of the raw voltage readings
	Vs.cal = -Vs.raw;
	// pr.[se] used to look like this, which is silly.
	// pr.s = Vs.rad = xform1d(Vs.raw, Vs.xform / 1.8,
			// Vs.offset / 1.8);
	if (fabs(Vs.xform) < .0001) Vs.xform = 1.8;
	pr.s = Vs.rad = (Vs.raw + Vs.offset) / Vs.xform;

	Ve.raw = daq->adcvolts[Ve.channel];
	Ve.cal = -Ve.raw;
	if (fabs(Ve.xform) < .0001) Ve.xform = -1.8;
	pr.e = Ve.rad = (Ve.raw + Ve.offset) / Ve.xform;

	dyncmp_var->anglevelocity = pr;                   // REWT: q1dot and q2dot

	// convert to x/y velocity
	V = ob->tach_vel = jacob2d_x_p2d(J, pr);
	// butterworth filter the tach velocity
	ob->ftach_vel.x = butter(ob->tach_vel.x, prev->tach_vel.x, prev->ftach_vel.x);
	ob->ftach_vel.y = butter(ob->tach_vel.y, prev->tach_vel.y, prev->ftach_vel.y);
	// if you want unfiltered tach vels, comment this line out.
	V = ob->ftach_vel;
    }

    // if we have a tach, ob->vel is from that,
    // else from encoder angles.
    ob->vel = V;

    //if (ob->sim.sensors) {
    //ob->vel = ob->sim.vel;
    //}

    ob->velmag = hypot(ob->vel.x, ob->vel.y);

    if (ob->velmag > ob->safety.velmag_kick) {
	do_error(ERR_PL_ENC_KICK);
	ob->vel = prev->vel;
    }

    ob->soft_accel.x = (ob->vel.x - prev->vel.x) * Hz;
    ob->soft_accel.y = (ob->vel.y - prev->vel.y) * Hz;
    ob->soft_accelmag = hypot(ob->soft_accel.x, ob->soft_accel.y);
}

// add vibration for testing.
// currently closed-loop only.
// note: if you experiment with this, be careful!
// <=1000 is treated specially, see second block.
// >1000 gives a light buzz, >3000 starts getting shaky.
// 20000 == 20 N deflection.
// 13 and 17 are numbers of samples, and they give us a vibration
// rate of 200/13 Hz in X and 200/17 Hz in Y, around 15 Hz.
// my milkshake brings all the bots to the yard.

void
vibrate(void) {
    u32 vibrate;
    static s32 dx, dy;

    // ob->vibrate may change while the following block is running.
    vibrate = ob->vibrate;

    if (vibrate > 0 && vibrate <= 20000) {

	// shake with a smooth triangular sawtooth.
	if (vibrate > 1000) {
	    dx = 6 - (ob->i % 13); // -6 thru 6 sawtooth ///////
	    if ((ob->i % 26) >= 13) dx = -dx;   //       /\/\/\/
	    dy = 8 - (ob->i % 17); // -8 thru 8 sawtooth
	    if ((ob->i % 34) >= 17) dy = -dy;
	    dx = (vibrate / 1000.0) * (dx / 6.0);
	    dy = (vibrate / 1000.0) * (dy / 8.0);
	}
	// below 1000 by 100's, shake at random,
	// bigger num is lower time freq.
	else if (vibrate >= 100 && vibrate <= 1000) {
	    s32 mod;

	    mod = (10 * ob->Hz) / vibrate;

	    if ((ob->i % mod) == 0) {
		s32 rand(void);
		dx = 15 - (rand() % 31);
		dy = 15 - (rand() % 31);
	    }
	}
    } else {
	dx = dy = 0;
    }
    ob->xvibe = dx;
    ob->yvibe = dy;
}

// All the analog signals pass through a low-pass passive
// RC filters to eliminate spurious high frequency noise components
// (30 Hz breaking point).
//
// Torque actuators:  The maximum continuous torque we can command
// is +/- 10 Nm (limited by the servo amplifier circuitry).
// Allows nominal torque only

// [Ttheta1]   [-L1*sin(theta1)  L1*cos(theta1)]   [Fx]
// [       ] = [                               ] * [  ]
// [Ttheta2]   [-L2*sin(theta2)  L2*cos(theta2)]   [Fy]

void
dac_torque_actuator(void)
{
    se pr;
    se torque;
    se volts;

    mat22 Jp, Jt;

    if (!ob->have_planar) return;
    // encoder angles
    pr.s = As.rad;
    pr.e = Ae.rad;

    Jp = j_polar_cartesian_2d(pr, rob->link);
    Jt = jacob2d_transpose(Jp);

    if (ob->vibrate) {
	ob->motor_force.x += ob->xvibe;
	ob->motor_force.y += ob->yvibe;
    }

    // torque in Nm
    torque.s = ob->motor_force.x * Jt.e00
	+ ob->motor_force.y * Jt.e01;
    // torque volts
    volts.s = (torque.s - Ts.offset) / Ts.xform;

    torque.e = ob->motor_force.x * Jt.e10
	+ ob->motor_force.y * Jt.e11;
    volts.e = (torque.e - Te.offset) / Te.xform;

    // voltage override, for testing and calibrating motors.
    // this lets us send a constant voltage to each motor
    // in an open loop mode.
    if (ob->test_raw_torque) {
	// func.write_log = write_motor_test_fifo_sample_fn;
	// ob->logfnid = 1 is set in motor_test script now.
	volts.s = ob->raw_torque_volts.s;
	volts.e = ob->raw_torque_volts.e;
	torque.s = volts.s * Ts.xform + Ts.offset;
	torque.e = volts.e * Te.xform + Te.offset;
    }

    // FVV 20180420 commented out wondering if it causes too slow coming on of force.
    //volts = planar_back_from_safety_damping(volts);

    // bracket voltages, preserving force orientation

    volts = preserve_orientation(volts, ob->pfomax);

    // if you're testing, you can set this to something gentle

    volts = preserve_orientation(volts, ob->pfotest);

    // TODO impose thermal model

    ob->motor_torque.s = torque.s;
    ob->motor_torque.e = torque.e;
    ob->motor_volts.s = volts.s;
    ob->motor_volts.e = volts.e;

    if (ob->have_planar_ao8) {
	// new for planar ao8
	uei_aout32_write(0, 0, ob->motor_volts.s);
	uei_aout32_write(0, 1, ob->motor_volts.e);
    } else {
	// old for planar mf
	uei_aout_write(ob->motor_volts.s, ob->motor_volts.e);
    }
}

void
dac_direct_torque_actuator(void)
{
  // Writes ob->motor_torque to the actuators.
  // This no longer uses ob->motor_force because it assumes
  // that has already been transformed into torques.
  
  se torque;
  se volts;
  
  torque.s = ob->motor_torque.s;
  torque.e = ob->motor_torque.e;
  
  volts.s = (torque.s - Ts.offset) / Ts.xform;
  volts.e = (torque.e - Te.offset) / Te.xform;
  
  volts = preserve_orientation(volts, ob->pfomax);
  
  // if you're testing, you can set this to something gentle
  
  volts = preserve_orientation(volts, ob->pfotest);
  
  ob->motor_volts.s = volts.s;
  ob->motor_volts.e = volts.e;
  
  uei_aout_write(ob->motor_volts.s, ob->motor_volts.e);
}


void
planar_set_zero_torque(void)
{
  if (!ob->have_planar) return;
  ob->motor_force.x = 0.0;
  ob->motor_force.y = 0.0;
  ob->motor_torque.s = 0.0;
  ob->motor_torque.e = 0.0;
  ob->motor_volts.s = 0.0;
  ob->motor_volts.e = 0.0;
}

void
planar_write_zero_torque(void)
{
  if (!ob->have_planar) return;
  planar_set_zero_torque();
  if (ob->have_planar_ao8) {
    // new for planar ao8
    uei_aout32_write(0, 0, 0.0);
    uei_aout32_write(0, 1, 0.0);
  } else {
    // old for planar mf
    uei_aout_write(0.0, 0.0);
  }
}

void
planar_after_compute_controls(void)
{
    if (!ob->have_planar) return;
    prev->pos = ob->pos;
    prev->vel = ob->vel;
    prev->theta = ob->theta;
    prev->thetadot = ob->thetadot;
    prev->fthetadot = ob->fthetadot;
    prev->tach_vel = ob->tach_vel;
    prev->ftach_vel = ob->ftach_vel;

    do_max();
}

void do_max(void)
{
	ob->max.vel.x = MAX(ob->max.vel.x, ob->vel.x);
	ob->max.vel.y = MAX(ob->max.vel.y, ob->vel.y);
	ob->max.motor_force.x = MAX(ob->max.motor_force.x, ob->motor_force.x);
	ob->max.motor_force.y = MAX(ob->max.motor_force.y, ob->motor_force.y);
	ob->max.motor_torque.s = MAX(ob->max.motor_torque.s, ob->motor_torque.s);
	ob->max.motor_torque.e = MAX(ob->max.motor_torque.e, ob->motor_torque.e);
}

// force transducer

/*  Inmotion2 is connected to the computer through two analog and one digital
    DIO functions contained in a single UEI - PCI board plus an ATI-ISA for
    forces and moments (alternative: the new ATI FT does not include an ISA board).

                 Elbow     Shoulder
     Position    (0)       (1)
                 16 lines  16 lines


	         Elbow	   Shoulder	          Elbow   Shoulder
     Velocity    (6)       (7) 		Torque    (0)	  (1)

     Force  	 Fx, Fy, Fz
     Moment      Mx, My, Mz

    The A/D input channels are configured in a full differential configuration
    of 16-bits, the D/A output channels are configured for 12-bits, the
    DIO channels are configured to multiplex 16 din and 16 dout lines, and the
    force ISA card is configured for 16-bits.

*/

// conversions from google
#define conv_lb_N 4.44822162
#define conv_V_N 26.68932972
#define conv_inlb_Nm 0.112984829

// by default, the ISA FT returns counts multiplied by a scale factor
// returned by get_counts_info().  This value is scaled by 10, if it
// returns 400, that means it's scaled by 40.  The 16.0 comes from the
// fact that it is returning a 12-bit quantity as a 16-bit quantity,
// and we need to account for that too.

#define ISA_COUNTS_SCALE (16.0 / 10.0)

//  ATI ISA Force Transducer
//  Force sensor: the first parameter is the rotation angle in
//  radians fo the sensor's X axis relative to the line connecting the
//  elbow to the handle: the second and third are the shoulder and the
//  elbow link lengths, respectively (see LinkS and LinkE below); Note
//  that we can't simply initialize the LinkS and LinkE because C++ does
//  not guarantee initialization order.
//
//  F( 'rot. angle in rad', 'LinkS', 'LinkE' )
//
//  Note that we designed Inmotion2 to guarantee alignment of X+ with
//  the robot arm. Therefore rot. angle in rad is always 0.0
//

/////
//  F(2.35619449, 0.4064, 0.51435),


// Link Length 1 = 16.0 in = 0.4064 m
//             2 =  6.0 in = 0.15240 m
//             3 = 16.0 in = 0.4064 m
//             4 = 20.25 in = 0.51435 m

// LinkS( 0.4064 ), LinkE( 0.51435 ),
// offsetx( 0.0 ), offsety( -0.65 ), // y is the distance between the
// actuator's and the world's coordinate centers

void
adc_ft_sensor(void)
{
    // matrix multiply amd convert
    // rob->ft.raw strain gauge voltages to forces

    u32 i;
    xy Fxy;
    xyz dev;

    if (!ob->have_ft)
	return;

    // ISA FT data starts at iraw[2]
    // get raw int iraw data from the ISA FT
    // scale it to float English quantities, put it in cooked.
    // Bias it, convert it to metric,
    // put it in rob>ft->filt, since the ISA device filters it.

    if (ob->have_isaft) {
	for (i = 0; i < 6; i++) {
	    f64 scale, conv;

	    if (i < 3) {
		// forces
		scale = (f64)rob->isaft.cpf * ISA_COUNTS_SCALE;
		conv = conv_lb_N;
	    } else {
		// moments
		scale = (f64)rob->isaft.cpt * ISA_COUNTS_SCALE;
		conv = conv_inlb_Nm;
	    }
	    if (scale == 0.0) scale = 1.0;
	    //
	    // scale 12bit int to 64bit float English
	    rob->isaft.raw[i] =
		(f64)rob->isaft.iraw[i+2] / scale;

	    // ob->scr[i] = (f64)rob->isaft.iraw[i+2];
	    // ob->scr[8+i] = rob->isaft.raw[i];

	    rob->ft.raw[i] = rob->isaft.raw[i];
	    // apply bias
	    // apply conversion from English to metric
	    rob->ft.filt[i] =
		    (rob->ft.raw[i] + rob->ft.bias[i]) * conv;
	}
    } else {	// have newer ATI FT that talks to DAQ board

	// curr has intermediate values, so don't use ft.curr
	f64 curr[6];

	for (i = 0; i < 6; i++) {
	    rob->ft.raw[i] = daq->adcvolts[rob->ft.channel[i]];
	    rob->ft.cooked[i] = rob->ft.raw[i] + rob->ft.bias[i];
	}
	
	rob->ain_07 = daq->adcvolts[rob->ain_07_channel];

	// rob->ain_07 = daq->adcvolts[Vs.channel];
/* 	for (i = 0; i < 6; i++) { */
/* 	    u32 j; */

/* 	    curr[i] = 0; */
/* 	    for (j = 0; j < 6; j++) { */
/* 		curr[i] += (rob->ft.cal[i][j] * rob->ft.cooked[j]); */
/* 	    } */
/* 	} */

	for (i = 0; i < 6; i++) {
	    f64 conv;
	    // u32 j;
	    // u32 mod;

	    if (i < 3)
	      //conv = conv_lb_N;
	        conv = conv_V_N;
	    else
		conv = conv_inlb_Nm;

	    curr[i] = rob->ft.cooked[i]; // ggh add
	    rob->ft.curr[i] = conv * curr[i];
	    //	    if (rob->ft.scale[i] == 0.0) rob->ft.scale[i] = 1.0;
	    // rob->ft.curr[i] = conv * curr[i] / rob->ft.scale[i];


	    // butterworth filtered
	    rob->ft.but[i] = butter(rob->ft.curr[i], rob->ft.prev[i], rob->ft.prevf[i]);
	    rob->ft.prev[i] = rob->ft.curr[i];
	    rob->ft.prevf[i] = rob->ft.but[i];

	    // sav gol filtered avg
	    rob->ft.sg[i] = apply_filter(rob->ft.curr[i], rob->ft.sghist[i]);

#ifdef LATER
	    // butterworth stopband
	    rob->ft.bsrawhist[i][0] = rob->ft.curr[i];
	    rob->ft.bs[i] = butstop(rob->ft.bsrawhist[i], rob->ft.bsfilthist[i]);
#endif // LATER
	    rob->ft.filt[i] = rob->ft.but[i];
	    // rob->ft.filt[i] = rob->ft.bs[i];
	    // rob->ft.filt[i] = rob->ft.sg[i];
	}
    }

    // for horizontally mounted FTs
    // with the link pointing straight back at the patient,
    // +x radiates from the arm.
    // +y is right
    // +z is up

    dev.x = rob->ft.filt[0];
    dev.y = rob->ft.filt[1];
    dev.z = rob->ft.filt[2];

    // this handles the FT mounted vertically at the end of the planar arm.
    // with the jr3 mounted vertically,
    // +x is right
    // +y is down
    // +z points from the arm

    if (rob->ft.vert) {
	dev.x = rob->ft.filt[2];
	dev.y = rob->ft.filt[0];
	dev.z = -rob->ft.filt[1];
    }

    // account for ft up/down orientation
    // (the flip code should be cleaner.)
    if (rob->ft.flip) {
	mat22 m;
	se xy;	// this is really an xy.
	f64 off;
	
	off=rob->ft.offset;

	xy.s = dev.x;
	xy.e = dev.y;

	m.e00 = cos(off);	m.e01 = sin(off);
	m.e10 = sin(off);	m.e11 = -cos(off);
	Fxy = jacob2d_x_p2d(m, xy);

	dev.x = Fxy.x;
	dev.y = Fxy.y;
	dev.z = -dev.z;
    }

    // now we have filtered FT data, from either an ISA or DAQ ATI FT.

    // transform from FT coordinates rob->ft.curr[0,1]
    // to F.world

    {
	f64 del0;
	f64 cosD, sinD;
	f64 cosA, sinA;
	f64 cosB, sinB;
	f64 Ls, Le;

	se torque_aux;

	se angles;
	mat22 Jt;		// dev, world space

	del0 = Ae.rad - As.rad;
	cosD = cos(del0), sinD = sin(del0);
	if (rob->ft.flip) {
	    // we already factored the offsets into
	    // the flipped angles before.  this is kludgy code,
	    // it should be better integrated.
	    cosA = cos(0.0);
	    sinA = sin(0.0);
	} else {
	    cosA = cos(rob->ft.offset);
	    sinA = sin(rob->ft.offset);
	}
	Ls = rob->link.s;
	Le = rob->link.e;
	cosB = cosA * cosD - sinA * sinD;
	sinB = sinA * cosD + cosA * sinD;

	torque_aux.s = (dev.x * sinB + dev.y * cosB) * Ls;
	torque_aux.e = (dev.x * sinA + dev.y * cosA) * Le;

	// dev space
	angles.s = As.rad;
	angles.e = Ae.rad;
	rob->ft.dev.x = rob->ft.filt[0];
	rob->ft.dev.y = rob->ft.filt[1];
	rob->ft.dev.z = rob->ft.filt[2];
	rob->ft.xymag = hypot(dev.x, dev.y);
	rob->ft.moment.x = rob->ft.filt[3];
	rob->ft.moment.y = rob->ft.filt[4];
	rob->ft.moment.z = rob->ft.filt[5];

	// world space
	Jt = j_polar_cartesian_2d(angles, rob->link);
	Jt = jacob2d_transpose(Jt);
	Jt = jacob2d_inverse(Jt);
	Fxy = jacob2d_x_p2d(Jt, torque_aux);

	rob->ft.world.x = Fxy.x;
	rob->ft.world.y = Fxy.y;

	if (rob->ft.vert) {
	    rob->ft.world.z = -rob->ft.filt[1];
	    return;
	}

	if (rob->ft.flip) {
	    rob->ft.world.z = -rob->ft.filt[2];
	} else {
	    rob->ft.world.z = rob->ft.filt[2];
	}
    }

    if (ob->have_linear) {
	rob->ft.world.x = rob->ft.dev.y;
	rob->ft.world.y = -rob->ft.dev.x;
	rob->ft.world.z = -rob->ft.dev.z;
    }
}

// called when "oversampling" the ft

void
fast_read_ft_sensor(void)
{
	uei_ain_read();
	adc_ft_sensor();
}

// set the bias array to the current ft voltages,
// "zeroing" the bias.
// gets called when rob->ft.dobias is set.

void
ft_zero_bias()
{
	s32 i;
	for (i = 0; i < 6; i++) {
	  rob->ft.bias[i] = -rob->ft.raw[i];//daq->adcvolts[rob->ft.channel[i]]
	}
}


// Dustin's grasp sensor, both foam pad and strain gauge.
//

void
adc_grasp_sensor(void)
{
    if (!ob->have_grasp)
	    return;

    G.raw = daq->adcvolts[G.channel];
    G.cal = G.raw * G.gain;
    // for foam grasp sensor
    // G.force = (G.raw * G.gain - G.bias) - rob->ft.xymag;
    // for strain gauge grasp sensor
    G.force = (G.raw * G.gain + G.bias);
}

// Analog Devices ADXL 105EM-3 3-axis accelerometer
// http://www.analog.com/en/prod/0,2877,ADXL105,00.html
//
// takes 5V as input
// returns between 0-5V for each of X,Y,Z.
// output V = .5*G + 2.5
//
// rough calibration: -1G = 2V and 1G = 3V.
// so a typical 0G bias is 2.5 V and xform is .5 V/G.
// device output voltage range: +/- 4G (.5V - 4.5V)

void
adc_accel_sensor(void)
{
    u32 i;

    if (!ob->have_accel)
	    return;

    for (i=0; i<3; i++) {

	rob->accel.raw[i] = daq->adcvolts[rob->accel.channel[i]];
	rob->accel.curr[i] = (rob->accel.raw[i] - rob->accel.bias[i])
		/ rob->accel.xform;

	rob->accel.filt[i] = butter(rob->accel.curr[i],
		rob->accel.prev[i], rob->accel.filt[i]);
	rob->accel.prev[i] = rob->accel.curr[i];
	rob->accel.prevf[i] = rob->accel.filt[i];
    }
}

// note that we changed all the logging/sensing from 11 to 15
// touches logfiles, anref, loopfix, an_ulog/ref.

void
adc_current_sensor(void)
{
    f64 a, c;
    u32 i;

    if (!rob->csen.have)
	return;
    if (rob->csen.have > 4)
	rob->csen.have = 0;
    // each axis is a pair of currents
    for (i=0; i<(rob->csen.have); i++) {
	rob->csen.rawa[i] = daq->adcvolts[rob->csen.channela[i]];
	rob->csen.rawc[i] = daq->adcvolts[rob->csen.channelc[i]];
	a = rob->csen.rawa[i] - rob->csen.kt_biasa[i];
	c = rob->csen.rawc[i] - rob->csen.kt_biasc[i];
	if (fabs(rob->csen.xforma[i]) < .000001)
	    rob->csen.xforma[i] = 2.35;
	a = a / rob->csen.xforma[i];
	if (fabs(rob->csen.xformc[i]) < .000001)
	    rob->csen.xformc[i] = 2.35;
	c = c / rob->csen.xformc[i];

	// if (fabs(a) < .000001) a = .000001;
	rob->csen.alpha[i] = atan2(((a * cos(4./3.*M_PI)) - c),
	    (a * sin(4./3. * M_PI)));

	// rob->csen.alpha[i] = atan2((a + 2 * c) / -sqrt(3.0), a);

	if (fabs(cos(rob->csen.alpha[i]) < .1)) {
		rob->csen.curr[i] = c / (cos(rob->csen.alpha[i] + (4./3.*M_PI)));
	} else {
		rob->csen.curr[i] = a / cos(rob->csen.alpha[i]);
	}
	// ob->scr[0] = -rob->csen.curr[i] * rob->csen.kt[i];
	rob->csen.torque[i] = (-rob->csen.curr[i]) * rob->csen.kt[i];

	// these torques are magnitudes.  signs are determined in robot_sensact.
    }
}

// gnuplot
// plot '/tmp/at7.asc'  u 1:14 w l, '' u 1:($15 * .027) w l
