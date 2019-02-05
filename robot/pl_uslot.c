// uslot.c -switchuser slot functions, to be modified by InMotion2 programmers
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

// ************************************************************************
// PLG March 7, 2013
// define some minjerk functions for position and velocity
// Equations from:
//        Flash, Tamar, and Neville Hogan. "The coordination of arm
//        movements: an experimentally confirmed mathematical model." The
//        journal of Neuroscience 5, no. 7 (1985): 1688-1703.
//
f64 minjerk_pos(f64 p1, f64 p2, f64 ttot, f64 tcur)
{
  f64 tau = tcur / ttot;
  return p1 + (p1-p2)*(15*(tau*tau*tau*tau)
    - (6*tau*tau*tau*tau*tau)
    - (10*tau*tau*tau));
}
//
f64 minjerk_vel(f64 p1, f64 p2, f64 ttot, f64 tcur)
{
  return (p1-p2)*(-30*(tcur*tcur*tcur*tcur)/(ttot*ttot*ttot*ttot*ttot)
    + 60*(tcur*tcur*tcur)/(ttot*ttot*ttot*ttot)
    - 30*(tcur*tcur)/(ttot*ttot*ttot));
}
// ************************************************************************

//void simple_ctl(u32);
void damp_ctl(u32);
void planar_const_ctl(u32);

void null_ctl_dyncomp(u32);
void null_ctl(u32);
void move_phase_ctl(u32);
void null_ic_ctl(u32);
void zero_ft(u32);
void zero_ft_nodyncomp(u32);
void movetopt(u32);
void static_ctl(u32);
void static_ctl_fade(u32);
void curl_ctl(u32);
void viscous_force_field(u32);
void force_channel(u32);

void trajectory_capture(u32);
void trajectory_reproduce(u32);



void
init_slot_fns(void)
{
  //ob->slot_fns[0] = null_ctl_dyncomp;
  ob->slot_fns[0] =  null_ctl;
  ob->slot_fns[1] =  null_ctl_dyncomp;
  ob->slot_fns[2] =  zero_ft_nodyncomp;
  ob->slot_fns[3] =  damp_ctl;
  ob->slot_fns[4] =  movetopt;          // move the subject to px,py in time t
  ob->slot_fns[5] =  static_ctl_fade;   // hold subject at a point and then gradually fade out
  ob->slot_fns[6] =  move_phase_ctl;    // used during the experiment for free moving (and updating whether the subject has left the center)
  ob->slot_fns[7] =  null_ctl;
  ob->slot_fns[8] =  trajectory_capture;
  ob->slot_fns[9] =  trajectory_reproduce;
  ob->slot_fns[10] = null_ctl;
  ob->slot_fns[11] = null_ctl;
  ob->slot_fns[12] = null_ctl;
  ob->slot_fns[13] = viscous_force_field;
  ob->slot_fns[14] = movetopt;
  ob->slot_fns[15] = force_channel;   // this is to handle force-channel function! [Ananda, 20sep]  
  ob->slot_fns[16] = static_ctl;   // stay at p1x p1y under active control
  ob->slot_fns[17] = curl_ctl; 
}

#define X ob->pos.x
#define Y ob->pos.y
#define cnt moh->counter
#define dir moh->current_dir
#define refX moh->last_pointX
#define refY moh->last_pointY
#define dyn_comp 1
#define X ob->pos.x
#define Y ob->pos.y
#define fX ob->motor_force.x
#define fY ob->motor_force.y
#define vX ob->vel.x
#define vY ob->vel.y
#define sX rob->ft.world.x
#define sY rob->ft.world.y
#define sZ rob->ft.world.z

#define fvv_vmax_x         ob->fvv_vmax_x
#define fvv_vmax_y         ob->fvv_vmax_y

#define fvv_final_x         ob->fvv_final_x
#define fvv_final_y         ob->fvv_final_y

#define fvv_trial_phase    ob->fvv_trial_phase
#define fvv_robot_center_x ob->fvv_robot_center_x
#define fvv_robot_center_y ob->fvv_robot_center_y
//#define fvv_robot_min_y    ob->fvv_robot_min_y
#define fvv_vel            ob->fvv_vel
#define fvv_max_vel        ob->fvv_max_vel
#define fvv_trial_timer    ob->fvv_trial_timer
#define fvv_vel_low_timer  ob->fvv_vel_low_timer

#define traj_count         ob->traj_count
#define traj_n_samps       ob->traj_n_samps
#define traj_final_x       ob->traj_final_x
#define traj_final_y       ob->traj_final_y
#define trajx              ob->trajx
#define trajy              ob->trajy
#define replay_damping     ob->replay_damping
#define replay_stiffness   ob->replay_stiffness




const double distance_cutoff=.05; // in meters; the distance from the center that counts as "started moving"

// Controls the decision about the trial end. 
// If velocity falls below vy_prop*(maximum velocity of that trial)
// for a duration of vy_low_duration, then we declare it the end of the
// trial, which means we initiate the clamp to stop the participant where they are.
// proportion of the maximum velocity
const double vel_prop=.05;       // proportion of maximum velocity
const int vel_low_duration = 15; // in # of frames

double max_vel_prop; // used to cache the calculation of vel_prop*fvv_max_vel

//const int max_trial_duration = // in # of frames



void
damp_ctl(u32 id)
{
  
  f64 damp;
  
  damp = ob->damp;
  
  fX = -( damp * (vX) );
  fY = -( damp * (vY) );
}

void
raw_volts_ctl(u32 id)
{
  
  f64 damp;
  
  damp = ob->damp;
  
  fX = -( ob->stiff * (X - ob->ref.pos.x)
	  + damp * (vX - ob->ref.vel.x) );
  
  fY = -( ob->stiff * (Y - ob->ref.pos.y)
	  + damp * (vY - ob->ref.vel.y) );
}

void
point_ctl(u32 id)
{
  
  f64 damp;
  
  damp = ob->damp;
  
  fX = -( ob->stiff * (X - ob->ref.pos.x)
	  + damp * (vX - ob->ref.vel.x) );
  
  fY = -( ob->stiff * (Y - ob->ref.pos.y)
	  + damp * (vY - ob->ref.vel.y) );
}


// derived from rotate_ctl

// NOTE!  this controller does not follow the "morphing rectangle" paradigm
// of the other controllers.  it takes a starting x/y point and
// a finishing x/y point, and assumes that there is a collapsing slot
// between them.  this is strange and sad, but currently true.



//static f64
//pl_i_ref_fn(u32 i, u32 term, f64 phase, f64 amplitude) {
//	return (amplitude * cos((2.0 * M_PI * i / term) + phase));
//}

// constant force contrller

void
planar_const_ctl(u32 id)
{
    fX = ob->const_force.x;
    fY = ob->const_force.y;
}


void
null_ctl(u32 id)
{
  // Null controller with no dynamics compensation
  fX = 0.0;
  fY = 0.0;
  //#ifdef dyn_comp 
  //  dynamics_compensation(fX,fY,3,1.0);
  //# else
  ob->motor_force.x = fX;
  ob->motor_force.y = fY;
  ob->motor_torque.s = 0.0;
  ob->motor_torque.e = 0.0;
  ob->motor_volts.s = 0.0;
  ob->motor_volts.e = 0.0;
  //#endif
}




void
null_ctl_dyncomp(u32 id)
{
  //fX = ob->plg_last_fX * 0.0;
  //fY = ob->plg_last_fY * 0.0;
  fX = 0.0;
  fY = 0.0;
  #ifdef dyn_comp 
    dynamics_compensation(fX,fY,3,1.0);
  # else
  ob->motor_force.x = fX;
  ob->motor_force.y = fY;
  #endif
}






void
move_phase_ctl(u32 id)
{
  /* 
     This is a controller that is basically a null field,
     except for when the velocity falls below a certain 
     amount and then it sets fvv_trial_phase to 6.
  */


  if (! (ob->fvv_move_done)) {
    //if (fvv_trial_phase==5) {
    // PHASE = moving

    fvv_vel = sqrt(pow(vX,2)+pow(vY,2));

    // If we're in the free moving phase, keep track
    // of the maximum speed. If we're currently faster than
    // the maximum speed, update it.
    if (fvv_vel>fvv_max_vel) {
      fvv_max_vel = fvv_vel;
      max_vel_prop = vel_prop*fvv_max_vel; // calculate the proportion of the velocity below which we'll stop the trial
      fvv_vmax_x      = X; 
      fvv_vmax_y      = Y;
      
    }

    // Furthermore, if we are below a particular percentage of the maximum velocity, count time
    if (fvv_vel<max_vel_prop) {
      fvv_vel_low_timer++; // tick
    } else {
      // If we're not below the threshold, reset the counter.
      fvv_vel_low_timer = 0; // reset
    }
    
    if (fvv_vel_low_timer>vel_low_duration) {
      // If we have been below the threshold-low speed long enough,
      // declare it the end of the trial.
      //fvv_trial_phase = 6; // DEPRECATED this marks that the trial has come to an end!
      ob->fvv_move_done = 1;  // a better way to mark that the movement is completed
      //plg_move_done = 1;
      fvv_final_x = X;
      fvv_final_y = Y;
    }

    fvv_trial_timer++;

  }

  // No forces; just free movement
  fX = 0.0;
  fY = 0.0;

  
  // Infrastructure for capturing (if we are doing that -- but why not?)
  do_capture();
  
  // And dynamics compensation please
#ifdef dyn_comp 
  dynamics_compensation(fX,fY,3,1.0);
# else
  ob->motor_force.x = fX;
  ob->motor_force.y = fY;
#endif
}





void
do_capture()
/* Capture the current position and add it to the capture array. */
{
  if (ob->fvv_capture) {
    int traj_cnt = traj_count;// cast to unsigned just to be sure
    if (traj_count<=TRAJECTORY_BUFFER_SIZE) {
      
      // Capture the current position
      trajx[traj_cnt] = X;
      trajy[traj_cnt] = Y;
      
      ++traj_count;
    }
  }
}





void
zero_ft_nodyncomp(u32 id)
{
  int i;
  for (i=0; i<6; i++) {
    ob->plg_ftzero[i] += -rob->ft.raw[i];
  }
  ob->plg_ftzerocount++;
  fX = 0.0;
  fY = 0.0;
  
  //#ifdef dyn_comp 
  //dynamics_compensation(fX,fY,3,1.0);
  //# else
    ob->motor_force.x = fX;
    ob->motor_force.y = fY;
  ob->motor_torque.s = 0.0;
  ob->motor_torque.e = 0.0;
  ob->motor_volts.s = 0.0;
  ob->motor_volts.e = 0.0;
    //#endif
  
}




// moves the subject passively
// from plg_p1x, plg_p1y to plg_p2x, plg_p2y
// in time plg_movetime
//
void movetopt(u32 id)
{
  f64 wx, wy, v;
  f64 pdesx, pdesy ;
  f64 pdesxd, pdesyd;
  f64 stiff, damp;
  f64 timenow = (f64)(ob->i - ob->plg_counterstart) / (f64)ob->Hz;

  ob->plg_moveto_done = 0;
  if (timenow<0.0 || timenow>ob->plg_movetime) {
    //    fX = 0.0;
    //    fY = 0.0;
    fX = ob->plg_last_fX * 0.90; // gradually reduce the force towards zero
    fY = ob->plg_last_fY * 0.90;
    ob->plg_moveto_done = 1;
  }
  else {
    // min-jerk path from p1 to p2
    pdesx = minjerk_pos(ob->plg_p1x, ob->plg_p2x, ob->plg_movetime, timenow);
    pdesy = minjerk_pos(ob->plg_p1y, ob->plg_p2y, ob->plg_movetime, timenow);
    pdesxd = minjerk_vel(ob->plg_p1x, ob->plg_p2x, ob->plg_movetime, timenow);
    pdesyd = minjerk_vel(ob->plg_p1y, ob->plg_p2y, ob->plg_movetime, timenow);
    stiff = ob->plg_stiffness;
    damp = ob->plg_damping;
    fX = (-stiff*(X-pdesx) - damp*(vX-pdesxd));
    fY = (-stiff*(Y-pdesy) - damp*(vY-pdesyd));
    //mkt: accumulate partitioned FT world forces over movement
    switch (ob->ba_accumforce){
    case 0: //inactive (do nothing)
      break;
    case 1: //zero accumulators
      ob->mkt_finline = ob->mkt_finline = 0.;
      ob->plg_ftzerocount = 0;
      ob->ba_accumforce = 2;
      break;
    case 2: //accumulate
      wx = rob->ft.world.x;
      wy = rob->ft.world.y;
      v = cos(ob->mkt_mvtangle)*wx + sin(ob->mkt_finline)*wy;
      ob->mkt_finline += v*v;
      v = sin(ob->mkt_mvtangle)*wx + cos(ob->mkt_finline)*wy;
      ob->mkt_fortho += v*v;
      ob->plg_ftzerocount++;
      break;
    }
  }
  
  ob->plg_last_fX = fX;
  ob->plg_last_fY = fY;
#ifdef dyn_comp 
  dynamics_compensation(fX,fY,3,1.0);
# else
  ob->motor_force.x = fX;
  ob->motor_force.y = fY;
#endif
}


void 
static_ctl(u32 id)
{
  /*
    Static controller that keeps the robot at a position
    (ob->plg_p1x, ob->plg_p1y) with stiffness ob->plg_stiffness
    and damping ob->plg_damping.
   */
  f64 pcurx = ob->plg_p1x;
  f64 pcury = ob->plg_p1y;
  f64 stiff = ob->plg_stiffness;
  f64 damp  = ob->plg_damping;
  fX= (-stiff*(X-pcurx) - damp*(vX));
  fY= (-stiff*(Y-pcury) - damp*(vY));
#ifdef dyn_comp 
  dynamics_compensation(fX,fY,3,1.0);
# else
  ob->motor_force.x = fX;
  ob->motor_force.y = fY;
#endif
}



void 
static_ctl_fade(u32 id)
{
  /* This is the same as the static_ctl controller,
     which keeps us at the given coordinates,
     except that this fades the force gently,
     using the multiplier fvv_force_fade that goes to
     zero. */
  
  ob->fvv_force_fade *= 0.99; 
  f64 fade  = ob->fvv_force_fade; // between 1 (full force) and 0 (no force)
  //fade = fade*0.9;                // Exponentially decay force

  f64 pcurx = fvv_robot_center_x;
  f64 pcury = fvv_robot_center_y;
  f64 stiff = ob->plg_stiffness;
  f64 damp  = ob->plg_damping;

  /* Determine whether the subject is in the workspace or not */
  double dist = sqrt(pow(X-fvv_robot_center_x,2)+pow(Y-fvv_robot_center_y,2));
  ob->fvv_workspace_enter = (dist>distance_cutoff);


  fX= fade* ((-stiff*(X-pcurx) - damp*(vX)));
  fY= fade* ((-stiff*(Y-pcury) - damp*(vY)));

  do_capture();
  
#ifdef dyn_comp 
  dynamics_compensation(fX,fY,3,1.0);
# else
  ob->motor_force.x = fX;
  ob->motor_force.y = fY;
#endif
}





// curl controller
// with x=vy y=-vx, and +curl if it rotates cw. and -curl rotates ccw.

void
curl_ctl(u32 id)
{

  f64 curl;
  f64 damp;
  
  curl = ob->curl;
  damp = ob->damp;
  
  fX =  ( curl * (vY) );
  fY = -( curl * (vX) );

  #ifdef dyn_comp 
  dynamics_compensation(fX,fY,3,1.0);
# else
  ob->motor_force.x = fX;
  ob->motor_force.y = fY;
#endif

  ob->fvv_curl_force.x = ob->motor_force.x;
  ob->fvv_curl_force.y = ob->motor_force.y;
  
}






void
viscous_force_field(u32 id)
{

  f64 viscosity = ob->damp;
  
  fX = -viscosity * vX;
  fY = -viscosity * vY;

#ifdef dyn_comp 
  dynamics_compensation(fX,fY,3,1.0);
# else
  ob->motor_force.x = fX;
  ob->motor_force.y = fY;
#endif
  
  ob->fvv_curl_force.x = ob->motor_force.x;
  ob->fvv_curl_force.y = ob->motor_force.y;
  
}










// Adding force channel; added by Ananda (Sept 20) following old Gribble's utils.tcl

void force_channel(u32 id)
{
  f64 x1 = ob->plg_p1x;
  f64 y1 = ob->plg_p1y;
  f64 x2 = ob->plg_p2x;
  f64 y2 = ob->plg_p2y;
  f64 stiff = ob->plg_stiffness;
  f64 w = ob->plg_channel_width;
  // code from Jeremy Wong for force channel forces
  // step 1: slope & intercept of channel
  
  if (x1 == x2) {
    
    // then we only want x forces
    double f_norm = 0.0;
    f_norm = (fabs(X-x1)>w) * (X-x1-w) * stiff;
    fX = f_norm;
    fY = 0.0;
    
  } else if (y1 == y2) {
    
    double f_norm = 0.0;
    // then we only want y forces
    f_norm = (fabs(Y-y1)>w) * (Y-y1-w) * stiff;
    fX = 0.0;
    fY = f_norm;
    
  } else {
    
    f64 m_c = (y2-y1) / (x2-x1);
    f64 b_c = y2 - (m_c * x2);
    // step 2: slope & intercept of perpendicular line to channel
    f64 m_perp = -1 / m_c;
    f64 b_perp = Y - (m_perp * X);
    // step 3:
    f64 x_i = (b_perp - b_c) / (m_c - m_perp);
    f64 y_i = (x_i * m_perp) + b_perp;
    f64 l_perp = sqrt(((X-x_i) * (X-x_i)) + ((Y-y_i) * (Y-y_i)));
    f64 f_norm = ((l_perp-w)>0) * (l_perp - w)*stiff;
    f64 phi = atan2((Y - y_i),(X - x_i)); // need atan2 for direction.
    fX = f_norm * cos(phi);
    fY = f_norm * sin(phi);
  }
  #ifdef dyn_comp 
    	dynamics_compensation(fX,fY,3,1.0);
	# else
    	ob->motor_force.x = fX;
    	ob->motor_force.y = fY;
	#endif
  //dynamics_compensation(fX,fY,3,1.0);
}









void 
trajectory_capture(u32 id)
{
  /*
    This controller captures the positions of the robot while
    letting the subject move freely.
  */
  do_capture();

  fX = 0.0;
  fY = 0.0;
  
#ifdef dyn_comp 
  dynamics_compensation(fX,fY,3,1.0);
# else
  ob->motor_force.x = fX;
  ob->motor_force.y = fY;
#endif
}





//const double damp      = 40.0;
//const double stiffness = 4000.0;
//const double damp      = 4.0;
//const double stiffness = 400.0;


void 
trajectory_reproduce(u32 id)
{
  /*
    This controller reproduces a trajectory that has been loaded into memory.
    Essentially, it's a PD controller.

    Here is roughly what it does.
    - This controller expects that you have loaded a series of (x,y) points into the trajx,trajy arrays. 
    - The trajectory is supposed to have traj_n_samps number of samples.
    - We set a counter traj_cnt that starts at zero and runs for the length of the trajectory (in samples).
    - At each iteration, we find the (x,y) where we are supposed to be, and computes an estimate of the 
    velocity (vx,vy) that we are supposed to be moving at, and uses this to drive the motors.
    - When we are done, we set fvv_trial_phase to 100 and switch to holding the handle
    at the final position (traj_final_x,traj_final_y) so that other processes can pick up that
    our replay has ended.
  */
  int traj_cnt = traj_count;// cast to unsigned just to be sure
  if (traj_count<=traj_n_samps) {
    // If we are still active replaying the trajectory (haven't finished yet)
    f64 attractx   = trajx[traj_cnt];
    f64 estimatevx = (trajx[traj_cnt+1]-trajx[traj_cnt])*400.0; // estimate vX (400 Hz is sampling rate)
    fX = replay_stiffness*(attractx-X)+replay_damping*(estimatevx-vX);
    f64 attracty   = trajy[traj_cnt];
    f64 estimatevy = (trajy[traj_cnt+1]-trajy[traj_cnt])*400.0;
    fY = replay_stiffness*(attracty-Y)+replay_damping*(estimatevy-vY);
    ++traj_count;
  } else {
    // If we are no longer actively replaying the trajectory, we become
    // a point controller holding us at the end position
    // i.e. we mimick static_ctl
    ob->replay_done = 1;
    // signal to Tcl that we have ended, in case it's not obvious
    // once this is signaled we will actually go on to a different controller
    f64 stiff = ob->plg_stiffness;
    f64 damp  = ob->plg_damping;
    fX= (-stiff*(X-traj_final_x) - damp*(vX));
    fY= (-stiff*(Y-traj_final_y) - damp*(vY));
  }
#ifdef dyn_comp 
  dynamics_compensation(fX,fY,3,1.0);
# else
  ob->motor_force.x = fX;
  ob->motor_force.y = fY;
#endif
}

