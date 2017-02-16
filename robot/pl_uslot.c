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
#include "userdecls.h"
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
void moh_consolidation(void);
void moh_stiffness(void);
void point_box_ctl(u32);
void point_ctl(u32);
void damp_ctl(u32);
void rotate_ctl(u32);
void adap_ctl(u32);
void sine_ctl(u32);
void curl_ctl(u32);
void planar_fn_ctl(u32);
void planar_const_ctl(u32);
void flasher_ctl(u32);
void curl_ctl(u32);

void null_ctl_dyncomp(u32);
void null_ctl(u32);
void null_ic_ctl(u32);
void zero_ft(u32);
void zero_ft_nodyncomp(u32);
void curlfield(u32);
void movetopt(u32);
void forcechannel(u32);
void static_ctl(u32);


/*void wrist_ctl(u32);
void ankle_ctl(u32);
void curl_ctl(u32);
void wrist_fn_ctl(u32);
void wrist_curl_ctl(u32);
void wrist_ps_ctl(u32);
void planar_fn_ctl(u32);
void planar_const_ctl(u32);
void ankle_point_ctl(u32);
void linear_ctl(u32);
void linear_point_ctl(u32); //Added: Pontus
void flasher_ctl(u32);
void linear_adap_ctl(u32);
void wrist_adap_ctl(u32);
void wrist_ps_adap_ctl(u32);
void hand_ctl(u32);
*/

void
init_slot_fns(void)
{
  //ob->slot_fns[0] = null_ctl_dyncomp;
  ob->slot_fns[0] = null_ctl;
  ob->slot_fns[1] = null_ctl_dyncomp;
  ob->slot_fns[2] = zero_ft_nodyncomp;
  ob->slot_fns[3] = damp_ctl;
  ob->slot_fns[4] = movetopt;  // move the subject to px,py in time t
  ob->slot_fns[5] = adap_ctl;
  ob->slot_fns[6] = sine_ctl;
  ob->slot_fns[7] = planar_fn_ctl;
  ob->slot_fns[8] = planar_const_ctl;
  ob->slot_fns[9] = curl_ctl;
  ob->slot_fns[10] = null_ctl;     // forces are zero, no inertial compensation
  ob->slot_fns[11] = null_ic_ctl;  // forces are zero, yes inertial compensation
  ob->slot_fns[12] = zero_ft;      // used when zeroing the ATI force transducer
  ob->slot_fns[13] = curl_ctl;    // a velocity-dependent force field
  ob->slot_fns[14] = movetopt;     // move the subject to px,py in time t
  ob->slot_fns[15] = forcechannel; // a force channel to px,py width w
  ob->slot_fns[16] = static_ctl;   // stay at p1x p1y under active control
}

#define X ob->pos.x
#define Y ob->pos.y
//#define vX ob->fsoft_vel.x
//#define vY ob->fsoft_vel.y
//#define fX ob->ba_fx
//#define fY ob->ba_fy
//#define fX moh->fX
//#define fY moh->fY
#define hand_place moh->hand_place
#define servo_flag moh->servo_flag
#define stiffness moh->stiffness
#define ffs moh->ffs
#define centerX moh->xcenter
#define centerY moh->ycenter
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


void
moh_consolidation(void)
{
  f64 damp = -10.0;
  if (hand_place==3  ) {
    
    fX = stiffness*(centerX-X)+damp*vX;
    fY = stiffness*(centerY-Y)+damp*vY;
    stiffness = stiffness+0.7;
    if (stiffness>100) {
      stiffness = 100.0;
    }
    
  } 
  if (hand_place==1){
    fX = 13.0*ffs*vY;
    fY = -13.0*ffs*vX;
    /* 	fX = 10.0*vY; */
    /* 	fY = -10.0*vX; */
    stiffness = 8.0;
  }
  if (hand_place==2) {
    
    fX = 15.0*ffs*vY;
    fY = -15.0*ffs*vX;
    /* 	fX = 10.0*vY; */
    /* 	fY = -10.0*vX; */
    stiffness = 8.0;
  }
#ifdef dyn_comp 
  dynamics_compensation(fX,fY,3,1.0);
# else
  ob->motor_force.x = fX;
  ob->motor_force.y = fY;
#endif
}

void
moh_stiffness(void)
{
  f64 damp = -100.0; 
  f64 temp_stiffness = 3000.0;
  f64 temp,xcnt,ycnt,temp1;
  f64 x0,xn,y0,yn;
  if (servo_flag == 1) {
    if (cnt<=40.0) {
      x0 = refX; y0 = refY; 
      xn = refX+0.006*cos(dir); yn = refY+0.006*sin(dir);
      temp1 = (15.0*pow(cnt/40.0,4)) - (6.0*pow(cnt/40.0,5)) - (10.0*pow(cnt/40.0,3)); 
      xcnt = x0+(x0-xn)*temp1;
      ycnt = y0+(y0-yn)*temp1;
      //xcnt = refX-(0.006*cos(dir)*temp1);
      //ycnt = refY-(0.006*sin(dir)*temp1);
      ++cnt;
    } else if (cnt>40.0 && cnt<120.0) {
      xn = refX+0.006*cos(dir); yn = refY+0.006*sin(dir);
      xcnt = xn; ycnt = yn;
      //xcnt = refX+(0.006*cos(dir)); ycnt = refY+(0.006*sin(dir));
      ++cnt;
    } else if (cnt>=120.0 && cnt<=160.0) {
      temp = cnt-120.0;
      x0 = refX+0.006*cos(dir); y0 = refY+0.006*sin(dir); xn = refX; yn = refY;
      temp1 = (15.0*pow(temp/40.0,4)) - (6.0*pow(temp/40.0,5)) - (10.0*pow(temp/40.0,3));
      //xcnt = x0;
      //ycnt = y0;
      xcnt = x0+(x0-xn)*temp1;
      ycnt = y0+(y0-yn)*temp1;
      //xcnt = refX+(0.006*cos(dir))+(0.006*cos(dir)*temp1);
      //xcnt = refY+(0.006*sin(dir))+(0.006*sin(dir)*temp1);
      ++cnt;
    } else if (cnt>160.0 && cnt<200.0) {
      xcnt = refX; ycnt = refY;
      ++cnt;
    } else {
      servo_flag = 2.0;
      xcnt = refX; ycnt = refY; 
    }
    
    //fX = xcnt - X;
    //fY = ycnt - Y;
    fX = temp_stiffness*(xcnt-X) + damp*vX;
    fY = temp_stiffness*(ycnt-Y) + damp*vY;
    //fX = 3.0;
    //fY = 0.0;
  } else {
    fX = 0.0;
    fY = 0.0;
    cnt = 0.0;
  }
#ifdef dyn_comp 
  dynamics_compensation(fX,fY,3,1.0);
# else
  ob->motor_force.x = fX;
  ob->motor_force.y = fY;
#endif
}

// when you get to the end of a slot and it doesn't stop,
// you might want to stiffen the controller, as a function of time.

// e.g., you want the slot to triple in stiffness over two seconds at 200Hz.
// call: slot_term_stiffen(id, 400, 3.0)

// note that this does not change the stiffness, it changes the x/y
// motor forces after they are calculated but before they are pfo'd.

static void
slot_term_stiffen(u32 id, u32 time, f64 imult) {
  u32 termi;
  f64 mult;
  
  termi = ob->slot[id].termi;
  
  if (termi <= 0 || time <= 0 || imult < 0.0) return;
  
  if (termi > time) termi = time;
  
  mult = 1.0 + (termi * (imult - 1.0) / time);
  // new 5/06...
  // todo...
  // ob->stiffener = some f(imult, termi)
  // mult = (100.0 + ob->stiffener) / 100.0;
  
  fX *= mult;
  fY *= mult;
}

void
point_box_ctl(u32 id)
{

  f64 damp;
  
  damp = ob->damp;
  
  ob->ref.pos.x = ob->safety.pos;
  ob->ref.pos.y = ob->safety.pos;
  
  fX = -( ob->stiff * (X - ob->ref.pos.x)
	  + damp * (vX - ob->ref.vel.x) );
  
  fY = -( ob->stiff * (Y - ob->ref.pos.y)
	  + damp * (vY - ob->ref.vel.y) );
  
  if ( !(X < -ob->safety.pos
	 || X >  ob->safety.pos
	 || Y < -ob->safety.pos
	 || Y >  ob->safety.pos
	 )) {
    fX = 0.0;
    fY = 0.0;
  }
}

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

/* // moving box controller
void
simple_ctl(u32 id)
// moving_box_ctl(u32 id)
{
    f64 x, y, w, h;		// center x/y, width, height

				// minus, plus, minus,  plus
    f64 l, r, b, t;		// left, right, bottom, top
    f64 lx, ly, lw, lh;		// length b0 to b1.
    f64 w2, h2;

    u32 i, term;		// index and termination
    f64 damp;

    f64 fx, fy;			// intermediate values for fX and FY

    dpr(3, "simple_controller\n");

    // calculate lengths
    lx = ob->slot[id].b1.point.x - ob->slot[id].b0.point.x;
    ly = ob->slot[id].b1.point.y - ob->slot[id].b0.point.y;
    lw = ob->slot[id].b1.w - ob->slot[id].b0.w;
    lh = ob->slot[id].b1.h - ob->slot[id].b0.h;

    term = ob->slot[id].term;
    if (term == 0) term = 1;
    i = ob->slot[id].i;

    // box xywh
    // ob->slot[id].bcur.point.x = x = ob->slot[id].b0.point.x + i * lx / term;
    // ob->slot[id].bcur.point.y = y = ob->slot[id].b0.point.y + i * ly / term;

    ob->slot[id].bcur.point.x = x = ob->slot[id].b0.point.x + 
	    i_min_jerk(i, term, lx);
    ob->slot[id].bcur.point.y = y = ob->slot[id].b0.point.y + 
	    i_min_jerk(i, term, ly);

    ob->slot[id].bcur.w = w = ob->slot[id].b0.w + i * lw / term;
    ob->slot[id].bcur.h = h = ob->slot[id].b0.h + i * lh / term;

    // wall lrtb
    w2 = w / 2.0;
    h2 = h / 2.0;
    l = x - w2;
    r = x + w2;
    b = y - h2;
    t = y + h2;



    damp = ob->damp;

    // in case nothing triggers, should not happen.

    // use -damp*V as the "zero" force, or you will feel
    // a bump when you "hit the wall of the box."

    // was...
    // fx = 0.0;
    // fy = 0.0;
    fx = -damp * vX;
    fy = -damp * vY;

    // outside
    if (X < l)
	fx = -((ob->stiff * (X - l)) + (damp * vX));
    if (X > r)
	fx = -((ob->stiff * (X - r)) + (damp * vX));
    if (Y < b)
	fy = -((ob->stiff * (Y - b)) + (damp * vY));
    if (Y > t)
	fy = -((ob->stiff * (Y - t)) + (damp * vY));

    // inside
    if (X > l && X < r && Y > b && Y < t) {
	// fx = 0.0;
	// fy = 0.0;
	fx = -damp * vX;
	fy = -damp * vY;
    }
    fX = fx;
    fY = fy;
    // ob->scr[0] = X;
    // ob->scr[1] = Y;
    // ob->scr[2] = l;
    // ob->scr[3] = b;
    // ob->scr[4] = r;
    // ob->scr[5] = t;
    // ob->scr[6] = i;
    slot_term_stiffen(id, 400, 3.0);
}
*/
// rotating box controller
void
rotate_ctl(u32 id)
{
    // this controller normalizes the slot along the +X axis.
    // box0 is src point, box1 is dest point.
    // base-center is offset by -x/-y to (0,0)
    // and rotated to "3 o'clock" by the angle -rot,
    // so the base lies along the y axis, 
    // the "width" along the (vertical) y axis,
    // and "height" pointing zero degrees to 3 o'clock.

    // all these are in normalized space:
    // (for the boxes, the x/y are normalized, the w/h are always normal)
    xy ncfvec;			// command force vector
    xy npos;			// manipulandum position
    xy nvel;			// rotated velocity
    box nmov;			// the moving box

    f64 left, right, bottom, top;

    // these are in world space
    xy end;			// b1-b0;
    xy off;			// offset (-b0)

    // these are space independent
    f64 dwidth;
    f64 w2;
    f64 rot;			// angle of rotation
    f64 hyp;			// hypotenuse 

    u32 i, term;		// index and termination
    f64 damp, stiff;

    dpr(3, "rotating_controller\n");

    // x and y components of right triangle formed by b0/b1
    end.x = ob->slot[id].b1.point.x - ob->slot[id].b0.point.x;
    end.y = ob->slot[id].b1.point.y - ob->slot[id].b0.point.y;

    // yes, atan2 y,x and hypot x,y
    rot = atan2(end.y, end.x);
    hyp = hypot(end.x, end.y);

    // delta width
    dwidth = ob->slot[id].b1.w - ob->slot[id].b0.w;

    term = ob->slot[id].term;
    if (term == 0) term = 1;
    i = ob->slot[id].i;

    // move the box from origin along +x in normal space
    // the current point is i/term along the length line
    // (you might make i/term a minimum jerk fn if you like)
    nmov.point.x = i_min_jerk(i, term, hyp);
    nmov.point.y = 0.0;	// y does not move
    nmov.w = ob->slot[id].b0.w + i * dwidth / term;
    nmov.h = hyp - nmov.point.x;

    // +x slot
    // wall lrtb
    w2 = nmov.w / 2.0;

    left = nmov.point.x;
    right = nmov.point.x + nmov.h;
    bottom = nmov.point.y - w2;
    top = nmov.point.y + w2;

    damp = ob->damp;
    stiff = ob->stiff;

    // in case nothing triggers, should not happen.

    // use -damp*V as the "zero" force, or you will feel
    // a bump when you "hit the wall of the box."

    // was...
    // fx = 0.0;
    // fy = 0.0;

    // normalize manipulandum point/position
    off.x = -ob->slot[id].b0.point.x;
    off.y = -ob->slot[id].b0.point.y;
    npos = rotate2d(xlate2d(ob->pos, off), -rot);
    // normalize velocity vector
    nvel = rotate2d(ob->vel, -rot);

    // todo: normalize ft sensor vals for force feedback
    // todo: variable damping

    // calculate default command force vector
    ncfvec.x = -damp * nvel.x;
    ncfvec.y = -damp * nvel.y;

    // outside
    if (npos.x < left)
	ncfvec.x = -((stiff * (npos.x - left)) + (damp * nvel.x));
    if (npos.x > right)
	ncfvec.x = -((stiff * (npos.x - right)) + (damp * nvel.x));
    if (npos.y < bottom)
	ncfvec.y = -((stiff * (npos.y - bottom)) + (damp * nvel.y));
    if (npos.y > top)
	ncfvec.y = -((stiff * (npos.y - top)) + (damp * nvel.y));

    // inside
    if (npos.x > left && npos.x < right && npos.y > bottom && npos.y < top) {
	ncfvec.x = -damp * nvel.x;
	ncfvec.y = -damp * nvel.y;
    }

    // we have a normal force vector, rotate it back to world space.
    
    ob->motor_force = rotate2d(ncfvec, rot);

    slot_term_stiffen(id, 600, 5.0);
}

// adaptive controller
// derived from rotate_ctl

// NOTE!  this controller does not follow the "morphing rectangle" paradigm
// of the other controllers.  it takes a starting x/y point and
// a finishing x/y point, and assumes that there is a collapsing slot
// between them.  this is strange and sad, but currently true.

void
adap_ctl(u32 id)
{
    // this controller normalizes the slot along the +X axis.
    // box0 is src point, box1 is dest point.
    // base-center is offset by -x/-y to (0,0)
    // and rotated to "3 o'clock" by the angle -rot,
    // so the base lies along the y axis, 
    // the "width" along the (vertical) y axis,
    // and "height" pointing zero degrees to 3 o'clock.

    // all these are in normalized space:
    // (for the boxes, the x/y are normalized, the w/h are always normal)
    xy ncfvec;			// command force vector
    xy npos;			// manipulandum position
    xy nvel;			// rotated velocity
    xy nftforce;		// ft force
    box nmov;			// the moving box

    f64 left, right, bottom, top;

    xy ftvec;			// horizontal ft force vector

    // these are in world space
    xy end;			// b1-b0;
    xy off;			// offset (-b0)

    // these are space independent
    f64 dwidth;
    f64 w2;
    f64 rot;			// angle of rotation
    f64 hyp;			// hypotenuse 

    u32 i, term;		// index and termination
    f64 damp, stiff, side_stiff;

    dpr(3, "adaptive_controller\n");

    // x and y components of right triangle formed by b0/b1
    end.x = ob->slot[id].b1.point.x - ob->slot[id].b0.point.x;
    end.y = ob->slot[id].b1.point.y - ob->slot[id].b0.point.y;

    // yes, atan2 y,x and hypot x,y
    rot = atan2(end.y, end.x);
    hyp = hypot(end.x, end.y);

    // delta width
    dwidth = ob->slot[id].b1.w - ob->slot[id].b0.w;

    term = ob->slot[id].term;
    if (term == 0) term = 1;
    i = ob->slot[id].i;

    // move the box from origin along +x in normal space
    // the current point is i/term along the length line
    // (you might make i/term a minimum jerk fn if you like)
    nmov.point.x = i_min_jerk(i, term, hyp);
    nmov.point.y = 0.0;	// y does not move
    nmov.w = ob->slot[id].b0.w + i * dwidth / term;
    nmov.h = hyp - nmov.point.x;

    // +x slot
    // wall lrtb
    w2 = nmov.w / 2.0;

    left = nmov.point.x;
    right = nmov.point.x + nmov.h;
    bottom = nmov.point.y - w2;
    top = nmov.point.y + w2;

    damp = ob->damp;
    stiff = ob->stiff;
    side_stiff = ob->side_stiff;

    // in case nothing triggers, should not happen.

    // use -damp*V as the "zero" force, or you will feel
    // a bump when you "hit the wall of the box."

    // was...
    // fx = 0.0;
    // fy = 0.0;

    // normalize manipulandum point/position
    off.x = -ob->slot[id].b0.point.x;
    off.y = -ob->slot[id].b0.point.y;
    npos = rotate2d(xlate2d(ob->pos, off), -rot);
    // normalize velocity vector
    nvel = rotate2d(ob->vel, -rot);
    // normalize ft force
    ftvec.x = rob->ft.world.x;
    ftvec.y = rob->ft.world.y;
    nftforce = rotate2d(ftvec, -rot);

    ob->norm.x = npos.x;
    ob->back.x = nmov.point.x;

    // performance metrics

// comments from adapctr2.cpp

// pm1 : initialization
// pm2a: sum of power along target axis
// pm2b: sum of deviation from min. jerk pos
// pm3 : sum of (distance normal to target axis)^2
// pm4 : max distance along target axis
// npts: number of points in the above sums

// ngvec[4+11*game_n] = ngvec[4+11*game_n] + sFx*sVx; // PM2a
// ngvec[5+11*game_n] = ngvec[5+11*game_n] + sX-lmw; // PM2b
// ngvec[6+11*game_n] = ngvec[6+11*game_n] + sY*sY; // PM3
// if (fabs(sX) > ngvec[7+11*game_n]) ngvec[7+11*game_n] = fabs(sX); // PM4
// ngvec[8+11*game_n]++; // npts

    // note that these pm.things are all state, except for npoints
    // npoints must be zeroed to init, the rest need not be zeroed.

    if (npos.x > nmov.point.x) {
	ob->pm.active_power += 0;
	ob->pm.min_jerk_deviation += fabs(npos.x - nmov.point.x); // pm2b
    } else {
	ob->pm.active_power += (nvel.x * nftforce.x); // pm2a
	ob->pm.min_jerk_deviation += 0;
    }

    ob->pm.min_jerk_dgraph += fabs(npos.x - nmov.point.x); // graph
    ob->pm.dist_straight_line += (npos.y * npos.y); // pm3

    if (ob->pm.max_dist_along_axis < fabs(npos.x))
	ob->pm.max_dist_along_axis = fabs(npos.x); // pm4
    ob->pm.npoints++; // npts

    // todo: normalize ft sensor vals for force feedback
    // todo: variable damping

    // calculate default command force vector
    ncfvec.x = -damp * nvel.x;
    ncfvec.y = -damp * nvel.y;

    // outside
    if (npos.x < left)
	ncfvec.x = -((stiff * (npos.x - left)) + (damp * nvel.x));
    if (npos.x > right)
	ncfvec.x = -((stiff * (npos.x - right)) + (damp * nvel.x));
    if (npos.y < bottom)
	ncfvec.y = -((side_stiff * (npos.y - bottom)) + (damp * nvel.y));
    if (npos.y > top)
	ncfvec.y = -((side_stiff * (npos.y - top)) + (damp * nvel.y));

    // inside
    if (npos.x > left && npos.x < right && npos.y > bottom && npos.y < top) {
	ncfvec.x = -damp * nvel.x;
	ncfvec.y = -damp * nvel.y;
    }

    // we have a normal force vector, rotate it back to world space.
    
    ob->motor_force = rotate2d(ncfvec, rot);
    slot_term_stiffen(id, 600, 5.0);
}

//
// sine_ctl
// output an open loop sine wave.
// test_raw_torque must be set.
//
void
sine_ctl(u32 id)
{
    f64 x, v, sv, ev;

    if (!ob->test_raw_torque)
	    return;

    x = (ob->Hz * ob->sin_period);
    if (x <= 0.0) x = 1.0;
    x = ob->i * (2.0 * M_PI) / x;
    v = sin(x) * ob->sin_amplitude;

    sv = 0.0;
    if (ob->sin_which_motor & 1) sv = v;
    ob->raw_torque_volts.s = sv;
    ev = 0.0;
    if (ob->sin_which_motor & 2) ev = v;
    ob->raw_torque_volts.e = ev;
}

// curl controller
// with x=vy y=-vx, and +curl, it rotates cw. -curl rotates ccw.

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
}

static f64
pl_i_ref_fn(u32 i, u32 term, f64 phase, f64 amplitude) {
	return (amplitude * cos((2.0 * M_PI * i / term) + phase));
}

// planar function contrller
void
planar_fn_ctl(u32 id)
{
    f64 x, y, w, h;		// center x/y, width, height

				// minus, plus, minus,  plus
    f64 l, r, b, t;		// left, right, bottom, top
    f64 lx, ly, lw, lh;		// length b0 to b1.
    f64 w2, h2;

    u32 i, term;		// index and termination
    f64 damp;

    f64 fx, fy;			// intermediate values for fX and FY

    dpr(3, "planar function controller\n");

    // calculate lengths
    lx = ob->slot[id].b1.point.x - ob->slot[id].b0.point.x;
    ly = ob->slot[id].b1.point.y - ob->slot[id].b0.point.y;
    lw = ob->slot[id].b1.w - ob->slot[id].b0.w;
    lh = ob->slot[id].b1.h - ob->slot[id].b0.h;

    term = ob->slot[id].term;
    if (term == 0) term = 1;
    i = ob->slot[id].i;

    ob->slot[id].bcur.point.x = x = ob->slot[id].b0.point.x + 
	pl_i_ref_fn(i, term, 0., 0.1);
    ob->slot[id].bcur.point.y = y = ob->slot[id].b0.point.y + 
	pl_i_ref_fn(i, term, M_PI / 2.0, 0.1);

    ob->slot[id].bcur.w = w = ob->slot[id].b0.w + i * lw / term;
    ob->slot[id].bcur.h = h = ob->slot[id].b0.h + i * lh / term;

    // wall lrtb
    w2 = w / 2.0;
    h2 = h / 2.0;
    l = x - w2;
    r = x + w2;
    b = y - h2;
    t = y + h2;



    damp = ob->damp;

    // in case nothing triggers, should not happen.

    // use -damp*V as the "zero" force, or you will feel
    // a bump when you "hit the wall of the box."

    // was...
    // fx = 0.0;
    // fy = 0.0;
    fx = -damp * vX;
    fy = -damp * vY;

    // outside
    if (X < l)
	fx = -((ob->stiff * (X - l)) + (damp * vX));
    if (X > r)
	fx = -((ob->stiff * (X - r)) + (damp * vX));
    if (Y < b)
	fy = -((ob->stiff * (Y - b)) + (damp * vY));
    if (Y > t)
	fy = -((ob->stiff * (Y - t)) + (damp * vY));

    // inside
    if (X > l && X < r && Y > b && Y < t) {
	// fx = 0.0;
	// fy = 0.0;
	fx = -damp * vX;
	fy = -damp * vY;
    }
    fX = fx;
    fY = fy;
    // ob->scr[0] = X;
    // ob->scr[1] = Y;
    // ob->scr[2] = l;
    // ob->scr[3] = b;
    // ob->scr[4] = r;
    // ob->scr[5] = t;
    // ob->scr[6] = i;
    slot_term_stiffen(id, 400, 3.0);
}

// constant force contrller

void
planar_const_ctl(u32 id)
{
    fX = ob->const_force.x;
    fY = ob->const_force.y;
}

// flasher
void
flasher_ctl(u32 id)
{
    u32 mod;

    mod = (u32)ob->scr[0];
    // switch both leds when i%mod == 0.
    if ((ob->i % mod) == 0) {
	daq->dout0 = !daq->dout0;
	daq->dout1 = !daq->dout1;
    }
}

/*
void
null_ctl(u32 id)
{
  fX = ob->plg_last_fX * 0.0;
  fY = ob->plg_last_fY * 0.0;
  //#ifdef dyn_comp 
  //  dynamics_compensation(fX,fY,3,1.0);
  //# else
  ob->motor_force.x = fX;
  ob->motor_force.y = fY;
  //#endif
}
*/



void
null_ctl(u32 id)
{
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
null_ic_ctl(u32 id)
{
  fX = ob->plg_last_fX * 0.90;
  fY = ob->plg_last_fY * 0.90;
  dynamics_compensation(fX,fY,3,1.0);
}

void
zero_ft(u32 id)
{
  int i;
  for (i=0; i<6; i++) {
    ob->plg_ftzero[i] += -rob->ft.raw[i];
  }
  ob->plg_ftzerocount++;
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




void
curlfield(u32 id)
{
/*
  f64 b_xx = ob->plg_curlmat[0];
  f64 b_xy = ob->plg_curlmat[1];
  f64 b_yx = ob->plg_curlmat[2];
  f64 b_yy = ob->plg_curlmat[3];
  fX = -( (b_xx * vY) - (b_xy * vX) );
  fY = -( (b_yx * vX) - (b_yy * vY) );
  #ifdef dyn_comp 
    	dynamics_compensation(fX,fY,3,1.0);
	# else
    	ob->motor_force.x = fX;
    	ob->motor_force.y = fY;
	#endif
	*/

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

void forcechannel(u32 id)
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
static_ctl(u32 id)
{
f64 pcurx = ob->plg_p1x;
f64 pcury = ob->plg_p1y;
f64 stiff = ob->plg_stiffness;
f64 damp = damp = ob->plg_damping;
fX= (-stiff*(X-pcurx) - damp*(vX));
fY= (-stiff*(Y-pcury) - damp*(vY));
#ifdef dyn_comp 
 dynamics_compensation(fX,fY,3,1.0);
# else
 ob->motor_force.x = fX;
 ob->motor_force.y = fY;
#endif
}
