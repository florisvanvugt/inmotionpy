// math.c - math functions
// part of the robot.o Linux Kernel Module

// InMotion2 robot system software for RTLinux

// Copyright 2003-2005 Interactive Motion Technologies, Inc.
// Cambridge, MA, USA
// http://www.interactive-motion.com
// All rights reserved

#include "rtl_inc.h"
#include "ruser.h"
#include "robdecls.h"
//#include "userdecls.h"

// point2d operator * ( jacobian2d& J, point2d& p )

xy
jacob2d_x_p2d(mat22 J, se p)
{
    xy retp;
    retp.x = J.e00 * p.s + J.e01 * p.e;
    retp.y = J.e10 * p.s + J.e11 * p.e;
    return retp;
}

// jacobian2d operator * ( jacobian2d& J1, jacobian2d& J2 )

mat22
jacob2d_x_j2d(mat22 J1, mat22 J2)
{
    mat22 J;
    J.e00 = J1.e00 * J2.e00 + J1.e01 * J2.e10;
    J.e01 = J1.e00 * J2.e01 + J1.e01 * J2.e11;
    J.e10 = J1.e10 * J2.e00 + J1.e11 * J2.e10;
    J.e11 = J1.e10 * J2.e01 + J1.e11 * J2.e11;
    return J;
}

// jacobian2d jacobian2d::inverse()

mat22
jacob2d_inverse(mat22 Jin)
{
    mat22 J;
    f64 invdet = 1.0 / (Jin.e00 * Jin.e11 - Jin.e01 * Jin.e10);

    J.e00 =  Jin.e11 * invdet;	J.e01 = -Jin.e01 * invdet;
    J.e10 = -Jin.e10 * invdet;	J.e11 =  Jin.e00 * invdet;
    return J;
}

// jacobian2d jacobian2d::transpose()

mat22
jacob2d_transpose(mat22 Jin)
{
    mat22 J;

    J.e00 = Jin.e00;	J.e01 = Jin.e10;
    J.e10 = Jin.e01;	J.e11 = Jin.e11;
    return J;
}

// for calculating x/y

xy
xy_polar_cartesian_2d(se theta, se link)
{
    xy ret;

    ret.x = link.s * cos(theta.s) + link.e * cos(theta.e);
    ret.y = link.s * sin(theta.s) + link.e * sin(theta.e);

    return ret;
}

// for calculating velocities with a tachometer

mat22
j_polar_cartesian_2d(se theta, se link)
{
    mat22 J;

    J.e00 = -link.s * sin(theta.s); J.e01 = -link.e * sin(theta.e);
    J.e10 =  link.s * cos(theta.s); J.e11 =  link.e * cos(theta.e);

    return J;
}

// rotate a 2d point about the origin
xy
rotate2d(xy point, f64 angle)
{
	xy ret;

	ret.x = point.x * cos(angle) - point.y * sin(angle);
	ret.y = point.x * sin(angle) + point.y * cos(angle);

	return ret;
}

// translate a 2d point
xy
xlate2d(xy point, xy offset)
{
	xy ret;

	ret.x = point.x + offset.x;
	ret.y = point.y + offset.y;

	return ret;
}

#ifdef NOTDEF
// box_cwh_to_xy
// center wh box to x1y1 x2y2 box
box_xy
box_chw_to_xy(box_cwh boxcwh)
{
	box_xy ret;
	f64 w2, h2;

	w2 = boxcwh.wh.x / 2.0;
	h2 = boxcwh.wh.y / 2.0;

	ret.p1.x = boxcwh.c.x - w2;
	ret.p1.y = boxcwh.c.y - h2;
	ret.p2.x = boxcwh.c.x + w2;
	ret.p2.y = boxcwh.c.y + h2;

	return ret;
}

// box_xy_to_cwh
// x1y1 x2y2 box to center wh box
box_cwh
box_xy_to_cwh(box_xy boxxy)
{
	box_cwh ret;

	ret.c.x = (boxxy.p1.x + boxxy.p2.x) / 2.0;
	ret.c.y = (boxxy.p1.y + boxxy.p2.y) / 2.0;
	ret.wh.x = fabs(boxxy.p1.x - boxxy.p2.x) / 2.0;
	ret.wh.y = fabs(boxxy.p1.y - boxxy.p2.y) / 2.0;

	return ret;
}
#endif // NOTDEF

// base*xform+offset

f64
xform1d(f64 base, f64 xform, f64 offset)
{
    return (base * xform) + offset;
}

f64
dbracket(f64 x, f64 min, f64 max)
{
    if (x < min) x = min;
    if (x > max) x = max;
    return x;
}

s32
ibracket(s32 x, s32 min, s32 max)
{
    if (x < min) x = min;
    if (x > max) x = max;
    return x;
}

// preserve force orientation.
// if the max is 10, and the torques are -25 and 100,
// bring them down to -2.5 and 10.
// if a math operation has yielded NAN or INF, zero the forces.

se
preserve_orientation(se t, f64 max)
{
    f64 div;

    // don't divide by zero
    if (max < EPSILON) max = EPSILON;

    if (fabs(t.s) > max) {
	div = fabs(t.s) / max;
	t.s /= div;
	t.e /= div;
    }
    if (fabs(t.e) > max) {
	div = fabs(t.e) / max;
	t.s /= div;
	t.e /= div;
    }
    if (!finite(t.s) || !finite(t.e)) {
	t.s = 0.0;
	t.e = 0.0;
    }
    return t;
}

// butterworth filter

// [b,a]=butter(n,Wc)
// Butterworth lowpass filter
// order=n, cutoff=pi*Wc radians
// b and a are row vectors

// octave:1> [b,a]=butter(1,.1)
// b = 0.13673   0.13673
// a = 1.00000  -0.72654 

// octave:2> [b,a]=butter(1,.2)
// b = 0.24524   0.24524
// a = 1.00000  -0.50953

// case 10 at 200Hz should give similar results to case 2 at 1000Hz

f64
butter(f64 curr, f64 prev, f64 prevf)
{
	f64 butc[2];
	f64 ret;

	switch (ob->butcutoff) {
	case 1:
		butc[0] = .0155;
		butc[1] = -.9691;
		break;
	case 2:
		butc[0] = .0305;
		butc[1] = -.9391;
		break;
	case 3:
		butc[0] = .0450;
		butc[1] = -.9099;
		break;
	case 4:
		butc[0] = .0592;
		butc[1] = -.8816;
		break;
	case 5:
		butc[0] = .0730;
		butc[1] = -.8541;
		break;
	case 8:
		butc[0] = .1122;
		butc[1] = -.7757;
		break;
	case 10:
	case 0:
	default:
		butc[0] = .1367;
		butc[1] = -.7265;
	break;
	case 15:
		butc[0] = .1938;
		butc[1] = -.6128;
		break;
	case 20:
		butc[0] = .2452;
		butc[1] = -.5095;
		break;
	case 40:
		butc[0] = .4208;
		butc[1] = -.1584;
		break;
	case 50:
		butc[0] = .5;
		butc[1] = 0.0;
		break;
	}

	ret = butc[0] * curr
	    + butc[0] * prev
	    - butc[1] * prevf;

	return ret;
}


f64
butstop(f64 *raw, f64 *filt)
{
	// 55..65 bandstop
	// f64 butc[2][3] = {{.86325, .5402, .86325}, {0.0, .5402, .7265}};
	f64 butc[2][3] = {{.86325, .5402, .86325}, {1.0, .5402, .7265}};
	f64 ret;

	u32 i;

	ret = 0.0;
	for (i=0;i<3;i++) {
		ret += butc[0][i] * raw[i] 
		    - butc[1][i] * filt[i];
	}

	// bump the histories, for next sample.
	for (i = 2; i > 0; i--) {
		raw[i] = raw[i-1];
		filt[i] = filt[i-1];
	}
	filt[0] = ret;
	return ret;
}

// Savitzky Golay filter
// coeffs:
// M=2  nl=4  nr=0
// f64 coeff[] = { 0.086, -0.143, -0.086, 0.257, 0.886 };
// see Numerical Recipes in C, Chapter 14.8

// args: current value and history array
// return: filtered value

f64
apply_filter(f64 curr, f64 *hist)
{
	// curr is multiplied by [0]
	f64 coeff[] = { 0.086, -0.143, -0.086, 0.257, 0.886 };
	// f64 coeff[] = {0.127 -0.018, -0.103, -0.127, -0.091,
		// 0.006, 0.164, 0.382, 0.661};

	u32 coeffsize;

	f64 ret;
	u32 i;

	coeffsize = (sizeof(coeff) / sizeof(coeff[0]));
	// ob->scr[0] = coeffsize;

	// bump the prevs, insert curr
	for (i = coeffsize-1; i > 0; i--) {
		hist[i] = hist[i-1];
	}
	hist[0] = curr;

	// sum the products
	ret = 0.0;
	for (i = 0; i < coeffsize; i++) {
		ret += coeff[i] * hist[i];
	}

	// insert new filtered value for next time.
	hist[0] = ret;

	return ret;
}

// minimum jerk formula
// function of current time and total time.
// needs to be multiplied by distance to produce velocity

// f64 min_jerk(f64 currtime, f64 tottime)
// {
//     return ( 30.0 * pow(currtime,4.0) / pow(tottime,5.0)
//            - 60.0 * pow(currtime,3.0) / pow(tottime,4.0)
//            + 30.0 * pow(currtime,2.0) / pow(tottime,3.0) );
// }

// integral minimum jerk
// called with:
// current time (in ticks)
// terminal time (in ticks)
// current distance from base point (without minimum jerk)
//
// returns current min jerk distance from base point
//
// e.g.:
// constant velocity: start + i * distance / term
// min jerk: start + i_min_jerk(i, term, distance)

f64
i_min_jerk(u32 currtime, u32 tottime, f64 distance)
{
    if (tottime <= 0) return 0.0;
    return ( distance * (
	     6.0 * pow(currtime,5.0) / pow(tottime,5.0)
           - 15.0 * pow(currtime,4.0) / pow(tottime,4.0)
           + 10.0 * pow(currtime,3.0) / pow(tottime,3.0) ) );
}

// uses IMT's coefficients to generate a multiplication factor that scales in a minimum jerk trajectory. Input is a value ranging between zero and one
// written by Guillaume
f64 
i_mjerk_ggh(f64 time_ratio)
{
    if (time_ratio <= 0.0) return 0.0;
    //    f64 tRatio3 = (time_ratio * time_ratio * time_ratio);    
    return ( (time_ratio * time_ratio * time_ratio) *( time_ratio *( 6.0 * time_ratio - 15.0) + 10.0) );
}

// computes euclidean distance between 2 points (both input as xy variables)
// written by Guillaume
f64 
euc_dist2D(xy P1,xy P2)
{
    f64 interx,intery;
    interx = P2.x-P1.x;
    interx *= interx;
    intery = P2.y-P1.y;
    intery *=intery;

    return sqrt(interx+intery);
}


// delta radian normalize
// an encoder may spin around, returning a number between
// zero and 2*pi.  if we are calculating a velocity by subtracting
// successive position values, we have a problem with the position
// crossed from 2*pi to zero, or back.  To fix this, ensure that the
// value is in the range -pi..pi.  Also, cover the case where theta
// is outside the range -2pi..2pi.

f64 delta_radian_normalize(f64 theta)
{
        while (theta > M_PI)
                theta -= (2.0 * M_PI);
        while (theta < -M_PI)
                theta += (2.0 * M_PI);

	return theta;
}

// normalize to between 0 and 2pi
//
f64 radian_normalize(f64 theta)
{
        while (theta > (2.0 * M_PI))
                theta -= (2.0 * M_PI);
        while (theta < 0.0)
                theta += (2.0 * M_PI);

	return theta;
}

// bounds checks, so the trig funcs don't crash

f64 xasin(f64 x) {
    if (x < -1.0) x = -1.0;
    if (x > 1.0) x = 1.0;
    return asin(x);
}

f64 xacos(f64 x) {
    if (x < -1.0) x = -1.0;
    if (x > 1.0) x = 1.0;
    return acos(x);
}
