// userdecls.h - data declarations for the InMotion2 robot software system
//

// InMotion2 robot system software for RTLinux

// Copyright 2003-2005 Interactive Motion Technologies, Inc.
// Cambridge, MA, USA
// http://www.interactive-motion.com
// All rights reserved

#include <math.h>
#include "robdecls.h"


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

extern Moh *moh;
