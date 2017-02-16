// pc7266.c - US Digital ISA incremental encoder interface card
// http://www.usdigital.com/products/pc7266/
// part of the robot.o Linux Kernel Module

// InMotion2 robot system software for RTLinux

// Copyright 2003-2005 Interactive Motion Technologies, Inc.
// Cambridge, MA, USA
// http://www.interactive-motion.com
// All rights reserved

#include "rtl_inc.h"
#include "ruser.h"
#include "robdecls.h"

// TODO: possibly delete #include "asm/io.h"

// Linux reverses order of outp[bw]() args
#define inp    inb
#define outp(a,b)      outb((b),(a))

//  Define the resolution: ENCODER_COUNT
//  This number is derived by taking the number of lines of the encoder 
//  (1024) and multiplying by 10 for the interpolation and 4 for the
//  quadrature counters in the card. (1024*10*4 = 40960)

#define ENCODER_COUNT     0xA000
#define MAX_ENC_COUNT     0x1000000

// ISA ioport addresses
#define BASE              0X300  // Base address of counter card

// DAT 0-3 300, 302, 304, 306
// CTL 0-3 301, 303, 305, 307
#define DAT(ARG)	  (BASE+(2*(ARG)))
#define CTL(ARG)	  (DAT(ARG)+1)

// Mode Configurations for the PAL's:

// # Write an 8-bit number to address 8 and 9

// Mode 0: All Disabled.
// Mode 1: Load Counter Triggered by index.
// Mode 2: Reset Counter triggered by index.
// Mode 3: Interrupt Triggered by index.
// Mode 4: Load Counter and Interrupt triggered by index.
// Mode 5: Reset Counter and Interrupt triggered by index.
// Mode 6: Interrupt triggered by Carry.
// Mode 7: Reset Counter triggered by index, and interrupt triggered by carry.

// the board is labeled 1-4, but the code calls them 0-3.
#define PAL1              BASE+8 // register for PAL1 (controls mode of 0 & 1) 
#define PAL2              BASE+9 // register for PAL2 (controls mode of 2 & 3)

// LS7266 commands
#define CLOCK_DATA        3     // FCK frequency divider (orig 14)
#define CLOCK_SETUP       0X98   // transfer PR0 to PSC (x and y)
#define INPUT_SETUP       0XC1   // enable inputs A and B (x and y)
#define QUAD_X1           0XA8   // quadrature multiplier to 1 (x and y)
#define QUAD_X2           0XB0   // quadrature multiplier to 2 (x and y)
#define QUAD_X4           0XB8   // quadrature multiplier to 4 (x and y)
#define BP_RESET          0X01   // reset byte pointer
#define BP_RESETB         0X81   // reset byte pointer (x and y)
#define CNTR_RESET        0X02   // reset counter
#define CNTR_RESETB       0X82   // reset conter (x and y)
#define TRSFRPR_CNTR      0X08   // transfer preset to counter
#define TRSFRCNTR_OL      0X90   // transfer COUNTER to OL (output latch)
#define EFLAG_RESET       0X86   // reset E bit of the flag register
#define INDEX_DISABLE     0XE0   // disable the index register

// called by main.c:do_init()

void
pc7266_init(void)
{
    // f64 ang_out;
    u32 i;

    if (!rob->pc7266.have)
	    return;

#ifdef LATER
    // if we call this, we need to arrange to have exit called too
    if (!request_region(BASE, 8, "pc7266"))
	    return;
#endif // LATER

    outp(PAL1,0);
    outp(PAL2,0);

    inp(PAL1);	// why?

    for (i=0; i<4; i++) {
	outp(CTL(i), EFLAG_RESET);       // reset E bit of flag register
	outp(CTL(i), BP_RESETB);         // reset byte pointer (x and y)  
	outp(DAT(i), CLOCK_DATA);        // FCK frequency divider
	outp(CTL(i), CLOCK_SETUP);       // transfer PRO to PSC (x and y)
	outp(CTL(i), INPUT_SETUP);       // enable inputs A and B (x and y)
	outp(CTL(i), QUAD_X4);           // quadrature multiplier to 4 (x and y)
    }
}

#ifdef LATER
void
pc7266_exit(void)
{
    if (!rob->pc7266.have)
	    return;

    release_region(BASE, 8);
}
#endif // LATER

// read one encoder port.
// chan is a channel 0-3 (corresponds to 1-4 on the card).
// returns f64 encoder value

f64
pc7266_read_ch(u32 chan)
{
    s32 raw;
    f64 pos;

    if (!rob->pc7266.have)
	return (-1.0);

    outp(CTL(chan), TRSFRCNTR_OL);	// latch counter
    outp(CTL(chan), BP_RESET);		// reset byte pointer
    raw  = (s32)inp(DAT(chan));		// least significant byte 
    raw += (s32)inp(DAT(chan))<<8;
    raw += (s32)inp(DAT(chan))<<16;	// most significant byte

    // normalize negative values
    if ((rob->pc7266.max > 0) && (raw >= (rob->pc7266.max/2)))
	raw -= rob->pc7266.max;

    // raw (normalized) position
    rob->pc7266.raw[chan] = raw;

    // scaled position
    pos = (f64)raw * rob->pc7266.scale;

    return pos; 
}

// reset the four counters to zero.
// called when you set rob->pc7266.zero to other than 0.

void
pc7266_reset_all_ctrs(void)
{
    u32 i;

    if (!rob->pc7266.have)
	    return;

    for (i=0; i<4; i++) {
	outp(CTL(i), CNTR_RESET);       // reset counter
    }
}

// calibration helper function.
// set rob->pc7266.docal == 1 enter calibration mode.
// then move encoder to register index.
// then set docal to 2 to set back to normal mode.
// (this will then set docal to 0.)

void
pc7266_calib(void)
{
    if (!rob->pc7266.have)
	    return;

    switch (rob->pc7266.docal) {
    case 1:
    // Set to Mode 2 (reset counter), calibration mode.
    // (note that this is called "mode 2" but case 1.
	outp(PAL1,0x22);  // set counter 1 & 2 to mode 2 (index resets counter)
	outp(PAL2,0x22);  // set counter 3 & 4 to mode 2 (index resets counter)
	break;

    case 2:
    default:
    // back to normal mode.
	outp(PAL1,0x00);  //now ignore the index bit on channels 1 and 2
	outp(PAL2,0x00);  //now ignore the index bit on channels 3 and 4
	rob->pc7266.docal = 0;
	break;
    }
}

// read all four ports, this is what the main loop calls.

void
pc7266_encoder_read(void)
{
    u32 i;

    if (!rob->pc7266.have)
	return;

    for (i=0;i<4;i++) {
	rob->pc7266.enc[i] = pc7266_read_ch(i);
    }
}

