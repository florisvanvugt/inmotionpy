// isaft.c - ATI ISA Gamma force tranducer driver
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

// based on ATI ISA FT example software, see:
// 	http://www.ati-ia.com/download/isasoft.htm
//	http://www.ati-ia.com/download/csource.zip

//
//examples.c v1.3
//------------------------------------------------------------------
// Source Code for ATI ISA FT adapter
// Copyright 1998, 1999 ATI Industrial Automation, All rights reserved.
//
// This file contains sample C Source code routines for interfacing the
// ATI ISA/FT card.  It is intended for use by non-Windows applications
// in interfacing the card.
//
// The interrupt handler routine has compiler-specific calls; it may 
// require modification for compilers other than Borland C++.  All other 
// routines, including the polling method of data collection, should work
// in a standard C compiler with little or no modification.
//
// Windows applications are expected to use the ATIFT.OCX ActiveX driver
// supplied by ATI and do not require driver source code.
//
// This code should compile without errors using Borland C++ V4.51 or later.
//
// Author: Jerry Dahl and Dave Lora, ATI Industrial Automation
//------------------------------------------------------------------
//modifications
//
//Sam Skuce 5.28.2001
//	-modified calc_transform to calculate the TTF for itself, rather
//       than ask the user to supply it.
//Sam Skuce 9.26.2001
//	-created new function read_ft_point to read a single data point.


// #include <dos.h>	// inport(), outport()
// #include <math.h>	// sin(), cos()
// #include <sys/timeb.h>

#define  FALSE		0
#define  TRUE		1
#define  BASE		0x280	// default settings
#define  IRQ		7	//

// #define      INPW            inport          // the name of these functions may vary depending on compiler
// #define      OUTPW   outport
// #define      INPWB           inportb
// #define  OUTPWB      outportb

// Linux reverses order of out[bw]() args
#define	INPW	inw
#define OUTPW(a,b)  	outw((b),(a))
#define	INPWB	inb
#define OUTPWB(a,b)	outb((b),(a))

static u16 addr_reg = BASE;		// base I/O address of FT Card
static u16 data_reg = BASE + 2;		// addr_reg + 2
static u16 status_reg = BASE + 4;	// addr_reg + 4
// static u16 irq_num = IRQ;		// interrupt number
// static u16 data_available = FALSE;	// flag from interrupt handler
static u16 temp_buffer[500];		// temp command buffer

//------------------------------------------------------------------
// Copyright 1998, 1999 ATI Industrial Automation, All rights reserved.
//
// function:  send_command()
// sends a command to the card
//
// Uses global registers:
//	addr_reg        = PC I/O address of F/T card's address register
//	data_reg        = PC I/O address of F/T card's data register
//
// Calling parameters:
//	*bptr = pointer to command structure
//	numwords = length of command
//
// Return Value:	1       if successful
//			0       if abort
//------------------------------------------------------------------
static s32
send_command(u16 * bptr, s32 numwords)
{
    u16 dword, command, retcode;
    s32 i;

    //---------------------------------------
    // write command block to DPRAM at offset 0x000
    //---------------------------------------
    command = *bptr;		// save command word
    OUTPW(addr_reg, 0);		// set address register to command_block
    for (; numwords; numwords--)	// write command block to dual port RAM
    {
	dword = *bptr;
	OUTPW(data_reg, dword);
	bptr++;
    }
    //---------------------------------------
    // read MailBox at DPRAM 0x3FE to clear host interrupt flag
    //---------------------------------------
    OUTPW(addr_reg, 0x3FE);	// set mailbox address
    INPW(data_reg);		// reset interrupt flag

    OUTPW(addr_reg, 0x3FE);	// set mailbox address
    OUTPW(data_reg, command);	// clear acknowledgment

    //---------------------------------------
    // write command word to Mailbox at DPRAM 0x3FF to set
    // DSP interrupt flag
    //---------------------------------------
    OUTPW(addr_reg, 0x3ff);
    OUTPW(data_reg, command);

    //---------------------------------------
    //  waits for command completion (interrupt flag) from DSP
    //---------------------------------------
    retcode = TRUE;
    // forever
    // for(loop=TRUE;loop;)

    // we don't want it to spin forever, just long enough.
    for (i = 0; i < 1000000; i++) {
	dword = INPW(status_reg);

	// wait for interrupt flag
	if ((dword & 0x0010) == 0)	// interrupt set
	{
	    // read mailbox location at 0x3FE to reset interrupt flag
	    OUTPW(addr_reg, 0x3FE);	// write dpram address
	    retcode = INPW(data_reg);	// reset interrupt flag

	    // test mailbox for command acknowledgment
	    // if(retcode & 0x00ff) loop=FALSE;     // test for ack
	    if (retcode & 0x00ff)
		break;		// test for ack
	}
    }
    //---------------------------------------
    // enable output by setting command to zero
    //---------------------------------------
    OUTPW(addr_reg, 0);
    OUTPW(data_reg, 0);

    return (retcode);
}

//------------------------------------------------------------------
// Copyright 1998, 1999 ATI Industrial Automation, All rights reserved.
//
// function:  receive_command()
// reads numwords words from ft card starting at fifo offset 0
//
// global registers:
//    addr_reg  = PC I/O address of F/T card's address register
//    data_reg  = PC I/O address of F/T card's data register
//
// Calling parameters:
//        *bptr = pointer to command structure
//    numwords  = length of command
//------------------------------------------------------------------
static void
receive_command(u16 * bptr, s32 numword)
{
    u16 dword;

    OUTPW(addr_reg, 0);		// reset address
    for (; numword; numword--) {
	dword = INPW(data_reg);
	*bptr = dword;
	bptr++;
    }
}


#ifdef LATER
//-------------------------------------------------
// output_info command structure
//-------------------------------------------------
struct {
    u16 cmd_rc;
    u16 mode;
    u16 units;
    u16 resolution;
    u16 cnts_per_force;
    u16 cnts_per_torque;
    u16 cpf_div;
    u16 cpt_div;
} output_info;

//------------------------------------------------------------------
// Copyright 1998, 1999 ATI Industrial Automation, All rights reserved.
//
// function: get_output_info()
//      Sends Output_Information Command to F/T card
//        and fills output_info with data
//
// global registers:
//	addr_reg = PC I/O address of F/T card's address register
//	data_reg = PC I/O address of F/T card's data register
//
// Return Value:        1       if successful
//			0       if abort
//------------------------------------------------------------------
static s32
get_output_info(void)
{
    u16 *buf_ptr;

    output_info.cmd_rc = 0x0A00;	// Output Info command word
    output_info.mode = 0x0000;	// mode=read
    buf_ptr = &output_info.cmd_rc;
    if (!send_command(buf_ptr, 2))
	return (FALSE);
    receive_command(buf_ptr, sizeof(output_info) / 2);
    return (TRUE);
}
#endif // LATER

//------------------------------------------------------------------
// Copyright 1998, 1999 ATI Industrial Automation, All rights reserved.
//
// function: get_counts_info()
//      gets 12-bit (nominal) counts per force and torque, and units
//
// Calling parameters:
//	*cnts_force  = the nominal (12-bit) counts per force (default units)
//	*cnts_torque = the nominal (12-bit) counts per torque (default units)
//	*units           = the default units for the calibration (1=lbf,lbf-in;
//		2=N,N-mm; 3=N,N-m; 4=Kg,Kg-cm; 5=Klbf,Klbf-in)
//
//	Return Value:        1       if successful
//			     0       if abort
//------------------------------------------------------------------
static s32
get_counts_info(u16 * cnts_force, u16 * cnts_torque, u16 * units)
{
    u16 *buf_ptr;

    temp_buffer[0] = 0x0100;	// Sensor Data command word
    temp_buffer[1] = 0x0000;	// mode = read
    buf_ptr = &temp_buffer[0];
    if (!send_command(buf_ptr, 2))
	return (FALSE);
    receive_command(buf_ptr, 147);	// number of words to read = 147
    *cnts_force = temp_buffer[139];
    *cnts_torque = temp_buffer[140];
    *units = temp_buffer[145];
    return (TRUE);
}

//function read_ft_point
//      reads a single f/t point into argument data
//
//arguments
//	data - an array of 8 longs to hold f/t data
//	data[0] = sequence
//		[1] = monitor condition
//		[2] = fx
//		[3] = fy
//		[4] = fz
//		[5] = tx
//		[6] = ty
//		[7] = tz

// this is based on ATI's read_ft_point, but does not do interrupt mode,
// it just polls.  the ISA board at up to 7800 Hz, it will always be ready.

static void
isa_read_ft_point(s32 data[])
{
    s32 i;
    s16 dword;
    s32 lword, lwordh;

    //-----------------------------------------
    // new data available.
    // first get f/t buffer address
    //-----------------------------------------

    OUTPW(addr_reg, 0x3FE);	// write dpram address
    dword = INPW(data_reg);	// reset int flag
    if ((dword & 0xff00) == 0xff00)
	dword = dword & 0x01ff;
    else
	dword = 0x100;

    OUTPW(addr_reg, dword);	// starting addr
    dword = INPW(data_reg);	// read 1st word=Sequence Number
    data[0] = (u16) dword;
    dword = INPW(data_reg);	// read 2nd word=Monitor Condition
    data[1] = (u16) dword;

    //-----------------------------------------
    // get the new F/T data in counts
    // to get actual forces and torque, the application
    // should divide the counts by the
    // counts_per_force and counts_per_torque
    // conversion factors
    //-----------------------------------------
    for (i = 0; i < 6; i++)	// get force in counts
    {
	lword = INPW(data_reg);	// start at 3rd word
	lwordh = INPW(data_reg);
	lword = lword + (lwordh << 16);	// convert to 32-bit long
	data[i + 2] = lword;
    }
}

void
isa_ft_init(void)
{
    if (!ob->have_isaft)
	return;

    get_counts_info(&rob->isaft.cpf, &rob->isaft.cpt,
		&rob->isaft.units);
}
// get a single ISA sample.
// get_counts_info wakes up the board and asks for conversion factors.
// it should only be called after have_isaft is set, and only once.

void
isa_ft_read(void)
{
    if (!ob->have_isaft)
	return;

    // get a single sample
    isa_read_ft_point(rob->isaft.iraw);
}
