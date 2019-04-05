// uei.c - i/o for the UEI PowerDAQ board
// part of the robot.o Linux Kernel Module

// InMotion2 robot system software for RTLinux

// Copyright 2003-2005 Interactive Motion Technologies, Inc.
// Cambridge, MA, USA
// http://www.interactive-motion.com
// All rights reserved

typedef int s32;
typedef char s8;
typedef unsigned long long u64;
typedef u64 hrtime_t;

#include "rtl_inc.h"
#include "uei_inc.h"
#include "robdecls.h"

// 7/04 apparently, some AO boards report as 0x42
// derived from powerdaq-2.1/include/pdfw_def.h
// TODO: delete #undef PD_IS_AO
// TODO: delete #define PD_IS_AO(id)    (((id>=0x14E)&&(id<=0x158))||(id==0x42))

// uei_ain_init and uei_ain_read are derived from the pd_ain uei example.

// Ain config: bipolar, +/-10V range, CL = SW, CV = cont, Single Ended
// #define AIN_CFG AIN_BIPOLAR | AIN_RANGE_10V | AIB_CVSTART0 | AIB_CVSTART1

// assuming certain bit values are zero is unwise.
// Ain config:   bipolar,      +/-10V range,   CL = SW,          CV = cont,                Single Ended

#define AIN_CFG (AIN_BIPOLAR | AIN_CL_CLOCK_SW | AIN_CV_CLOCK_CONTINUOUS)
#define AIN_SINGLE_CFG (AIN_CFG | AIN_SINGLE_ENDED | AIN_RANGE_10V)

// rtlinux differential is untested.  
// be sure to change ain_cl_size to 8 instead of 16
#define AIN_DIFF_CFG (AIN_CFG | AIN_START_TRIGGER_RISE | AIN_RANGE_10V | AIN_DIFFERENTIAL)
     

#define MAX_CL_SIZE		0x100	// maximum number of CL entries on MFx board
#define MAX_RESULT_QTY		MAX_CL_SIZE	// maximum number of samples is limited by maximum CL size = 0x100
#define PDAQ_ENABLE 1

#define ADCOVS 1

// check for null pointer.

u32
uei_bad_array_ptrs(s8 *name)
{
    if (daq->dacvolts == 0) {
        prf(&(ob->eofifo),
            "uei_bad_array_ptrs %s: daq->dacvolts == 0\n", name);
        do_error(ERR_UEI_BAD_ARRAY_PTRS);
        return 1;
    }
    return 0;
}

// call asap after daq mbuff is allocated,
// so there is no chance of dereferencing them before the are set.
void
uei_ptr_init(void)
{
    s32 i;
    // not ptrs, but these should be inited
    for (i=0; i<4; i++)
	daq->uei_board[i] = i;

    // these areas are equivalent
    daq->dienc = &(daq->m_dienc[0][0]);
    daq->dout_buf = &(daq->m_dout_buf[0]);
    daq->adc = &(daq->m_adc[0][0]);
    daq->dac = &(daq->m_dac[0][0]);
    daq->adcvolts = &(daq->m_adcvolts[0][0]);
    daq->dacvolts = &(daq->m_dacvolts[0][0]);
    daq->adcvoltsmean = &(daq->m_adcvoltsmean[0][0]);
    daq->adcvoltsmed = &(daq->m_adcvoltsmed[0][0]);
}

// This code handles multiple UEI boards.
// if the boards are installed in an order you don't like,
// there is an array called daq->uei_board to let you move the
// physical boards into logcal positions.

// phandle is physical handle
// lhandle is logical handle
// hardware commands go to physical handles.
// users refer to boards by logical handles,
// and retrieve their data using logical handles.

// do this once
// it doesn't care about virtual positions, it's just starting them.

// The functions in this file handle both PD2_MF_16_150_16L
// (multi-function) and PD2-AO-8/16 (analog output only).
// They take different command formats.
// We get the subdevids and then check them with PD_IS_MF() and PD_IS_AO().

void
uei_aio_init(void)
{
    s32 ret;
    static u32 adc_cl[MAX_RESULT_QTY];
    s32 boardi;
    s32 i;
    // TODO: delete extern s32 num_pd_boards;	// set by uei.
    // TODO: delete extern pd_board_t pd_board[PD_MAX_BOARDS];

    // TODO: delete daq->n_ueidaq_boards = num_pd_boards;
    daq->n_ueidaq_boards = PdGetNumberAdapters();

    // if no boards, no init.

    for (boardi = 0; boardi < daq->n_ueidaq_boards; boardi++) {
        s32 AIHandle;
        s32 AOHandle;
        s32 DIHandle;
        s32 DOHandle;

        s32 AO8Handle;
	Adapter_Info adapter_info;

	ret = _PdGetAdapterInfo(boardi, &adapter_info);
	daq->adapter_type[boardi] = adapter_info.atType;

        // TODO: delete daq->subdevid[phandle] = pd_board[phandle].PCI_Config.SubsystemID;

	// TODO: delete if (PD_IS_AO(daq->subdevid[phandle])) {
	if (daq->adapter_type[boardi] & atPD2AO) {
	    // TODO: delete ret = pd_ao32_reset(phandle);
	    AO8Handle = PdAcquireSubsystem(boardi, AnalogOut, DAQ_ACQUIRE);
	    daq->ao8_handle[boardi] = AO8Handle;
	    ret = _PdAOutReset(AO8Handle);
	    ret = _PdAO32Reset(AO8Handle);
	    continue;
	}

	// TODO: delete if (!PD_IS_MF(daq->subdevid[phandle]))
	if (!(daq->adapter_type[boardi] & atMF))
		continue;

	// it's an MF board.

        AIHandle = PdAcquireSubsystem(boardi, AnalogIn, DAQ_ACQUIRE);
        AOHandle = PdAcquireSubsystem(boardi, AnalogOut, DAQ_ACQUIRE);
        DIHandle = PdAcquireSubsystem(boardi, DigitalIn, DAQ_ACQUIRE);
        DOHandle = PdAcquireSubsystem(boardi, DigitalOut, DAQ_ACQUIRE);
	daq->ain_handle[boardi] = AIHandle;
	daq->aout_handle[boardi] = AOHandle;
	daq->din_handle[boardi] = DIHandle;
	daq->dout_handle[boardi] = DOHandle;

	// TODO: delete: ret = pd_ain_reset(phandle);
	ret = _PdAInReset(AIHandle);
	// TODO: delete: ret = pd_aout_reset(phandle);
	ret = _PdAOutReset(AOHandle);
	ret = _PdDInReset(DIHandle);
	ret = _PdDOutReset(DOHandle);

	// 1x oversample
	daq->ain_cl_size = ADCOVS*16;
	// treat each board the same, 16 samples.
	// daq->ain_cl_size = 16;

	// we can't init uei_board here because this init happens AFTER
	// imt2.cal has been read.  In the 99% case, there is just
	// one board, and daq->uei_board[0] is 0, and all is well.
	// if you have more than one board, set it up in the cal file.
	// phandle == daq->uei_board[lhandle]
	// if you want the 3rd uei board (in position 2) to stick
	// adc voltages in slots 0-15, do:
	// s ueiboard lhandle phandle, i.e.:
	// s uei_board 0 2

	// fill CL with "slow bit" flag
	// if > 1 boards, this is all the same.
	for (i = 0; i < daq->ain_cl_size; i++)
	    adc_cl[i] = (i % 16) | SLOW;
	    // adc_cl[i] = i | SLOW;

	// see AIN_CFG defined above
	// daq->ain_cfg = (AIN_CFG | AIN_SINGLE_ENDED | AIN_RANGE_5V);
	daq->ain_cfg = (AIN_CFG | AIN_SINGLE_ENDED | AIN_RANGE_10V);

	// TODO: delete: ret = pd_ain_set_config(phandle, daq->ain_cfg, 0, 0);
	ret = _PdAInSetCfg(AIHandle, daq->ain_cfg, 0, 0);
	// TODO: delete: ret = pd_ain_set_channel_list(phandle, daq->ain_cl_size, adc_cl);
	ret = _PdAInSetChList(AIHandle, daq->ain_cl_size, adc_cl);
	// TODO: delete: ret = pd_ain_set_enable_conversion(phandle, PDAQ_ENABLE);
	ret = _PdAInEnableConv(AIHandle, PDAQ_ENABLE);
	// TODO: delete: ret = pd_ain_sw_start_trigger(phandle);
	ret = _PdAInSwStartTrig(AIHandle);
	// TODO: delete: ret = pd_ain_sw_cl_start(phandle);
	ret = _PdAInSwClStart(AIHandle);
    }
}

// close does not care about virtual positions.

void
uei_aio_close(void)
{
    s32 boardi;
    s32 ret;

    for (boardi = 0; boardi < daq->n_ueidaq_boards; boardi++) {
	// TODO: delete if (PD_IS_MF(daq->subdevid[phandle])) {
	if (daq->adapter_type[boardi] & atMF) {
	        // TODO: delete ret = pd_ain_reset(phandle);
	        ret = _PdAInReset(daq->ain_handle[boardi]);
		// TODO: delete ret = pd_aout_reset(phandle);
		ret = _PdAOutReset(daq->aout_handle[boardi]);
		ret = _PdDInReset(daq->din_handle[boardi]);
		ret = _PdDOutReset(daq->dout_handle[boardi]);
	        ret = PdAcquireSubsystem(daq->ain_handle[boardi], AnalogIn, DAQ_RELEASE);
		ret = PdAcquireSubsystem(daq->aout_handle[boardi], AnalogOut, DAQ_RELEASE);
	        ret = PdAcquireSubsystem(daq->din_handle[boardi], DigitalIn, DAQ_RELEASE);
		ret = PdAcquireSubsystem(daq->dout_handle[boardi], DigitalOut, DAQ_RELEASE);
	}
	// TODO: delete if (PD_IS_AO(daq->subdevid[phandle])) {
	if (daq->adapter_type[boardi] & atPD2AO) {
	        // TODO: delete ret = pd_ao32_reset(phandle);
	        ret = _PdAO32Reset(daq->ao8_handle[boardi]);
		ret = PdAcquireSubsystem(daq->ao8_handle[boardi], AnalogOut, DAQ_RELEASE);
	}
    }
}

// raw_to_volts is working entirely with virtual positions,
// so no need to map.
static void
uei_raw_to_volts(u16 *raw, f64 *volts)
{
    s32 i;
    f64 inrange;

    // check for 5/10
    inrange = 10.0;
    if ((daq->ain_cfg & AIB_INPRANGE) == AIN_RANGE_5V) {
	    inrange = 5.0;
    }

    for (i=0; i<daq->ain_cl_size; i++) {
	volts[i] = raw[i] * inrange / 0x8000;
	// wrong:
	// if (daq->m_adcvolts[lhandle][i] > 10.0)
	// 	daq->m_adcvolts[lhandle][i] -= 20.0;
	// the single value 0x8000 gave 10v instead of -10v.
	// 32768 (0x8000) and greater must be -10v, not 10v
	if (raw[i] >= 0x8000)
	    volts[i] -= (2. * inrange);
    }
}

// the following channel gets crosstalk from the previous channel
// 15 wraps around to 0, i think.
static void
uei_ain_bias_comp(u32 board, f64 *volts) {
	s32 i;
	// s zero or too big...
	if (fabs(daq->ain_bias_comp[board]) > .3)
		daq->ain_bias_comp[board] = .018; // .018 looks ok emprically

	for (i=15; i>=0;i--) {
		u32 prev;
		prev = i-1;
		if (i == 0) prev = 15;
		volts[i] -= (volts[prev] * daq->ain_bias_comp[board]);
		// returning > +/-10 might make people nervous.
		volts[i] = dbracket(volts[i], -10.0, 10.0);
	}
}

// this reads a/d channels.
// like tachometers, for velocity.

// note that the UEI api calls its get function: pd_ain_get_samples().
// so it's describing each 16-bit value as a "sample."
// in the rest of our code, a sample is the set of data collected
// during a single iteration of the control loop.  take notice.

// read loops through logical handles, getting physical handles,
// and reads data into logical board slots (lhandle)
// the error message prints physical handle.

void
uei_ain_read(void)
{
    s32 boardi;
    s32 AIHandle;
    u16 m_ain[16][16];
    f64 m_ainvolts[16][16];
    u16 *ain;
    f64 *ainvolts;
    u32 i;

    u32 len;
    int ret;

    if (uei_bad_array_ptrs("uei_ain_read")) return;

    if (daq->n_ueidaq_boards < 1) return;

    ain = &(m_ain[0][0]);
    ainvolts = &(m_ainvolts[0][0]);

    // daq->uei_board can swap the board postions.
    // if daq->uei_board[0] = 1,
    // then data from the 0th board is going to channels 16-31.
    for (boardi = 0; boardi < daq->n_ueidaq_boards; boardi++) {
	// TODO: delete if (!PD_IS_MF(daq->subdevid[phandle]))
	if (!(daq->adapter_type[boardi] & atMF))
	  continue;

	AIHandle = daq->ain_handle[boardi];

	daq->ain_got_samples = 0;
	// daq->ain_ret = pd_ain_get_samples(phandle, daq->ain_cl_size,
		// daq->m_adc[lhandle], &daq->ain_got_samples);
	/* TODO: delete daq->ain_ret = pd_ain_get_samples(phandle, daq->ain_cl_size,
	   ain, &daq->ain_got_samples); */
	daq->ain_ret = _PdAInGetSamples(AIHandle, daq->ain_cl_size,
	   ain, &daq->ain_got_samples);

	/* FVV todo, we could make this dpr() style debug code */
	if (daq->ain_got_samples != daq->ain_cl_size) {
	  dpr(0,
	      "pd_ain_get_samples: failed, samples != cl_size, "
	      "boardi = %d, wanted = %d, ret = %d, got = %d\n",
	      boardi, ob->samplenum, daq->ain_cl_size,
	      daq->ain_ret, daq->ain_got_samples);
	  do_error(ERR_UEI_NSAMPLES);
	}

	if (daq->ain_ret <= 0) {
	  dpr(0,
	      "pd_ain_get_samples: failed, ret <= 0, "
	      "boardi = %d count = %d, wanted = %d, ret = %d, got = %d\n",
	      boardi, ob->samplenum, daq->ain_cl_size,
	      daq->ain_ret, daq->ain_got_samples);
	  do_error(ERR_UEI_RET);
	}

	uei_raw_to_volts(ain, ainvolts);

	// crosstalk compensation
	uei_ain_bias_comp(boardi, ainvolts);

	// simple bubble sort
	// bubble sort is inefficient on a long list,
	// but this is typically 5 items.
	for (i = 0; i < 16; i++) {
	    u32 j;
	    f64 swapped;
	    f64 aintmp[16];
	    f64 tmp;
	    f64 ainsum;

	    swapped = 1;
	    ainsum = 0.0;

	    daq->m_adc[boardi][i] = m_ain[0][i];
	    daq->m_adcvolts[boardi][i] = m_ainvolts[0][i];

	    len = ADCOVS;

	    for (j = 0; j < len; j++) {
		// note the subscripts
		aintmp[j] = m_ainvolts[j][i];	// copy
		ainsum += m_ainvolts[j][i];	// sum
	    }
	    daq->m_adcvoltsmean[boardi][i] = ainsum / len;

	    // median sort
	    while (swapped) {
		swapped = 0;
		for (j=0; j<len-1; j++) {
		    if (aintmp[j] > aintmp[j+1]) {
			// swap, using a tmp
			tmp = aintmp[j];
			aintmp[j] = aintmp[j+1];
			aintmp[j+1] = tmp;

			// mark as swapped
			swapped = 1;
		    }
		}
	    } // while swapped

	    // median
	    daq->m_adcvoltsmed[boardi][i] = aintmp[len/2];

	    // use median as voltage
	    daq->m_adcvoltsmed[boardi][i] = aintmp[len/2];
	    daq->m_adcvolts[boardi][i] = aintmp[len/2];
	} // for each of 16 voltages


	// program powerdaq for a new burst
	// TODO: delete daq->ain_ret = pd_ain_sw_cl_start(phandle);	// starts clock
	ret = _PdAInSwClStart(AIHandle);
    } // for each board/phandle
}

// pd_aout_put_value:
// The analog outputs have a fixed output range of +/- 10V.
// Data representation is straight binary.
// To convert voltage into binary codes use the following formula.
//
// HexValue = ((Voltage + 10.0V) / 20.0) * 0xFFF
//
// so...
// -10 V = 0x0,
//   0 V = 0x7FF
//  10 V = 0xFFF
//
// The two Hex values for Aout channel 0 and 1 respectively
// can be combined to write to the analog output as follows:
//
// Value_To_Write = (HexValue1 << 12) OR (HexValue0)

// uei_aout_write V1, V2 - f64 volts.

// this tells the motors to move the arm.

// V1 is the shoulder motor, V2 the elbow.

// write writes to the 1st virtual board.

void
uei_aout_write(f64 V1, f64 V2)
{
    s32 ret;
    u32 out1, out2;
    u32 outdata;
    s32 boardi;
    // LATER
    // u32 yank_fault;

    if (uei_bad_array_ptrs("uei_aout_write")) return;

    if (daq->n_ueidaq_boards < 1) return;

    // must be on 1st board.
    boardi = 0;

    // TODO: delete if (!PD_IS_MF(daq->subdevid[phandle]))
    if (!(daq->adapter_type[boardi] & atMF))
	return;

    // move this to control.
    V1 = dbracket(V1, -10.0, 10.0);
    V2 = dbracket(V2, -10.0, 10.0);
    if (!finite(V1) || !finite(V2)) {
	V1 = V2 = 0.0;
    }

#ifdef LATER
    yank_fault = 0;
    // check yank.  if too great, clamp.
    if ( (fabs(V1 - daq->dacvolts[0]) > 1.0) 
      || (fabs(V2 - daq->dacvolts[1]) > 1.0) ) {
	se v;

	// perhaps other error reporting here
	yank_fault = 1;
	v.s = V1;
	v.e = V2;
	v = preserve_orientation(v, 1.0);
	V1 = v.s;
	V2 = v.e;
    }
#endif // LATER

    out1 = ((V1 + 10.0) / 20.0) * 0xFFF;
    out2 = ((V2 + 10.0) / 20.0) * 0xFFF;

    outdata = ((out1 << 12) | (out2)) & 0xFFFFFF;

    daq->dac[0] = out1;
    daq->dac[1] = out2;
    daq->dacvolts[0] = V1;
    daq->dacvolts[1] = V2;

    // TODO: delete ret = pd_aout_put_value(phandle, outdata);
    ret = _PdAOutPutValue(daq->aout_handle[boardi], outdata);
    // prf(ob->eofifo, "pd_aout_put_value = %x, V1 %d, V2 %d, ret = %d\n",
    //	outdata, (s32)(mV1*1000), (s32)(mV2*1000), ret);

}

// the PD2-AO and PD2-MF have different AOUT functions.
// See chapter 4 of the PowerDAQ Programmer's Manual.

// PD2-AO uses straight binary data encoding where
// 0x0000 = -10V,
// 0x7fff = 0V and
// 0xffff = +10V 
//
// (the AO uses 16 bits in a u16 to send one voltage,
// the MF uses 24 bits in a u32 to send two different voltages,
// see above.)

void
uei_aout32_write(s32 boardi, u16 channel, f64 voltage)
{
	u16 value;
	s32 ret;

	if (daq->n_ueidaq_boards < 1) return;

	// TODO: delete if (!PD_IS_AO(daq->subdevid[phandle]))
	if (!(daq->adapter_type[boardi] & atPD2AO))
		return;

	voltage = dbracket(voltage, -10.0, 10.0);
	value = ((voltage + 10.0) / 20.0) * 0xFFFF;
	// TODO: delete ret = pd_ao32_write(phandle, channel, value);
	ret = _PdAO32Write(daq->ao8_handle[boardi], channel, value);
}

// uei_aout32_test(void) {}
// is deprecated, use robot_motor_test_volts.


// uei_dio_scan1 performs 1 di scan.
// we may want to oversample to cancel noise,
// see uei_dio_scan below.

// read 16 bits into each of 2 vars, using 3 reads:
// HHHH HHHM MMMM MMML

// the two encoders expect writes to the lower three bits of the byte,
// so the write mask is 0x0707.  This is coordinated with
// DOUT0 and DOUT1 on the junction box, which are at: 0x8080.

// dio/din reports encoder values that are tranformed to x,y position.

// dio_scan reads from the 1st virtual board.

// dio_scan chats with the encoder to get its data.
// this code should really split between here and sensact.c,
// but it's a translation of existing code, so it was easier to just
// leave it like this.

static void
uei_dio_scan1(void)
{
    u32 pdwValue;
    u32 lowbitsA, mediumbitsA, highbitsA;
    u32 lowbitsB, mediumbitsB, highbitsB;
    s32 boardi;

    if (uei_bad_array_ptrs("uei_dio_scan1")) return;

    if (daq->n_ueidaq_boards < 1) return;

    // someday, we want to be able to do dio_scan on 2nd board, but not yet.
    boardi = 0;

    // TODO: delete if (!PD_IS_MF(daq->subdevid[phandle]))
    if (!(daq->adapter_type[boardi] & atMF))
	return;

    // static unsigned long hold_val, release_val, lower_val, medium_val, high_val, val;

    // Send hold command high for dout 0 8
    uei_dout_write_masked(boardi, 0x101, 0x0707);

    // Read the lower 8 bits -- send command high for dout 0 1 8 9
    uei_dout_write_masked(boardi, 0x303, 0x0707);
    uei_din_read(boardi, &pdwValue);

    lowbitsA = pdwValue & 0xFF;
    lowbitsB = (pdwValue & 0xFF00) >> 8;

    // Read medium 8 bits -- send command high for dout 0 2 8 10
    uei_dout_write_masked(boardi, 0x505, 0x0707);
    uei_din_read(boardi, &pdwValue);

    mediumbitsA = pdwValue & 0xFF;
    mediumbitsB = (pdwValue & 0xFF00) >> 8;

    // Read high 8 bits -- send command high for dout 0 1 2 8 9 10
    uei_dout_write_masked(boardi, 0x707, 0x0707);
    uei_din_read(boardi, &pdwValue);

    // note that we had to get rid of the highest bit
    highbitsA = pdwValue & 0x7F;
    highbitsB = (pdwValue & 0x7F00) >> 8;

    // these were called rd[] in the qnx ver...
    daq->dienc[0] = (highbitsA << 9) | (mediumbitsA << 1) | (lowbitsA >> 7);
    daq->dienc[1] = (highbitsB << 9) | (mediumbitsB << 1) | (lowbitsB >> 7);

    // status and flags
    daq->distat[0] = 0;
    daq->distat[0] |= highbitsA & 0x80;
    daq->distat[0] |= lowbitsA & 0x07;
    daq->distat[1] = 0;
    daq->distat[1] |= highbitsB & 0x80;
    daq->distat[1] |= lowbitsB & 0x07;

    // calculate encoder vels
    daq->dienc_vel[0] = daq->dienc[0] - daq->prev_dienc[0];
    while (daq->dienc_vel[0] > (1<<15))
	daq->dienc_vel[0] -= (1<<16);
    while (daq->dienc_vel[0] < -(1<<15))
	daq->dienc_vel[0] += (1<<16);
    daq->prev_dienc[0] = daq->dienc[0];
    daq->dienc_accel[0] = daq->dienc_vel[0] - daq->prev_dienc_vel[0];
    daq->prev_dienc_vel[0] = daq->dienc_vel[0];

    daq->dienc_vel[1] = daq->dienc[1] - daq->prev_dienc[1];
    while (daq->dienc_vel[1] > (1<<15))
	daq->dienc_vel[1] -= (1<<16);
    while (daq->dienc_vel[1] < -(1<<15))
	daq->dienc_vel[1] += (1<<16);
    daq->prev_dienc[1] = daq->dienc[1];
    daq->dienc_accel[1] = daq->dienc_vel[1] - daq->prev_dienc_vel[1];
    daq->prev_dienc_vel[1] = daq->dienc_vel[1];

    // release buffer to follow data
    uei_dout_write_masked(boardi, 0, 0x0707);
}

// oversample di, choosing median value of len, to cancel noise.
// daq->dienc[] may have a noisy value while uei_dio_scan is running,
// but this should not be a problem.

void
uei_dio_scan(void)
{
    struct {
	u16 enc[2];	// raw vals
	u32 sum;	// sort key
    } di[16], tmp;
    u32 i;

    u32 swapped = 1;	// for bubble sort
    u32 len;		// oversample size, array length
    u32 median;		// median

    if (uei_bad_array_ptrs("uei_dio_scan")) return;

    if (daq->n_ueidaq_boards < 1) return;

    // typical values of len are 1 or 5.
    len = daq->diovs;

    // if out of range, just do once.
    if (len <= 1 || len > 15) {
	uei_dio_scan1();
	return;
    }

    // read len times.  caculate sum of encode values each time.
    for (i=0; i<len; i++) {
	u32 d0, d1;

	uei_dio_scan1();
	d0 = di[i].enc[0] = daq->dienc[0];
	d1 = di[i].enc[1] = daq->dienc[1];
	di[i].sum = d0 + d1;
    }

    // simple bubble sort
    // di[i].sum is the sort key.
    // bubble sort is inefficient on a long list,
    // but this is typically 5 items.
    while (swapped) {
	swapped = 0;
	for (i=0; i<len-1; i++) {
	    if (di[i].sum > di[i+1].sum) {
		// swap, using a tmp
		tmp = di[i];
		di[i] = di[i+1];
		di[i+1] = tmp;

		// mark as swapped
		swapped = 1;
	    }
	}
    }

    // at this point, di[median] has the median values.
    // typically, len == 5 and median == 2;
    median = len / 2;
    daq->dienc[0] = di[median].enc[0];
    daq->dienc[1] = di[median].enc[1];
}

// keep a record of what has been written, and only change
// what's masked.  this way you can write to encoders and to other 
// lines too

s32
uei_dout_write_masked(s32 boardi, u32 val, u32 mask)
{
    u32 masked_val;
    u16 prev_val;

    if (uei_bad_array_ptrs("uei_dout_write_masked")) return -1;

    if (daq->n_ueidaq_boards < 1) return -1;

    // TODO: delete if (!PD_IS_MF(daq->subdevid[phandle]))
    if (!(daq->adapter_type[boardi] & atMF))
	return 0;

    uei_dout01(boardi);

    prev_val = daq->dout_buf[boardi];

    // the new val is the
    // the old unmasked bits plus
    // the new masked bits.
    masked_val = (prev_val & ~mask) | (val & mask);

    daq->dout_buf[boardi] = masked_val;

    // TODO: delete return pd_dout_write_outputs(phandle, masked_val);
    return _PdDOutWrite(daq->dout_handle[boardi], masked_val);
}

void
uei_dout01(s32 boardi) {
	 // bit 7
	 if (daq->dout0)
		 daq->dout_buf[boardi] |= 0x0080;
	 else
		 daq->dout_buf[boardi] &= ~0x0080;

	 // bit 15
	 if (daq->dout1)
		 daq->dout_buf[boardi] |= 0x8000;
	 else
		 daq->dout_buf[boardi] &= ~0x8000;
}
//
// heartbeat bit toggle

void
uei_dout_heartbeat(u32 boardi) {
    daq->dout_buf[boardi] ^= 0x4000;
}

void
uei_din_read(s32 boardi, u32 *val)
{
    // TODO: delete if (!PD_IS_MF(daq->subdevid[phandle]))
    if (!(daq->adapter_type[boardi] & atMF))
	return;
    // TODO: delete return pd_din_read_inputs(phandle, val);
    _PdDInRead(daq->din_handle[boardi], val);
}

void
uei_dio_write_led(u16 led) {
  
    s32 handle;

    handle = daq->dout_handle[1];
  _PdDOutWrite(handle, led);

}

