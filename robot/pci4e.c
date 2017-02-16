// pci4e.c - US Digital PCI incremental encoder interface card
// http://www.usdigital.com/products/pci4e/
// part of the robot.o Linux Kernel Module

// InMotion2 robot system software for RTLinux

// Copyright 2005 Interactive Motion Technologies, Inc.
// Cambridge, MA, USA
// http://www.interactive-motion.com
// All rights reserved

#include "rtl_inc.h"
#include "ruser.h"
#include "robdecls.h"

// TODO: delete #include "asm/io.h"

// TODO: possibly delete #include <linux/pci.h>

// TODO: delete #define VENDOR_ID 0x1892
// TODO: delete #define DEVICE_ID 0x5747

#include "pci4eHelper.h"
#define REG4E(i,REG) (((i) * 8) + (REG))

#define NENC 4

// this driver is coded to work for only one counter card.

// the pci4e does i/o through a set of 8 registers for each of 4 encoders.
// the misc register is special purpose, and is different for each encoder,
// see the manual.

// note that this driver doesn't read and write any reg structures directly,
// it just uses C to manage the addresses to be passed to readl/writel.

#if 0 // delete
struct pci4e_reg {
    u32 preset;	// 0
    u32 output;	// 1
    u32 match;	// 2
    u32 control; // 3
    u32 status;	// 4
    u32 reset_channel; // 5
    u32 transfer_preset; // 6
    u32 misc;	// 7
};

#define NENC 4

// one for each encoder
struct pci4e_regs {
	struct pci4e_reg chan[NENC];
};
#endif // delete

// TODO: delete static void pci4e_write(u32, void *);
// TODO: delete static u32 pci4e_read(void *);

static void
// TODO: delete pci4e_set_modes(void) {
pci4e_set_modes(s16 boardn) {
    u32 i;

#if 0 // delete
    struct pci4e_regs *remap;

    if (!rob->pci4e.have)
	return;

    if (!rob->pci4e.remap)
	return;

    remap = (struct pci4e_regs *)rob->pci4e.remap;
#endif // delete

    if (rob->pci4e.limit == 0)
	// default counts per rev 0-20479
	rob->pci4e.limit = 1 << 24;

    for (i=0; i<NENC; i++) {
	// see manual
	// control mode 7C000 ==>
	// 20 index will reset/preset accumulator (no)
	//
	// 19 swap a/b (controls direction of count) (no)
	// 18 enable counter (yes)
	// 16/17 modulo n counter (yes)
	//
	// 14/15 x4 mode (yes)
	// TODO: delete pci4e_write(0x7C000,&remap->chan[i].control);
	PCI4E_WriteRegister(boardn, REG4E(i, CONTROL_REGISTER), 0x7C000);
	// TODO: delete pci4e_write((u32)rob->pci4e.limit-1, &remap->chan[i].preset);
	PCI4E_WriteRegister(boardn, REG4E(i, PRESET_REGISTER), rob->pci4e.limit - 1);
    }
}

/* TODO: restore this if correct  static */void
// TODO: delete pci4e_init_one (struct pci_dev *dev)
pci4e_init_one (s16 boardn)
{
    if (!rob->pci4e.have)
	return;	// no board

#if 0 // delete
    if (!dev)
	return;	// no dev

    if (rob->pci4e.remap)
	return; // already did this

    rob->pci4e.dev = dev;
    // TODO: make this line work rob->pci4e.bar = pci_resource_start(dev, 0);
    // TODO: make this line work rob->pci4e.len = pci_resource_len(dev, 0);
    // TODO: make this line work rob->pci4e.remap = ioremap(rob->pci4e.bar, rob->pci4e.len);

    // dpr(0, "found pci4e bar 0x%x, len 0x%x, remap 0x%x\n",
	    // rob->pci4e.bar, rob->pci4e.len, remap);
#endif // delete

    pci4e_set_modes(boardn);
}

// called by main.c:do_init()

void
pci4e_init(void)
{
    // TODO: delete struct pci_dev *dev;
    u16 i;
    s16 nboards;
    s32 ret;

    if (!rob->pci4e.have)
	return;

    ret = PCI4E_Initialize(&nboards);

#if 0 // delete
    // find the pci4e, now, just 1 dev
    dev = NULL;
    /* TODO: make these lines 
    work while ((dev = pci_find_device(VENDOR_ID, DEVICE_ID, dev))) {
	if (!dev) break;
	pci4e_init_one(dev);

	// remove this break when you hack the driver
	// to handle more than one card.
	break;
    }
    */
#endif // delete

    for (i=0; i<nboards; i++) {
	pci4e_init_one(i);
	// remove this break when you hack the driver
	// to handle more than one card.
	break;
    }
}

#if 0 // delete
static u32
pci4e_read(void *port)
{
	u32 val;
	val = 0;  // TODO: delete this line
	// TODO: make this line work val = readl((u32)port);
	return val;
}

static void
pci4e_write(u32 val, void *port)
{
	// TODO: make this line work writel(val, (u32)port);
	return;
}
#endif // delete

// called by main:cleanup_devices()
void
pci4e_close(void)
{
    if (!rob->pci4e.have)
	    return;

    if (rob->pci4e.remap)
	// TODO: make this line work iounmap(rob->pci4e.remap);
    rob->pci4e.remap = NULL;
}

// read one encoder port.
// chan is a channel 0-3 (corresponds to 1-4 on the card).
// raw is 24 bits of u32
// returns f64 scaled, between -limit/2 and limit/2

static f64
pci4e_read_ch(s16 boardn, u32 i)
{
    // can't use s32, the driver dictates "long"
    long raw;
    f64 pos;
    // TODO: delete struct pci4e_regs *remap;

    if (!rob->pci4e.have)
	return (-1.0);

#if 0 // delete
    if (!rob->pci4e.remap)
	return (-1.0);

    remap = (struct pci4e_regs *)rob->pci4e.remap;

    pci4e_write(0, &remap->chan[i].output);
    raw = pci4e_read(&remap->chan[i].output);
#endif // delete

    PCI4E_GetCount(boardn, i, &raw);

    // raw (normalized) position
    rob->pci4e.raw[i] = (u32)raw & 0xFFFFFF;

    if (rob->pci4e.limit == 0)
	rob->pci4e.limit = 1 << 24;
    // scaled position
    // pos = (f64)raw * rob->pci4e.scale;
    // pos = (f64)raw * 2.0 * M_PI / rob->pci4e.limit;

    // scale pos.  if it's big, make it negative.
    pos = raw * rob->pci4e.scale;
    if (raw > (rob->pci4e.limit/2)) {
	pos = ((f64)raw - rob->pci4e.limit) * rob->pci4e.scale;
    }

    return pos; 
}

// reset the four counters to zero.
// called when you set rob->pci4e.zero to other than 0.
// also resets the limit register to pcienc_limit.

void
pci4e_reset_all_ctrs(void)
{
    u32 i;
    // TODO: delete struct pci4e_regs *remap;
    s16 boardn;

    boardn = 0;

    if (!rob->pci4e.have)
	    return;

#if 0 // delete
    if (!rob->pci4e.remap)
	return;

    remap = (struct pci4e_regs *)rob->pci4e.remap;
#endif // delete

    for (i=0; i<NENC; i++) {
	// TODO: delete pci4e_write(0, &remap->chan[i].reset_channel);       // reset counter
	PCI4E_WriteRegister(boardn, REG4E(i, RESET_CHANNEL_REGISTER), 0);
    }
    pci4e_set_modes(boardn);

}

void
pci4e_encoder_read(void)
{
    u32 i;
    s16 boardn;

    boardn = 0;

    if (!rob->pci4e.have)
	return;

    if (rob->pci4e.scale < 0.00000001)
	rob->pci4e.scale = 1.0;
    for (i=0;i<NENC;i++) {
	rob->pci4e.enc[i] = pci4e_read_ch(boardn, i);
	// these are now identical
	rob->pci4e.lenc[i] = rob->pci4e.enc[i];
    }
}

