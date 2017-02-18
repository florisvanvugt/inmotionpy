# Makefile for RTLinux Inmotion2 robot library

# InMotion2 robot system software for RTLinux

# Copyright 2003-2005 Interactive Motion Technologies, Inc.
# Cambridge, MA, USA
# http://www.interactive-motion.com
# All rights reserved

# you should read and understand ./robot.mk
# before changing this Makefile

include robot.mk

# these are necessary to deal with trig functions (-lm)
# and hrtime math (libgcc.a)

MATHLIBS = -L/usr/lib -lm -lc `$(CC) -print-file-name=libgcc.a`
INCS = robdecls.h rtl_inc.h ruser.h uei_inc.h userfn.h pipes.h userdecls.h cmds.h

SRCS = main.c uei.c fifo.c math.c \
	slot.c write.c isaft.c pc7266.c pci4e.c\
	pl_sensact.c pl_ulog.c pl_uslot.c \
	pci4eHelperIMT.c dyncomp.c

OBJS = main.o uei.o fifo.o math.o \
	slot.o write.o isaft.o pc7266.o pci4e.o\
	pl_sensact.o pl_ulog.o pl_uslot.o \
	pci4eHelperIMT.o dyncomp.o

export CROB_HOME:=$(shell pwd)
export IMT_CONFIG:=$(shell pwd)


# default:

default: all

all: robot shm rwarp 

$(OBJS) : $(INCS)

robot: $(OBJS) $(INCS)
	$(CC) $(LDFLAGS) $(OBJS) -o robot $(MATHLIBS)
#	$(LD) $(LDFLAGS) -m elf_i386 $(OBJS) -o robot $(MATHLIBS)

cmds.h: cmdlist.tcl
	tclsh mkcmds.tcl > cmds.out
	mv cmds.out cmds.h

shm:	cmds.h shm.c $(INCS)
	$(CC) -Wall $(CFLAGS) -o shm shm.c

# FVV ADDED (CAN BE REMOVED LATER)
findad:	cmds.h shm-findad.c $(INCS)
	$(CC) -Wall $(CFLAGS) -o findad shm-findad.c
# END-FVV ADDED

rwarp:	rwarp.c $(INCS)
	$(CC) -Wall $(CFLAGS) -o rwarp rwarp.c -L/usr/X11R6/lib -lX11 -lXtst



clean:
	rm -f robot cmds.h cmds.out shm.o shm rwarp rwarp.o pwrdaq*.o *\~ *\# $(OBJS) 

# clean, save, recompile
#save: clean
#	cd ..; tar zcvf save/crob.tgz.`date +%m%d` crob
#	make

tags: $(INCS) $(SRCS)
	ctags $(INCS) $(SRCS)

LINCS=-I/usr/realtime/include\
	-I/usr/i386-glibc21-linux/include

# not working yet
lint:
	splint -D__KERNEL__ $(LINCS) $(SRCS)
	splint shm.c

#INSTALL_DIR = ~/crob
# INSTALL_COMPONENTS = robot checkexist ecat examples FILES go gp imt2.cal lislot loghead nta.tcl reloadmodules.sh robot rwarp shm stop unloadmodules.sh vex wrta.tcl xygp #cgp 

#install:
#	mkdir -p $(INSTALL_DIR)
#	for i in $(INSTALL_COMPONENTS); do install $$i $(INSTALL_DIR); done
#	make -C notes install
#	make -C tools install
#	make -C varlist install


