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
INCS = robdecls.h rtl_inc.h ruser.h uei_inc.h userfn.h pipes.h

SRCS = main.c uei.c fifo.c math.c \
	slot.c write.c isaft.c \
	pl_sensact.c pl_ulog.c pl_uslot.c \
	dyncomp.c

OBJS = main.o uei.o fifo.o math.o \
	slot.o write.o isaft.o \
	pl_sensact.o pl_ulog.o pl_uslot.o \
	dyncomp.o


default: all

doc:
	doxygen doxygen.conf

run: all
	sudo ./robot

all: robot field_addresses.txt

shm:	shm.c cmds.h $(INCS)
	$(CC) -Wall $(CFLAGS) -o shm shm.c

field_addresses.txt: probe_addresses
	./probe_addresses > field_addresses.txt

probe_addresses: probe_addresses.c
	 $(CC) -Wall $(CFLAGS) -o probe_addresses probe_addresses.c

probe_addresses.c: robdecls.h parse_robdecls.py
	python3 parse_robdecls.py > probe_addresses.c

$(OBJS) : $(INCS)

robot: $(OBJS) $(INCS)
	$(CC) $(OBJS) $(LDFLAGS) -o robot $(MATHLIBS)

clean:
	rm -f robot pwrdaq*.o *\~ *\# $(OBJS) probe_addresses.c probe_addresses field_addresses.txt

tags: $(INCS) $(SRCS)
	ctags $(INCS) $(SRCS)

doxygen: html/index.html
	xdg-open html/index.html

html/index.html: interface.py shm.py
	doxygen doxygen.conf

LINCS=-I/usr/xenomai/include\
	-I/usr/i386-glibc21-linux/include

