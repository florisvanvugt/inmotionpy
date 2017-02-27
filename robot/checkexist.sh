# check for the existence of components
# also check for root access.
# (prerequisites for running robot code)

# this should be called at program startup time from user mode apps.
# it's called by go, which is called by shm.tcl:start_lkm.
# if you're not calling go, then call this yourself.

# InMotion2 robot system software for RTLinux

# Copyright 2003-2005 Interactive Motion Technologies, Inc.
# Cambridge, MA, USA
# http://www.interactive-motion.com
# All rights reserved

CROB_HOME=.

if [ -z "$CROB_HOME" ]; then
        echo "CROB_HOME is not set, imt_config/imt.rc must be sourced"
	exit 1
fi

MSG=$CROB_HOME/tools/zenity_wrap

# TODO: change our uname -a to something to shorter to make Andy happy
#uname -a | grep -q interactive-motion
#if [ $? != 0 ]; then
#        echo "The Linux kernel currently booted is not correct for Xenomai."
#	uname -a
#	exit 1
#fi

CROB=$CROB_HOME
OIR=/opt/imt/robot

# /home/imt/crob/robot.o: robot code
# /dev/mbuff: shared memory device
# /dev/rtf0: real time fifo device
# /usr/src/linux/rtlinux: rtlinux kernel module code
#	 (implies that linux is there too)
# /home/imt/daq/powerdaq: UEI daq code
# /usr/local/include/ueidaq UEI: daq includes


flist="
	$CROB/robot
	$CROB/imt2.cal
	/usr/realtime/modules/xeno_hal.ko
	$OIR/lib/imt.gif
	/lib/modules/`uname -r`/kernel/drivers/misc/pwrdaq.ko
"
# Removed from flist (fvv June 2014):
#       	$OIR/pci4e/pci4e.ko  (because we don't have PCI4e)
#	$IMT_CONFIG/robots/`cat $IMT_CONFIG/current_robot`/imt2.cal



# TODO: delete 	/dev/mbuff
# TODO: delete 	/dev/rtf0
# TODO: delete 	/usr/src/linux/rtlinux
# TODO: delete 	/home/imt/daq/powerdaq
# TODO: delete 	/usr/local/include/ueidaq

for i in $flist; do
	if [ ! -e $i ]; then
		echo checkexist: $i not found
		exit 1
	fi
done

# is there enough disk space?
# check therapist, log, and var

# the df/sed/cut call scrapes the Available blocks data from the output

# Filesystem           1K-blocks      Used Available Use% Mounted on
# /dev/sda2             24027656   7464332  15342788  33% /home

for dir in $LOG_HOME
do
    # if LOG_HOME doesn't exist, continue
    if [ ! -e $dir ]; then
	continue
    fi

    size=`df $dir | sed -n '2s/  */ /gp' | cut -d' ' -f4`
    # 100 mb exit
    if [ $size -le 100000 ]; then
	$MSG "checkexist: $dir filesystem is full ($size blocks), please delete files." --error
	exit 1
    fi

    # 1 Gb warn
    if [ $size -le 1000000 ]; then
	$MSG "checkexist: $dir filesystem is full ($size blocks), please delete files." --warning
    fi
done

dir=/var
size=`df $dir | sed -n '2s/  */ /gp' | cut -d' ' -f4`
# 50Mb exit
if [ $size -le 50000 ]; then
    $MSG "checkexist: $dir filesystem is full ($size blocks), please delete files." --error
    exit 1
fi

# 100 Mb warn
if [ $size -le 100000 ]; then
    $MSG "checkexist: $dir filesystem is full ($size blocks), please delete files." --warning
fi

exit 0
