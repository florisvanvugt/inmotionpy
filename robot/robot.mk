
prefix := $(shell /usr/realtime/bin/xeno-config --prefix)

CWD = $(shell pwd)
UEI_INC = powerdaq/include

CC = $(shell /usr/realtime/bin/xeno-config --cc)

CFLAGS = $(shell /usr/realtime/bin/xeno-config --xeno-cflags) -I/opt/imt/robot/$(UEI_INC) -g -O0
LDFLAGS = $(shell /usr/realtime/bin/xeno-config --xeno-ldflags) -lnative -lpowerdaq32 -lrtdm

ARCH = i386
CC = gcc

