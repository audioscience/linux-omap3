#
# dspbridge/src/rmgr/Makefile
#
# DSP-BIOS Bridge driver support functions for TI OMAP processors.
#
# Copyright (C) 2007 Texas Instruments, Inc.
#
# This program is free software; you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the
# Free Software Foundation version 2.
#
# This program is distributed .as is. WITHOUT ANY WARRANTY of any kind,
# whether express or implied; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Public License for more details.
#

#  ----------------------------------------------------------------------------
#  Revision History
#
#
#      REF=ORG   19, September 2002 Raghunandan Nagarajarao
#      Original version.
#  ----------------------------------------------------------------------------

PROJROOT=../..

include $(PROJROOT)/make/start.mk

# Do not change above "include" line(s)

# Arguments to tools, will move to make system once finalized.

EXTRA_CFLAGS         =
ifeq ($(PROCFAMILY),OMAP_2430)
CDEFS          = __KERNEL__ MODULE _TI_ _DB_TIOMAP
else
ifeq ($(PROCFAMILY),OMAP_3430)
CDEFS          = __KERNEL__ MODULE _TI_ _DB_TIOMAP
else
CDEFS          = __KERNEL__ MODULE _TI_ CHNL_DDMA CHNL_DDZC _DB_TIOMAP
endif
endif

ifeq ($(BUILD),deb) # Additional flags for debug builds
CDEFS          += DEBUG GT_TRACE
endif

EXEC_ARGS      =
ST_LIB_ARGS    =
SH_LIB_ARGS    =

# Define this macro if target runs in kernel mode
__KERNEL__ = 1

# Target name and extension
# static library        (ST_LIB): filename.a
# shared library soname (SH_LIB): filename.so.maj_ver.min_ver
# executable            (EXEC)  : filename.out

TARGETNAME  = librmgr


# TARGETTYPE must be EXEC, ST_LIB or SH_LIB in upper case.

TARGETTYPE  = MODULE

# install directory relative to the HOSTTARGET directory
# HOSTRELEASE =

# install directory relative to the root filesystem
# ROOTFSRELEASE =

# For shared object library, soname is filename.so.maj_ver

SH_SONAME =

# Folders in which gmake will run before building current target

SUBMODULES  = \

# Filename must not begin with '.', '/' or '\'

SOURCES     = \
common/dbdcd.c \
common/disp.c \
common/drv.c \
common/mgr.c \
common/node.c \
common/proc.c \
common/pwr.c \
common/rmm.c \
common/strm.c \
common/dspdrv.c \
nldr/nldr.c \
common/drv_interface.c \

# Search path for include files
# TODO -- Make search path to include HAL according to the Chip type

INCLUDES    = \
    $(KRNLSRC)/include \
    $(PROJROOT)/inc \
    $(PROJROOT)/inc/linux \
    common \
    $(PROJROOT)/src/hal/common/inc \
    $(PROJROOT)/src/hal/common/mbox \
    $(PROJROOT)/src/hal/common/mmu \
    $(PROJROOT)/src/hal/common/ocp
#    $(PROJROOT)/src/iva/wrapper

# Libraries needed for linking.

ST_LIBS        =
SH_LIBS        =

# Search path for library (and linker command) files.
# Current folder and target folder are included by default.

LIBINCLUDES =


# Do not change below "include" line(s)

include $(PROJROOT)/make/build.mk
