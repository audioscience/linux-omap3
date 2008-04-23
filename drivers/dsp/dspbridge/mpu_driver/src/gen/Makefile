#
# dspbridge/src/gen/Makefile
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
#  24-Feb-2003  vp: removed gh.c file from sources.
#  18-Oct-2002  sb: Created Initial version
#  ----------------------------------------------------------------------------

PROJROOT=../..

include $(PROJROOT)/make/start.mk

# Do not change above "include" line(s)

# Arguments to tools, will move to make system once finalized.

EXTRA_CFLAGS         =
CDEFS          = __KERNEL__
ifeq ($(BUILD),debug) # Additional flags for debug builds
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

TARGETNAME  = libgen


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
gb.c \
gt.c \
gs.c \
gh.c \
_gt_para.c \
uuidutil.c

# Search path for include files

INCLUDES    = \
	$(KRNLSRC)/include \
	$(PROJROOT)/inc \
	$(PROJROOT)/inc/linux

# Libraries needed for linking.

ST_LIBS        =
SH_LIBS        =

# Search path for library (and linker command) files.
# Current folder and target folder are included by default.

LIBINCLUDES =


# Do not change below "include" line(s)

include $(PROJROOT)/make/build.mk
