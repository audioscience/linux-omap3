#
# dspbridge/src/wmd/Makefile
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
#  21-Feb-2003 vp: removed CHNL_OMAP and CHNL_HELEN macro's.
#  18-Oct-2002 sb: Cretaed Initial Version
#  ----------------------------------------------------------------------------

PROJROOT=../..

include $(PROJROOT)/make/start.mk

# Do not change above "include" line(s)

# Arguments to tools, will move to make system once finalized.

EXTRA_CFLAGS         =

ifeq ($(PROCFAMILY),OMAP_2430)
CDEFS          = __KERNEL__ TICFG_PROC_VER TICFG_EVM_TYPE CHNL_SMCLASS CHNL_MESSAGES
else
ifeq ($(PROCFAMILY),OMAP_3430)
CDEFS          = __KERNEL__ TICFG_PROC_VER TICFG_EVM_TYPE CHNL_SMCLASS CHNL_MESSAGES
else
CDEFS          = __KERNEL__ TICFG_PROC_VER TICFG_EVM_TYPE CHNL_SMCLASS CHNL_MESSAGES CHNL_DDMA CHNL_DDZC
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

TARGETNAME  = libwmd


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
ifeq ($(PROCFAMILY),OMAP_2430)
WMD_SUBDIR=2430
else
ifeq ($(PROCFAMILY),OMAP_3430)
WMD_SUBDIR=3430
else
WMD_SUBDIR=1510
endif
endif
#linux/omap/$(WMD_SUBDIR)/tiomap_sm.c\

SOURCES     = \
omap/wmdchnl/common/chnl_sm.c\
omap/wmdchnl/common/msg_sm.c\
omap/wmdchnl/common/io_sm.c\
omap/$(WMD_SUBDIR)/tiomap.c\
omap/$(WMD_SUBDIR)/tiomap_pwr.c\
omap/common/tiomap_mmu.c\
omap/common/tiomap_sm.c\
omap/common/hal_mmu.c\
omap/common/tiomap_io.c\
omap/common/tiomap_util.c\
omap/common/mmu_fault.c\
omap/common/ue_deh.c

#TODO -- make search path to include proper HAL for chip type
# Search path for include files

#$(PROJROOT)/src/hal/common/regdefs/inc/arm11/

ifeq ($(PROCFAMILY),OMAP_3430)
INCLUDES    = \
    $(KRNLSRC)/include \
    $(PROJROOT)/inc \
    $(PROJROOT)/inc/linux \
    omap/wmdchnl/common \
    omap/common \
    omap/$(WMD_SUBDIR) \
    $(PROJROOT)/src/hal/common/inc \
    $(PROJROOT)/src/hal/common/dspss \
    $(PROJROOT)/src/hal/common/mbox \
    $(PROJROOT)/src/hal/common/ocp \
    $(PROJROOT)/src/hal/omap3/inc
else
INCLUDES    = \
    $(KRNLSRC)/include \
    $(PROJROOT)/inc \
    $(PROJROOT)/inc/linux \
    omap/wmdchnl/common \
    omap/common \
    omap/$(WMD_SUBDIR) \
    $(PROJROOT)/src/hal/common/inc \
    $(PROJROOT)/src/hal/common/dspss \
    $(PROJROOT)/src/hal/common/mbox \
    $(PROJROOT)/src/hal/common/ocp \
    $(PROJROOT)/src/hal/omap2/inc
endif

# Libraries needed for linking.

ST_LIBS        =
SH_LIBS        =

# Search path for library (and linker command) files.
# Current folder and target folder are included by default.

LIBINCLUDES =


# Do not change below "include" line(s)

include $(PROJROOT)/make/build.mk
