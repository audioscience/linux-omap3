#
# dspbridge/src/Makefile
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

PROJROOT	= ..

include $(PROJROOT)/make/start.mk
# Do not change above "include" line(s)

# Arguments to tools, will move to make system once finalized.

EXTRA_CFLAGS         =
CDEFS          =

EXEC_ARGS      =
ST_LIB_ARGS    =
SH_LIB_ARGS    =

# Define this macro if target runs in kernel mode
__KERNEL__ = 1

# Target name and extension
# static library        (ST_LIB): filename.a
# shared library soname (SH_LIB): filename.so.maj_ver.min_ver
# executable            (EXEC)  : filename.out

TARGETNAME  = bridgedriver


# TARGETTYPE must be EXEC, ST_LIB or SH_LIB in upper case.

TARGETTYPE  = MODULE

# install directory relative to the HOSTTARGET directory
HOSTRELEASE = lib/modules/$(KRN_RELEASE)/dspbridge

# install directory relative to the root filesystem
install:: $(KRN_VERSION)
ROOTFSRELEASE = lib/modules/$(KRN_RELEASE)/dspbridge

# For shared object library, soname is filename.so.maj_ver

#hal/common \
SH_SONAME =

# Folders in which gmake will run before building current target
ifeq ($(PROCFAMILY),OMAP_2430)
SUBMODULES  = \
gen \
osal \
wmd \
pmgr \
rmgr \
api \
api_kern \
dynload \
hal/omap2
else
ifeq ($(PROCFAMILY),OMAP_3430)
SUBMODULES  = \
gen \
osal \
wmd \
pmgr \
rmgr \
dynload \
hal/omap3
else
SUBMODULES  = \
gen \
osal \
wmd \
pmgr \
rmgr \
hal/omap3 \
dynload
endif
endif

ifeq ($(PROCFAMILY),OMAP_3430)

bridgedriver-objs    := osal/libosal.o gen/libgen.o rmgr/librmgr.o wmd/libwmd.o pmgr/libpmgr.o dynload/libdload.o hal/omap3/libhal_omap3.o \

obj-m	+= bridgedriver.o
endif



# Filename must not begin with '.', '/' or '\'

SOURCES     = \

# Search path for include files

INCLUDES    = \
    $(KRNLSRC)/include
#hal/common/libhal_common.o \
# Libraries needed for linking.
ST_LIBS        =
SH_LIBS        =
ifeq ($(PROCFAMILY),OMAP_2430)
LIB_OBJS       = \
	osal/libosal.o \
	gen/libgen.o \
	rmgr/librmgr.o \
	wmd/libwmd.o \
	pmgr/libpmgr.o \
	dynload/libdload.o \
	hal/omap2/libhal_omap2.o
else
ifeq ($(PROCFAMILY),OMAP_3430)
LIB_OBJS       = \
	osal/libosal.o \
	gen/libgen.o \
	rmgr/librmgr.o \
	wmd/libwmd.o \
	pmgr/libpmgr.o \
	dynload/libdload.o \
	hal/omap3/libhal_omap3.o
else
LIB_OBJS       = \
	osal/libosal.o \
	gen/libgen.o \
	rmgr/librmgr.o \
	wmd/libwmd.o \
	pmgr/libpmgr.o \
	hal/omap3/libhal_omap3.o \
	dynload/libdload.o
endif
endif

# Search path for library (and linker command) files.
# Target folder is included by default.

LIBINCLUDES =

ifneq ($(BRIDGE_BLD_LINUX_VERSION),1321)
ifeq ($(PROCFAMILY),OMAP_1710)
TOPLEVEL := 1
endif

ifeq ($(PROCFAMILY),OMAP_24xx)
TOPLEVEL := 1
endif

ifeq ($(PROCFAMILY),OMAP_2430)
TOPLEVEL := 1
endif

ifeq ($(PROCFAMILY),OMAP_3430)
TOPLEVEL := 1
endif
endif

# Do not change below "include" line(s)

include $(PROJROOT)/make/build.mk


