#
#  dspbridge/make/start.mk
#
#  DSP-BIOS Bridge build rules.
#
#  Copyright (C) 2007 Texas Instruments, Inc.
#
#  This program is free software; you can redistribute it and/or modify it
#  under the terms of the GNU Lesser General Public License as published
#  by the Free Software Foundation version 2.1 of the License.
#
#  This program is distributed .as is. WITHOUT ANY WARRANTY of any kind,
#  whether express or implied; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
#  Lesser General Public License for more details.
#


#  ----------------------------------------------------------------------------
#  Revision History
#
#  MAY 16, 2002    REF=ORG    Sripal Bagadia
#  APR 02, 2004    REWRITE    Keith Deacon
#  ----------------------------------------------------------------------------

# make sure we have a prefix
ifndef PREFIX
$(error Error: variable PREFIX not defined)
endif
ifndef PROJROOT
$(error Error: variable PROJROOT not defined)
endif

CMDDEFS =
CMDDEFS_START =

# this is a driver - so find the kernel
ifndef KRNLSRC
$(error Error: variable KRNLSRC not defined)
endif

# make sure we have a kernel .config
# this mostly insures we are in sync
ifeq ($(filter distclean,$(MAKECMDGOALS)),)
ifeq ($(wildcard $(KRNLSRC)/.config),)
$(error Error: Kernel not configured)
else
# just include the kernel config file - so we can use the variables
include $(KRNLSRC)/.config
endif
else
PROCFAMILY=unknown
endif

# if the cross prefix is not defined - assume arm-linux-

# Kernel release string
ifeq ($(wildcard $(KRNLSRC)/include/linux/utsrelease.h),)
KRN_VERSION = $(KRNLSRC)/include/linux/version.h
else
KRN_VERSION = $(KRNLSRC)/include/linux/utsrelease.h
endif
KRN_RELEASE = $(shell head -1 $(KRN_VERSION) | cut -d '"' -f 2 )
KRN_VER = $(shell head -1 $(KRN_VERSION) | cut -d '"' -f 2|cut -c1-3 )

BRIDGE_BLD_LINUX_VERSION = $(shell grep LINUX_VERSION_CODE $(KRN_VERSION) | cut -d ' ' -f 3 | cut -c1-4 )

ifndef CROSS
ifeq ($(KRN_VER),2.4)
CROSS=arm_v5t_le-
endif
ifeq ($(KRN_VER),2.6)
CROSS=arm-linux-
endif
endif

ifdef CONFIG_ARCH_OMAP24XX
PROCFAMILY=OMAP_24xx
ifeq ($(KRN_VER),2.4)
CROSS=arm_v5t_le-
endif
ifeq ($(KRN_VER),2.6)
CROSS=arm-linux-
endif
endif

ifdef CONFIG_ARCH_OMAP243X
PROCFAMILY=OMAP_2430
ifeq ($(KRN_VER),2.4)
CROSS=arm_v5t_le-
endif
ifeq ($(KRN_VER),2.6)
#CROSS=arm-linux-
CROSS=arm_v6_vfp_le-
endif
endif


ifdef CONFIG_ARCH_OMAP34XX
PROCFAMILY=OMAP_3430
ifeq ($(KRN_VER),2.4)
CROSS=arm_v5t_le-
endif
ifdef CONFIG_PM
CMDDEFS_START +=
else
CMDDEFS_START +=DISABLE_BRIDGE_PM
endif
ifeq ($(KRN_VER),2.6)
CROSS=arm-none-linux-gnueabi-
#CMDDEFS_START +=RES_CLEANUP_DISABLE
endif
endif

ifdef CONFIG_ARCH_OMAP1710
PROCFAMILY=OMAP_1710
ifeq ($(KRN_VER),2.4)
CROSS=arm_v5t_le-
endif
ifeq ($(KRN_VER),2.6)
CROSS=arm-linux-
endif
endif

ifdef CONFIG_ARCH_OMAP1610
PROCFAMILY=OMAP_16xx
ifeq ($(KRN_VER),2.4)
CROSS=arm_v5t_le-
endif
ifeq ($(KRN_VER),2.6)
CROSS=arm-linux-
endif
endif

ifndef PROCFAMILY
$(error Error: PROCFAMILY can not be determined from Kernel .config)
endif

# CMDDEFS need to defined or things just won't build - this needs
# a good hard look because it's not structured well
ifdef CONFIG_DPM
#CMDDEFS=DEBUG GT_TRACE CONFIG_BRIDGE_DPM
CMDDEFS +=
#CMDDEFS=
else
CMDDEFS +=
#CMDDEFS=
endif

# define out output directories
#ifndef HOSTDIR
#HOSTDIR=$(PREFIX)/host
#endif

ifndef TARGETDIR
TARGETDIR=$(PREFIX)/target
endif

#ifndef ROOTFSDIR
#ROOTFSDIR=$(PREFIX)/rootfs
#endif


#default (first) target should be "all"
#make sure the target directories are created
all: $(TARGETDIR)

CONFIG_SHELL := /bin/bash

SHELL := $(CONFIG_SHELL)

# Current version of gmake (3.79.1) cannot run windows shell's internal commands
# We need to invoke command interpreter explicitly to do so.
# for winnt it is cmd /c <command>
SHELLCMD:=

ifneq ($(SHELL),$(CONFIG_SHELL))
CHECKSHELL:=SHELLERR
else
CHECKSHELL:=
endif

# Error string to generate fatal error and abort gmake
ERR = $(error Makefile generated fatal error while building target "$@")

CP  :=   cp

MAKEFLAGS = r

QUIET := &> /dev/null

# Should never be :=
RM    = rm $(1)
MV    = mv $(1) $(2)
RMDIR = rm -r $(1)
MKDIR = mkdir -p $(1)
INSTALL = install

# Current Makefile directory
MAKEDIR := $(CURDIR)

# Implicit rule search not needed for *.d, *.c, *.h
%.d:
%.c:
%.h:

#   Tools
CC	:= $(CROSS)gcc
AR	:= $(CROSS)ar
LD	:= $(CROSS)ld
STRIP	:= $(CROSS)strip
ifneq ($(wildcard $(KRNLSRC)/scripts/modpost),)
MODPOST := $(KRNLSRC)/scripts/modpost		\
        $(if $(CONFIG_MODVERSIONS),-m)             \
        $(if $(CONFIG_MODULE_SRCVERSION_ALL),-a,)

endif
ifneq ($(wildcard $(KRNLSRC)/scripts/mod/modpost),)
MODPOST := $(KRNLSRC)/scripts/mod/modpost
endif

ifeq ($(wildcard $(KRNLSRC)/include/linux/autoconf.h),)
$(error Error: Kernel not configured, Configure and Build the Kernel before building Bridgedriver)
else
LINUXINCLUDE    := -include $(KRNLSRC)/include/linux/autoconf.h
endif

# Common Test include directories
COMMONTESTINC = \
$(PROJROOT)/inc \
$(PROJROOT)/testmpu/inc \
$(PROJROOT)/testmpu/OSAL/include
