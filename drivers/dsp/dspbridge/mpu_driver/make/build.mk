#
#  dspbridge/make/build.mk
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
#      Original version.
#  ----------------------------------------------------------------------------

# ALL PATHS IN MAKEFILE MUST BE RELATIVE TO ITS DIRECTORY

CDEFS       += $(CMDDEFS) # Add command line definitions
CDEFS       += $(PROCFAMILY) # Processor Family e.g. 1510,1610
CDEFS	    += $(CMDDEFS_START) # Definitions from start.mk
#   ----------------------------------------------------------
#   REMOVE LEADING AND TRAILING SPACES FROM MAKEFILE MACROS
#   ----------------------------------------------------------

TARGETNAME  := $(strip $(TARGETNAME))
TARGETTYPE  := $(strip $(TARGETTYPE))
SUBMODULES  := $(strip $(SUBMODULES))
SOURCES     := $(strip $(SOURCES))
INCLUDES    := $(strip $(INCLUDES))
LIBINCLUDES := $(strip $(LIBINCLUDES))

SH_SONAME   := $(strip $(SH_SONAME))
ST_LIBS     := $(strip $(ST_LIBS))
SH_LIBS     := $(strip $(SH_LIBS))

EXTRA_CFLAGS      := $(strip $(CFLAGS))
CDEFS       := $(strip $(CDEFS))
EXEC_ARGS   := $(strip $(EXEC_ARGS))
ST_LIB_ARGS := $(strip $(ST_LIB_ARGS))
SH_LIB_ARGS := $(strip $(SH_LIB_ARGS))

#   ----------------------------------------------------------
#   COMPILER OPTIONS
#   ----------------------------------------------------------

# Preprocessor : dependency file generation
ifndef NODEPENDS
ifndef nodepends
EXTRA_CFLAGS += -MD
endif
endif

#   Overall
EXTRA_CFLAGS += -pipe
#   Preprocessor
EXTRA_CFLAGS +=
#   Debugging
ifeq ($(BUILD),deb)
EXTRA_CFLAGS += -g
else
EXTRA_CFLAGS += -fomit-frame-pointer
endif
#   Warnings
EXTRA_CFLAGS += -Wall  -Wno-trigraphs -Werror-implicit-function-declaration #-Wno-format
#   Optimizations
#EXTRA_CFLAGS += -O2 -fno-strict-aliasing
#EXTRA_CFLAGS += -Os -fno-strict-aliasing
EXTRA_CFLAGS += -fno-strict-aliasing
#   Machine dependent
ifeq ($(PROCFAMILY),OMAP_1510)
EXTRA_CFLAGS += -mapcs-32 -march=armv4 -mtune=arm9tdmi -mshort-load-bytes -msoft-float
endif
ifeq ($(PROCFAMILY),OMAP_16xx)
EXTRA_CFLAGS += -mapcs-32 -march=armv4 -mtune=arm9tdmi -mshort-load-bytes -msoft-float
endif
ifeq ($(PROCFAMILY), OMAP_1710)
EXTRA_CFLAGS += -mapcs-32 -march=armv4 -mtune=arm9tdmi -msoft-float -DKBUILD_BASENAME=$(basename $(TARGETNAME)) -DKBUILD_MODNAME=$(basename $(TARGETNAME)) -DMODULE -D__LINUX_ARM_ARCH__=5
endif
#if you want to compile 2420 against GCC 3.4.0 toolset then chose armv6 achitecture
#and remove the -mshort-load-bytes from the below compile flags
ifeq ($(PROCFAMILY),OMAP_24xx)
EXTRA_CFLAGS += -mapcs-32 -march=armv5 -malignment-traps -mshort-load-bytes -msoft-float -mthumb-interwork -DKBUILD_BASENAME=$(basename $(TARGETNAME)) -DKBUILD_MODNAME=$(basename $(TARGETNAME)) -DMODULE -D__LINUX_ARM_ARCH__=6
endif

ifeq ($(PROCFAMILY),OMAP_2430)
ifeq ($(KRN_VER),2.6)
#EXTRA_CFLAGS += -mapcs-32 -march=armv6 -malignment-traps -msoft-float -mthumb-interwork -DKBUILD_BASENAME=$(basename $(TARGETNAME)) -DKBUILD_MODNAME=$(basename $(TARGETNAME)) -DMODULE -D__LINUX_ARM_ARCH__=6
EXTRA_CFLAGS += -march=armv6 -msoft-float -DKBUILD_BASENAME=$(basename $(TARGETNAME)) -DKBUILD_MODNAME=$(basename $(TARGETNAME))  -DMODULE -D__LINUX_ARM_ARCH__=6
endif
ifeq ($(KRN_VER),2.4)
EXTRA_CFLAGS += -mapcs-32 -march=armv5 -malignment-traps -mshort-load-bytes -msoft-float -mthumb-interwork -DKBUILD_BASENAME=$(basename $(TARGETNAME)) -DKBUILD_MODNAME=$(basename $(TARGETNAME)) -DMODULE -D__LINUX_ARM_ARCH__=6
endif
endif


ifeq ($(PROCFAMILY),OMAP_3430)
EXTRA_CFLAGS += -mapcs -mno-sched-prolog -mabi=aapcs-linux -mno-thumb-interwork -march=armv7a -msoft-float -Uarm -D"KBUILD_STR(s)=\#s" -D"KBUILD_BASENAME=KBUILD_STR($(basename $(TARGETNAME)))" -D"KBUILD_MODNAME=KBUILD_STR($(basename $(TARGETNAME)))"  -DMODULE -D__LINUX_ARM_ARCH__=7
endif

#   Code generation
EXTRA_CFLAGS += -fno-common
#   Macros
EXTRA_CFLAGS += -DLINUX $(addprefix -D, $(CDEFS))

ifdef __KERNEL__
EXTRA_CFLAGS      += -D__KERNEL__  -fno-builtin
endif

#   ----------------------------------------------------------
#   OBJECTS
#   ----------------------------------------------------------

BUILDDIR    = .obj/

# setup the target - check the given type - make sure we have the
# correct suffix on it
# TARGETNAME should not have a suffix on it - give an error if it does
ifneq ($(suffix $(TARGETNAME)),)
   $(error TARGETNAME can not have a suffix)
endif
ifeq ($(TARGETTYPE),SH_LIB)
   TARGET      := $(basename $(TARGETNAME)).so
else
ifeq ($(TARGETTYPE),MODULE)
   TARGET      := $(basename $(TARGETNAME)).o
   TARGETKO    := $(addsuffix .ko,$(basename $(TARGET)))
   TARGETMOD   := $(addsuffix .mod,$(basename $(TARGET)))
else
ifeq ($(TARGETTYPE),ST_LIB)
   TARGET      := $(basename $(TARGETNAME)).a
else
ifeq ($(TARGETTYPE),EXEC)
   TARGET      := $(basename $(TARGETNAME)).out
else
ifneq ($(TARGETTYPE),)
TARGET         := $(error Invalid TARGETTYPE)
endif
endif
endif
endif
endif

#LIBINCLUDES += $(TARGETDIR) $(TGTROOT)/lib $(TGTROOT)/usr/lib
LIBINCLUDES += $(TARGETDIR)/lib
SRCDIRS :=  $(sort $(dir $(SOURCES)))
OBJDIRS :=  $(addprefix $(BUILDDIR),$(SRCDIRS)) $(BUILDDIR)

BASEOBJ := $(addsuffix .o,$(basename $(SOURCES)))
OBJECTS := $(addprefix $(BUILDDIR), $(BASEOBJ))

ST_LIBNAMES := $(addsuffix .a, $(addprefix lib, $(ST_LIBS)))
DL_LIBNAMES := $(addsuffix .so, $(addprefix lib, $(SH_LIBS)))

vpath %.a $(LIBINCLUDES) $(TGTROOT)/lib $(TGTROOT)/usr/lib
vpath %.so $(LIBINCLUDES) $(TGTROOT)/lib $(TGTROOT)/usr/lib

#   ----------------------------------------------------------
#   BUILD ARGUMENTS
#   ----------------------------------------------------------

MAPFILE := -Wl,-Map,$(TARGET).map
INCPATH := $(addprefix -I, . $(INCLUDES)) $(LINUXINCLUDE)
LIBPATH := $(addprefix -L, $(LIBINCLUDES))
LIBFILE := $(addprefix -l, $(ST_LIBS) $(SH_LIBS)) $(LIB_OBJS)

ifeq ($(TARGETTYPE),SH_LIB)
EXTRA_CFLAGS += -fpic
TARGETARGS := $(SH_LIB_ARGS) -nostartfiles -nodefaultlibs -nostdlib -shared -Wl
ifneq ($(SH_SONAME),)
TARGETARGS += -Wl,-soname,$(SH_SONAME)
endif
endif

ifeq ($(TARGETTYPE),MODULE)
TARGETARGS := $(SH_LIB_ARGS) -nostartfiles -nodefaultlibs -nostdlib -Wl,-r
ifneq ($(SH_SONAME),)
TARGETARGS += -Wl,-soname,$(SH_SONAME)
endif
endif

ifeq ($(TARGETTYPE),ST_LIB)
TARGETARGS := $(ST_LIB_ARGS) -nostartfiles -nodefaultlibs -nostdlib -Wl,-r
endif

ifeq ($(TARGETTYPE),EXEC)
TARGETARGS := $(EXEC_ARGS)
endif

.PHONY  :   all $(SUBMODULES) clean cleantrg SHELLERR Debug

#   ==========================================================
#   all
#   ==========================================================
all :  $(CHECKSHELL) $(SUBMODULES)

#   ==========================================================
#   Make submodules
#   ==========================================================
$(SUBMODULES):
ifndef NORECURSE
ifndef norecurse
	$(MAKE) -C $@ $(filter-out $(SUBMODULES),$(MAKECMDGOALS))
endif
endif

ifneq ($(TARGETTYPE),)

# if this is driver module level (and 1710 or 24xx), build KO file too
ifneq ($(TOPLEVEL),)
all :  $(OBJDIRS) $(TARGETKO)
else
all :  $(OBJDIRS) $(TARGET)
endif

#   ==========================================================
#   Create directories
#   ==========================================================
$(OBJDIRS) $(TARGETDIR):
	@$(call MKDIR, $@)

#   ==========================================================
#   Product 2.6.x kernel module based on target
#   ==========================================================

# Link module .o with vermagic .o
$(TARGETKO): $(TARGETMOD).o $(TARGET)
	$(LD) -EL -r -o $@ $^

# Compile vermagic
$(TARGETMOD).o: $(TARGETMOD).c
	$(CC) -c $(TARGETARGS) $(EXTRA_CFLAGS) $(INCPATH) -o $@ $<

# removed - need to be done as a pre-step to building
	$(MAKE) -C $(KRNLSRC) M=$(PROJROOT)/src modules

# Generate Module vermagic
$(TARGETMOD).c: $(TARGET)
	$(MODPOST) $(TARGET)


#   ==========================================================
#   Build target
#   ==========================================================
$(TARGET):$(OBJECTS) $(ST_LIBNAMES) $(DL_LIBNAMES)
#   @$(SHELLCMD) echo Building $@
#	$(CC) $(TARGETARGS) $(EXTRA_CFLAGS) $(LIBPATH) $(MAPFILE) -o $@ $(BASEOBJ) $(LIBFILE)
#	$(CC) $(TARGETARGS) $(EXTRA_CFLAGS) $(LIBPATH) $(MAPFILE) -o $@ $(OBJECTS) $(LIBFILE)
ifeq ($(TARGETTYPE),ST_LIB)
	$(AR) r $@ $(OBJECTS)
else
	$(CC) $(TARGETARGS) $(EXTRA_CFLAGS) $(LIBPATH) $(MAPFILE) -o $@ $(OBJECTS) $(LIBFILE)
endif

#   ==========================================================
#   Compile .c file
#   ==========================================================
$(BUILDDIR)%.o:%.c
#   echo Compiling $(patsubst $(BUILDDIR)%.o,%.c, $@)
	$(CC) -c $(EXTRA_CFLAGS) $(INCPATH) -o $@ $(patsubst $(BUILDDIR)%.o,%.c, $@)

#   ==========================================================
#   Compile .S file
#   ==========================================================
$(BUILDDIR)%.o:%.S
#   echo Compiling $(patsubst $(BUILDDIR)%.o,%.S, $@)
	$(CC) -c $(EXTRA_CFLAGS) $(INCPATH) -o $@ $(patsubst $(BUILDDIR)%.o,%.S, $@)

endif   # ifneq ($(TARGETTYPE),)

#   ----------------------------------------------------------
#   install - install the files
#   ----------------------------------------------------------
install:: $(TARGETDIR) $(SUBMODULES) $(TARGET)
ifdef HOSTRELEASE
ifdef SH_SONAME
	$(INSTALL) -D $(TARGET) $(TARGETDIR)/$(HOSTRELEASE)/$(SH_SONAME)
	$(RM) -f $(TARGETDIR)/$(HOSTRELEASE)/$(TARGET)
	ln -s $(SH_SONAME) $(TARGETDIR)/$(HOSTRELEASE)/$(TARGET)
else
ifneq ($(TOPLEVEL),)
	$(INSTALL) -D $(TARGETKO) $(TARGETDIR)/$(HOSTRELEASE)/$(TARGETKO)
else
	$(INSTALL) -D $(TARGET) $(TARGETDIR)/$(HOSTRELEASE)/$(TARGET)
endif
endif
endif
ifdef 0 # removed - components shouldn't put things in the production fs
ifdef ROOTFSRELEASE
	$(call MKDIR, $(ROOTFSDIR)/$(ROOTFSRELEASE))
ifdef SH_SONAME
	$(STRIP) --strip-unneeded -xgo $(ROOTFSDIR)/$(ROOTFSRELEASE)/$(SH_SONAME) $(TARGET)
	$(RM) -f $(ROOTFSDIR)/$(ROOTFSRELEASE)/$(TARGET)
	ln -s $(SH_SONAME) $(ROOTFSDIR)/$(ROOTFSRELEASE)/$(TARGET)
else
ifneq ($(TOPLEVEL),)
	$(STRIP) --strip-unneeded -xgo $(ROOTFSDIR)/$(ROOTFSRELEASE)/$(TARGETKO) $(TARGETKO)
else
	$(STRIP) --strip-unneeded -xgo $(ROOTFSDIR)/$(ROOTFSRELEASE)/$(TARGET) $(TARGET)
endif
endif
endif
endif

#   ----------------------------------------------------------
#   clean - Remove build directory and target files
#   Linux : Removes object and dependency files in build folder
#   DOS   : Removes object dirs in build folder
#   ----------------------------------------------------------
clean : $(SUBMODULES)
ifneq ($(TARGETTYPE),)
ifneq ($(OBJECTS),)
	- $(call RM,-f $(OBJECTS))
	- $(call RM,-f $(OBJECTS:.o=.d))
	- $(call RMDIR,-f $(BUILDDIR))
endif
	- $(call RM,-f $(TARGET))
	- $(call RM,-f $(TARGET).map)
	- $(call RM,-f $(TARGETKO))
	- $(call RM,-f $(TARGETMOD).c)
	- $(call RM,-f $(TARGETMOD).o)
	- $(call RM,-f $(TARGETMOD).d)
ifneq ($(TOPLEVEL),)
	- @$(call RM,-f $(TARGETKO))
	- @$(call RM,-f $(TARGETMOD).c)
	- @$(call RM,-f $(TARGETMOD).o)
	- @$(call RM,-f $(TARGETMOD).d)
endif
endif

cleantrg : $(SUBMODULES)
ifneq ($(TARGETTYPE),)
	- @$(call RM, $(TARGET))
	- @$(call RM, $(TARGET).map)
ifneq ($(TOPLEVEL),)
	- @$(call RM, $(TARGETKO))
	- @$(call RM, $(TARGETMOD).c)
	- @$(call RM, $(TARGETMOD).o)
	- @$(call RM, $(TARGETMOD).d)
endif
endif

#   ----------------------------------------------------------
#   Include dependency files generated by preprocessor.
#
#   Dependency files are placed in main object directory because
#   dependent files' paths for same source file varies with the
#   directory from where gmake is run
#   ----------------------------------------------------------
ifndef NODEPENDS
ifndef nodepends
ifneq ($(OBJECTS),)
-include $(OBJECTS:.o=.d)
endif
endif
endif

#   ----------------------------------------------------------
#   Generate fatal error if make variable SHELL is incorrect
#   ----------------------------------------------------------
SHELLERR::
	@$(SHELLCMD) echo Fatal error: SHELL set to $(SHELL) instead of $(MYSHELL)
	@$(SHELLCMD) echo set $(MYSHELL) to correct path and CASE SENSITIVE FILE NAME and EXTENSTION
	@$(SHELLCMD) echo of your command shell
	$(ERR)


#   ----------------------------------------------------------
#   For debugging script
#   ----------------------------------------------------------
Debug::$(SUBMODULES)
	@$(SHELLCMD) echo SHELL: $(SHELL)
	@$(SHELLCMD) echo
	@$(SHELLCMD) echo CDEFS: $(CDEFS)
	@$(SHELLCMD) echo
	@$(SHELLCMD) echo CONFIG_SHELL: $(CONFIG_SHELL)
	@$(SHELLCMD) echo
	@$(SHELLCMD) echo CURDIR: $(CURDIR)
	@$(SHELLCMD) echo
	@$(SHELLCMD) echo SRCDIRS: $(SRCDIRS)
	@$(SHELLCMD) echo
	@$(SHELLCMD) echo OBJDIRS: $(OBJDIRS)
	@$(SHELLCMD) echo
	@$(SHELLCMD) echo OBJECTS: $(OBJECTS)
	@$(SHELLCMD) echo
	@$(SHELLCMD) echo BUILDDIR: $(BUILDDIR)
	@$(SHELLCMD) echo
	@$(SHELLCMD) echo TARGETDIR TARGETNAME: $(TARGET)
	@$(SHELLCMD) echo
	@$(SHELLCMD) echo MAKEDIR: $(MAKEDIR)
	@$(SHELLCMD) echo
	@$(SHELLCMD) echo INCLUDES: $(INCLUDES)
	@$(SHELLCMD) echo
	@$(SHELLCMD) echo DL_LIBNAMES: $(DL_LIBNAMES)
	@$(SHELLCMD) echo
	@$(SHELLCMD) echo LIBFILE: $(LIBFILE)
	@$(SHELLCMD) echo

