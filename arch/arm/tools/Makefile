#
# linux/arch/arm/tools/Makefile
#
# Copyright (C) 2001 Russell King
#
# Modified by:
#   Sergio Aguirre <saaguirre@ti.com>

arch/arm/include/asm/mach-types.h: $(src)/gen-mach-types $(src)/mach-types
	@echo '  GEN     $@'
	@mkdir -p $(dir $@)
	$(Q)$(AWK) -f $^ > $@ || { rm -f $@; /bin/false; }
