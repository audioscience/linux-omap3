#
# Makefile for PTP 1588 clock support.
#

ptp-y					:= ptp_clock.o ptp_chardev.o ptp_sysfs.o
obj-$(CONFIG_PTP_1588_CLOCK)		+= ptp.o
obj-$(CONFIG_PTP_1588_CLOCK_IXP46X)	+= ptp_ixp46x.o
obj-$(CONFIG_PTP_1588_CLOCK_CPTS)	+= ptp_cpts.o
obj-$(CONFIG_PTP_1588_CLOCK_TI814X_GEN)	+= ptp_ti814x_gen.o
