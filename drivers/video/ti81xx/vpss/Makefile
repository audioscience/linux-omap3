INCS =	-I$(IPCDIR)/ \
	-I$(SYSLINK_ROOT)

EXTRA_CFLAGS += $(INCS)

obj-$(CONFIG_TI81XX_VPSS) += vpss.o
vpss-y := core.o fvid2.o dctrl.o grpx.o sbuf.o system.o
ifdef CONFIG_TI81XX_VPSS_SII9022A
vpss-y += sii9022a.o
endif
