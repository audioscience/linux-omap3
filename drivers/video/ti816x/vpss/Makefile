INCS =	-I$(IPCDIR)/ \
	-I$(SYSLINK_ROOT)

EXTRA_CFLAGS += $(INCS)

obj-$(CONFIG_TI816X_VPSS) += vpss.o
vpss-y := core.o fvid2.o dctrl.o grpx.o sbuf.o
ifdef CONFIG_TI816X_VPSS_SII9022A
vpss-y += sii9022a.o
endif
