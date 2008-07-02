# Makefile for OMAP1/2 camera driver

obj-$(CONFIG_VIDEO_OMAP_CAMERA) += omapcamera.o

objs-y$(CONFIG_ARCH_OMAP16XX) += omap16xxcam.o camera_core.o

obj-$(CONFIG_VIDEO_OMAP24XX_VIDEOLIB) += omap24xxlib.o 
obj-$(CONFIG_VIDEO_OMAP24XX_VIDEOOUT) += omap24xxvout.o
omapcamera-objs := $(objs-yy)

EXTRA_CFLAGS = -I$(src)/..
