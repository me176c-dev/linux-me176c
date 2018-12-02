#
# Makefile for HM2056 camera from Aptina
#

ifneq (,$(filter ME176C K013 ME181C K011,$(TARGET_PROJECT)))
obj-$(CONFIG_VIDEO_HM2056)	+= hm2056_raw_me176c.o
else
obj-$(CONFIG_VIDEO_HM2056)	+= hm2056_raw.o
endif
