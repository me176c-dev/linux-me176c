obj-$(CONFIG_BCM_BT_LPM) += bcm_bt_lpm.o
ifneq (,$(filter ME176CE,$(ASUS_PROJECT)))
        KBUILD_CFLAGS += -DME176CE_WORKAROUND
endif
ifneq (,$(filter ME176C,$(ASUS_PROJECT)))
    KBUILD_CFLAGS += -DFOR_2076
endif
