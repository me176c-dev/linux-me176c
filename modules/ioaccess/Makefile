TARGET_MODULE := ioaccess

ccflags-y := -DXOSA_LINUX -I$(src)/Include

# If we running by kernel building system
ifneq ($(KERNELRELEASE),)
	$(TARGET_MODULE)-objs := Linux/ioaccess.o
	obj-m := $(TARGET_MODULE).o

# If we are running without kernel build system
else
	BUILDSYSTEM_DIR?=/lib/modules/$(shell uname -r)/build
	PWD:=$(shell pwd)

all :
# run kernel build system to make module
	$(MAKE) -C $(BUILDSYSTEM_DIR) M=$(PWD) modules

clean:
# run kernel build system to cleanup in current directory
	$(MAKE) -C $(BUILDSYSTEM_DIR) M=$(PWD) clean

install:
	$(MAKE) -C $(BUILDSYSTEM_DIR) INSTALL_MOD_PATH=$(DESTDIR) M=$(PWD) modules_install

load:
	modprobe $(TARGET_MODULE)

unload:
	modprobe -r $(TARGET_MODULE)

endif

