#PWD:= $(shell pwd)

#IOT_HOME = /opt/clanton-tiny/1.4.4/sysroots

#KDIR:=$(IOT_HOME)/i586-poky-linux-uclibc/usr/src/kernel  

#PATH := $(PATH):$(IOT_HOME)/x86_64-pokysdk-linux/usr/bin/i586-poky-linux

#CROSS_COMPILE = i586-poky-linux

#SROOT=$(IOT_HOME)/i586-poky-linux/

#CC = i586-poky-linux-gcc



IOT_HOME = /opt/iot-devkit/1.7.2/sysroots

KDIR:=$(IOT_HOME)/i586-poky-linux/usr/src/kernel  

PATH := $(PATH):$(IOT_HOME)/x86_64-pokysdk-linux/usr/bin/i586-poky-linux

CROSS_COMPILE = i586-poky-linux-

SROOT=$(IOT_HOME)/i586-poky-linux/

CC = i586-poky-linux-gcc

CFLAGS+ = -g -Wall 


ARCH = x86



APP = main



obj-m:= HCSR_drv.o



all:

	make EXTRA_FLAGS=$(CFLAGS) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KDIR) M=$(PWD) modules

	$(CC) -o $(APP) main.c -pthread -Wall -g --sysroot=$(SROOT)


clean:

	rm -f *.ko

	rm -f *.o

	rm -f Module.symvers

	rm -f modules.order

	rm -f *.mod.c

	rm -rf .tmp_versions

	rm -f *.mod.c

	rm -f *.mod.o

	rm -f \.*.cmd

	rm -f Module.markers

	rm -f $(APP) 
