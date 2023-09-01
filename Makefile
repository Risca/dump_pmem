obj-m += dump_pmem.o

PWD := $(CURDIR)
KDIR ?= /mnt/sdb1/omap5_uevm/linux-bisect
CROSS_COMPILE ?= /mnt/sdb1/omap5_uevm/arm-gnu-toolchain-12.3.rel1-x86_64-arm-none-linux-gnueabihf/bin/arm-none-linux-gnueabihf-
ARCH ?= arm

all:
	make -C $(KDIR) CROSS_COMPILE=$(CROSS_COMPILE) ARCH=$(ARCH) M=$(PWD) modules

install:
	make -C $(KDIR) CROSS_COMPILE=$(CROSS_COMPILE) ARCH=$(ARCH) M=$(PWD) INSTALL_MOD_PATH=/srv/nfs/rootfs modules_install

clean:
	make -C $(KDIR) M=$(PWD) clean
