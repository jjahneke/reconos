obj-m := reconosmod64.o

SRC := $(shell pwd)

reconosmod64-objs := reconos.o osif_intc.o proc_control.o

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) CFLAGS_MODULE=-D"RECONOS_ARCH_zynq64"

modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) CFLAGS_MODULE=-D"RECONOS_ARCH_zynq64" modules_install

clean:
	rm -f *.o *~ core .depend .*.cmd *.ko *.mod.c
	rm -f Module.markers Module.symvers modules.order
	rm -rf .tmp_versions Modules.symvers
