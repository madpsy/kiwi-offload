# Built on the Beagle (needs linux-headers-$(uname -r)): make -C kmod ; sudo insmod kmod/kodma.ko
obj-m := kodma.o
KDIR ?= /lib/modules/$(shell uname -r)/build
all:
	$(MAKE) -C $(KDIR) M=$(CURDIR) modules
clean:
	$(MAKE) -C $(KDIR) M=$(CURDIR) clean
