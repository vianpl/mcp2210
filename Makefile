obj-m := hid-mcp2210.o gpio-mcp23xx.o

ccflags-y += -g -DDEBUG

# normal makefile
KDIR ?= /lib/modules/`uname -r`/build

default:
	$(MAKE) -C $(KDIR) M=$$PWD

clean:
	rm -rf   Module.symvers modules.order *.o *.ko *.mod.c
