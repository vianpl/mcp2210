obj-m := hid-mcp2210.o gpio-mcp23xx.o mcp2210-eval.o

ccflags-y += -g -DDEBUG

# normal makefile
KERNEL_SRC ?= /lib/modules/`uname -r`/build

default:
	$(MAKE) -C $(KERNEL_SRC) M=$$PWD

install:
	$(MAKE) -C $(KERNEL_SRC) M=$$PWD install

clean:
	rm -rf   Module.symvers modules.order *.o *.ko *.mod.c
