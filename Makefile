obj-m := hid-mcp2210.o mcp2210-eval.o

ccflags-y += -g -DDEBUG

# normal makefile
KERNEL_SRC ?= /lib/modules/`uname -r`/build

default:
	$(MAKE) -C $(KERNEL_SRC) M=$$PWD

modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$$PWD modules_install

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$$PWD clean
