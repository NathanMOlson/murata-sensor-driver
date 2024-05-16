
obj-m += sch16xx.o

# path to a compiled kernel
KPATH ?= /usr/src/linux-headers-$$(uname -r)

EXTRA_CFLAGS += $(ADD_EXTRA_CFLAGS) -Werror -Wall
#EXTRA_CFLAGS += -DDEBUG

ARCH = arm64
#CROSS_COMPILE = arm-linux-gnueabihf-

MAKE_ARCH = $(MAKE) C=2 ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE)
#MAKE_ARCH = make

all:
	$(MAKE_ARCH) -C $(KPATH) M=$(PWD) modules

clean:
	$(MAKE_ARCH) -C $(KPATH) M=$(PWD) clean
