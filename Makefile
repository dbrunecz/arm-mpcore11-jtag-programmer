#sudo apt-get install libftdi-dev

CFLAGS+= -Wall -Werror
CFLAGS+= -ffunction-sections
CFLAGS+= -fdata-sections

LDFLAGS= -lftdi
LDFLAGS+= -Wl,-gc-sections -Wl,-print-gc-sections

DATESTR:=$(shell date -Iseconds)
USERNAME:=$(shell id -urn)
HOSTNAME:=$(shell hostname)
MACH:=$(shell uname -pi | tr ' ' '-')
KERN:=$(shell uname -r)
DISTVER:=$(shell lsb_release -ds | tr ' ' '-')
CCVER:=$(shell $(CC) --version | grep -i cc | tr -d '()' | tr ' ' '-')
GITREV=$(shell git log -1 --pretty=format:"%H")

jtag-armbin-load.o: CFLAGS += -DDATESTR=\"$(DATESTR)\"
jtag-armbin-load.o: CFLAGS += -DUSERNAME=\"$(USERNAME)\"
jtag-armbin-load.o: CFLAGS += -DHOSTNAME=\"$(HOSTNAME)\"
jtag-armbin-load.o: CFLAGS += -DMACH=\"$(MACH)\"
jtag-armbin-load.o: CFLAGS += -DKERN=\"$(KERN)\"
jtag-armbin-load.o: CFLAGS += -DDISTVER=\"$(DISTVER)\"
jtag-armbin-load.o: CFLAGS += -DCCVER=\"$(CCVER)\"
jtag-armbin-load.o: CFLAGS += -DGITREV=\"$(GITREV)\"
jtag-armbin-load.o: CFLAGS += -ggdb 

all: jtag-armbin-load

jtag-armbin-load: jtag-armbin-load.o jtag.o mpsse.o jtag.o
	$(CC) $^ -o $@ $(LDFLAGS)

clean:
	@rm -f *.o jtag-armbin-load
