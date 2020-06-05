# -*-Makefile-*-
RTOS_PATH=~/esp-open-rtos
ESPPORT = /dev/ttyFTDI_CABLE_TX
ESPBAUD = 230400

PROGRAM=blackmagic

CFLAGS += -DLIBFTDI -DLWIP
CFLAGS += -Wno-unused-parameter
CFLAGS += -Wno-sign-compare
CFLAGS += -Wno-old-style-declaration
CFLAGS += -Wno-type-limits
CFLAGS += -Wno-unused-variable
CFLAGS += -Wno-empty-body
CFLAGS += -DconfigENABLE_BACKWARD_COMPATIBILITY=1

# Requires hack from https://groups.google.com/forum/#!searchin/esp-open-rtos/hostname/esp-open-rtos/bptnLZDxaEY/bBsTYf02AAAJ
CFLAGS +=  -DLWIP_NETIF_HOSTNAME=1 -DESP8266_HOSTNAME=\"BLACKMAGIC01\"

# we need to supercede these, or borrow specific versions from other platforms
SRC := $(filter-out main.c gdb_if.c jtagtap.c, $(SRC))

# default timing functions, good enough for us
SRC += timing.c

# yep, manual VPATH search, thanks to the wondrous esp-open-rtos build system
PROGRAM_SRC_FILES = $(foreach sdir, . $(VPATH), $(foreach f, $(SRC), $(wildcard $(sdir)/$(f))))

# use the Berkeley sockets interface
PROGRAM_SRC_FILES += platforms/libftdi/gdb_if.c
# ... and the bitbang JTAG interface
PROGRAM_SRC_FILES += platforms/stm32/jtagtap.c

BUILD_DIR = platforms/esp8266/build/
FIRMWARE_DIR = platforms/esp8266/firmware/

EXTRA_COMPONENTS=extras/dhcpserver

all: include/version.h

include $(RTOS_PATH)/common.mk

host_clean:
	$(Q) rm -rf $(BUILD_DIR) $(FIRMWARE_DIR)

debug_shell: flash
	miniterm.py $(ESPPORT) 460800