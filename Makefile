ifneq ($(V), 1)
MFLAGS += --no-print-dir
Q := @
endif

all:
	$(Q)if [ ! -f libopencm3/Makefile ]; then \
		echo "Initialising git submodules..." ;\
		git submodule init ;\
		git submodule update ;\
	fi
	$(Q)$(MAKE) $(MFLAGS) -C libopencm3 lib
	$(Q)$(MAKE) $(MFLAGS) -C src

clean:
	$(Q)$(MAKE) $(MFLAGS) -C libopencm3 $@
	$(Q)$(MAKE) $(MFLAGS) -C src $@
	rm -rf src/_build

ZIPFILE=src/_build/t.zip

zip:
	$(Q)$(MAKE) $(MFLAGS) -C src

$(ZIPFILE): all
	nrfutil pkg generate --application src/_build/nrf52840_xxaa.hex --hw-version 52 --sd-req 0 --application-version=0 $@

#SERIAL=CEB289B40D29
SERIAL=EC4938BFC6DB

usbdfu: $(ZIPFILE)
	nrfutil dfu usb-serial --package=$< --serial-number=$(SERIAL) --connect-delay=0
