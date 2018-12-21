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

esp8266:
	$(Q)$(MAKE) $(MFLAGS) -C src include/version.h
	$(Q)$(MAKE) $(MFLAGS) -C src PROBE_HOST=esp8266

clean:
	$(Q)$(MAKE) $(MFLAGS) -C libopencm3 $@
	$(Q)$(MAKE) $(MFLAGS) -C src $@

flash:
	$(Q)$(MAKE) $(MFLAGS) -C src flash
