Wireless debugging with esp8266.

What it is
----------

This is a port of Black Magic Probe to the esp8266, which allows wireless debugging of blackmagic-supported ARM targets.

What it is not
--------------

This is not a debugger for the esp8266 itself. I believe OpenOCD supports esp8266 now, but porting that to run *on* an esp8266 would be quite an undertaking.

Requirements
------------

Requires esp-open-rtos from https://github.com/SuperHouse/esp-open-rtos

Requires a seral interface cable (FTDI cable) for programming the esp8266.

For hardware, I ordered esp-01 modules from ebay seller 'tomyuen007' for $2.69 each, and connected them to switched dual-AA battery holders from ebay seller 'usa-daily-deals' for $1.76 each.

Installation
------------

Install esp-open-rtos according to its Readme. This will require installing esp-open-sdk as well.

Connect the serial interface cable to the RX and TX pins of the module.

Try the `http_get` example in the rtos distribution to make sure the SDK is set up and installed properly.

In Makefile.inc, set the following:

  RTOS_PATH (path to the rtos installation)
  ESPPORT (path to serial interface to esp8266 module)

Power up the module with GPIO0 pulled low to put it in programming mode.

Then `make PROBE_HOST=esp8266 flash` should build and install the firmware.

Connection
----------

I have only tested with an SWD connection. The ESP-01 does not have a lot of extra pins, so I reused RXD for SWCLK and GPIO2 for SWDIO. The pins are defined in platform.h.

Usage
-----

When powered up, the firmware will present a wifi access point with SSID 'blackmagic'. Connect to this access point. The firmware presents its debug interface at port 2000 on 172.16.0.1.

Inside gdb, connect with `target extended-remote 172.16.0.1:2000`. Then scan, attach, and debug as usual.

Hacking
-------

Undefine `ACCESS_POINT_MODE` in platform.c to have the debugger act as a wifi client instead of an access point.  This didn't work when I tried it on my network.  The data never made it from the esp8266's TCP stack to gdb, although it worked in the other direction.

Change .authmode from AUTH_OPEN to have a password-protected wifi experience.  Password is set by AP_PSK.

Don't try debugging prints during the SWD transactions. They will affect timing enough that JTAG will fail.

License
-------

esp8266 license terms are a whole kettle of fish.  The esp-open-rtos guys are gradually reverse-engineering the esp binary blobs and hopefully we will soon have a fully GPL-compatible SDK and it will be legal to distribute the compiled firmware images.
