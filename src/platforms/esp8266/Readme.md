# Wireless debugging with esp8266.

## What it is

This is a port of Black Magic Probe to the esp8266, which allows wireless debugging of blackmagic-supported ARM targets.

## What it is not

This is not a debugger for the esp8266 itself. I believe OpenOCD supports esp8266 now, but porting that to run *on* an esp8266 would be quite an undertaking.

## Requirements

Requires esp-open-rtos from https://github.com/SuperHouse/esp-open-rtos

Requires a seral interface cable (FTDI cable) for programming the esp8266.

For hardware, I ordered esp-01 modules from ebay seller 'tomyuen007' for $2.69 each, and connected them to switched dual-AA battery holders from ebay seller 'usa-daily-deals' for $1.76 each.

I made a PCB layout at https://github.com/markrages/blackmagic-hardware/tree/master/contrib/esp8266, also https://oshpark.com/shared_projects/1pLYdVFd . The PCB gives nice connections for the 0.05" 10-pin SWD cable and FTDI cable.

##  Installation

Install esp-open-rtos according to its Readme. This will require installing esp-open-sdk as well.

Make a `private_ssid_config.h` file according to the SDK Readme.  This will allow the module onto your wifi network.

Apply the patch `open_rtos_hostname.patch` to esp-open-rtos:

```
esp-open-rtos$ patch -p1 < open_rtos_hostname.patch
```
This will allow a friendly hostname to connect to, if your network's DHCP+DNS server is configured appropriately. This method is from https://groups.google.com/forum/#!searchin/esp-open-rtos/hostname/esp-open-rtos/bptnLZDxaEY/bBsTYf02AAAJ

Connect the serial interface cable to the RX and TX pins of the module.

Try the `http_get` example in the rtos distribution to make sure the SDK is set up and installed properly.

In Makefile.inc, set the following:

  RTOS_PATH (path to the rtos installation)
  ESPPORT (path to serial interface to esp8266 module)

Power up the module with GPIO0 pulled low to put it in programming mode.

Then `make PROBE_HOST=esp8266 flash` should build and install the firmware.

## Target connection

I have only tested with an SWD connection. The ESP-01 does not have a lot of extra pins, so I reused RXD for SWCLK and GPIO2 for SWDIO. The pins are defined in platform.h.

## Usage

When powered up, the firmware will connect to the network using the credentials in `private_ssid_config.h`.  If your DHCP server accepts hostnames, it will take the hostname `blackmagic01`.  This is set in the Makefile; if you have more than one module give them different names!

Inside gdb, connect with `target extended-remote blackmagic01:2022`. Then scan, attach, and debug as usual.

## Hacking

### Access point mode:

Uncomment `#define ACCESS_POINT_MODE` in `platform.c` to enable access point mode.

When powered up, the firmware will present a wifi access point with SSID `blackmagic`. Connect to this access point. The firmware presents its debug interface at port 2022 on 172.16.0.1.

Inside gdb, connect with `target extended-remote 172.16.0.1:2022`. Then scan, attach, and debug as usual.

Change `.authmode` from `AUTH_OPEN` to have a password-protected wifi experience.  Password is set by AP_PSK.

### Advice

Don't try debugging prints during the SWD transactions. They will affect timing enough that JTAG will fail.

## License

esp8266 license terms are a whole kettle of fish.  The esp-open-rtos guys are gradually reverse-engineering the esp binary blobs and hopefully we will soon have a fully GPL-compatible SDK and it will be legal to distribute the compiled firmware images.
