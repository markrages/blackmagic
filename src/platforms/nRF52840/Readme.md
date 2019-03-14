# debugging with the nRF52840

## What it is

This is a port of Black Magic Probe to the nRF52840

## Requirements

Requires n5 SDK.

Requires nRF52840-DONGLE.  Although the nRF52 is really flexible with GPIO, so if you have different nRF52840 hardware it is probably a matter of editing platform.h to get your hardware supported.

## Installation

1. Install nrfutil.

2. Install nRF5x SDK.  Version 15.2.0 is known to work

3. Edit src/platforms/nRF52840/Makefile.inc to point SDK_ROOT to the SDK location.

4. Edit Makefile to put the serial number of your nRF52840 dongle.

## License

The license terms of the Nordic SDK aren't compatible with blackmagic
(clause 4 makes it non-free) so you cannot legally distribute the executable.
