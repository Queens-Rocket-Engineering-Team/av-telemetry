# Communications Firmware
This directory contains the firmware for the Avionics Communication System. The firmware in this directory should be mostly functional.

We currently use Arduino framework with arduino-esp32 (version 2.0.5) and STM32Duino core (version 2.7.1) for developing firmware for our modules. We flash firmware to the modules via usb/uart and SWD using STLinkV2.

## Attributions
Our firmware uses a variety of external libraries. Most libraries can be found through the Arduino library manager, and those that cannot have the corresponding GitHub link beside their include statements. We have taken care not to include any external libraries within our repository; only code we have written (or have modified and credited) are included here.
