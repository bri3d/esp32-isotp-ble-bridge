# ISOTP BLE Bridge

The goal of this project is to build a native Macchina A0 firmware which can bridge BLE to ISOTP.

This project is built using the ESP32 native toolchain ESP-IDF (based on FreeRTOS) and can be compiled using `idf.py build` (typically after `. $IDF_PATH/export.sh`). It can be flashed after building using `idf.py flash` and serial debugging logs can be viewed with `idf.py monitor`.

Currently, the project works, with a simple example client in the `client` directory. 

# Logs

Comment out `CONFIG_LOG_DEFAULT_LEVEL_INFO=y` and set `CONFIG_LOG_DEFAULT_LEVEL_DEBUG=y` in `sdkconfig` then recompile to turn on debug level logs.

# A few notes about Macchina A0

Clever, dead simple board - a CAN transceiver, voltage regulators, and an ESP32-WROVER module with ESP32 revision 3 core. Tons of resources to use, including 8MB of SPI SRAM, 16MB of Flash.

* GPIO 5 is CAN_TX
* GPIO 4 is CAN_RX
* GPIO 21 is attached to the "S" (Silent) pin on the CAN transceiver. It must be pulled LOW to allow the CAN transceiver to communicate.
* GPIO 13 switches power to the WS2812 LED
* GPIO 2 is the WS2812 LED control line.
* GPIO 35 is attached to voltage sense for the VIn. The multipliers and formula can be found here. https://github.com/rnd-ash/Macchina-J2534/blob/master/firmware/pt_device_a0.cpp#L30
* GPIO 32 (CS), 19 (DI), 18 (CLK), and 23 (DO) are supposedly attached to a W25N01GV 1Gbit (128MB!) flash chip in the schematic, but I think this is an option purchased separately - `A0 can also be custom ordered with additional solder down memory`


And that's about it!

ESP32 implements a clone of the NXP CAN transceiver, which has been renamed to TWAI, presumably due to copyright reasons or because the transceiver IP isn't actually licensed. The TWAI drivers built into "ESP-IDF" seem to work well.
