# ISOTP BLE Bridge

The goal of this project is to build a native Macchina A0 firmware which can bridge BLE to ISOTP.

This project is built using the ESP32 native toolchain ESP-IDF (based on FreeRTOS) and can be compiled using `idf.py build`.

# Currently supported software:

Simos Logger, android based logger:
https://github.com/Switchleg1/Simos-Tools

VW_Flash, python based ecu flashing and logging software:
https://github.com/bri3d/VW_Flash

# A few notes about Macchina A0

Clever, dead simple board - a CAN transceiver, voltage regulators, and an ESP32-WROVER module with ESP32 revision 3 core. Tons of resources to use, including 8MB of SPI SRAM, 16MB of Flash.

* GPIO 5 is CAN_TX
* GPIO 4 is CAN_RX
* GPIO 21 is attached to the "S" (Silent) pin on the CAN transceiver. It must be pulled LOW to allow the CAN transceiver to communicate.
* GPIO 13 switches power to the WS2812 LED
* GPIO 2 is the WS2812 LED control line.

And that's about it!

ESP32 implements a clone of the NXP CAN transceiver, which has been renamed to TWAI, presumably due to copyright reasons or because the transceiver IP isn't actually licensed. The TWAI drivers built into "ESP-IDF" seem to work well.
