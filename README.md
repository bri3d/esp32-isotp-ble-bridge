# ISOTP BLE Bridge

The goal of this project is to build a native Macchina A0 (or ESP32 clone) firmware which can bridge BLE to ISOTP. The A0 is increasingly more difficult to find in stock, an alternative is to purchase the individual dev boards and assemble your own "clone".

Building and installing
-----------------------
1) This project is built using the ESP32 native toolchain ESP-IDF (based on FreeRTOS) and can be compiled using `idf.py build`.

2) Or download and flash precompiled firmware:
https://github.com/Switchleg1/esp32-isotp-ble-bridge/tree/BridgeLEG/release

# Supported software

1) SimosTools, android based ecu flashing and logging software:
https://play.google.com/store/apps/details?id=com.app.simostools

2) VW_Flash, python based ecu flashing and logging software:
https://github.com/bri3d/VW_Flash

# Supported hardware

1) Genuine Macchina A0
https://www.macchina.cc/catalog/a0-boards/a0-under-dash

2) DIY - ESP32 dev board "clone"
Parts list:
Any ESP32 dev board - https://www.amazon.ca/KeeYees-Development-Bluetooth-Microcontroller-ESP-WROOM-32/dp/B07PP1R8YK/ref=sr_1_20?crid=22P4LA1F8VBNC&keywords=esp32&qid=1643595911&sprefix=esp32%2Caps%2C105&sr=8-20
SN65HVD230 breakout board - https://www.amazon.ca/CloverUS-SN65HVD230-Transceiver-Communication-Arduino/dp/B07XDNNLZ5/ref=sr_1_1?crid=2CQQY3K3UYASM&keywords=SN65HVD230&qid=1643596755&sprefix=sn65hvd230%2Caps%2C88&sr=8-1
Small DC buck converter to 5v - https://www.amazon.ca/eBoot-MP1584EN-Converter-Adjustable-Module/dp/B01MQGMOKI/ref=sr_1_9?crid=192T7TLTFMKHE&keywords=buck+converter&qid=1643596906&sprefix=buck+converter%2Caps%2C105&sr=8-9
OBD2 connector - https://www.sparkfun.com/products/9911
