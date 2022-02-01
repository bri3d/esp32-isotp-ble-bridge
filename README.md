# ISOTP BLE Bridge

The goal of this project is to build a native Macchina A0 (or ESP32 clone) firmware which can bridge BLE to ISOTP. The A0 has become increasingly more difficult to find in stock as of 2022, an alternative is to purchase the individual dev boards and assemble your own "clone".

Building and installing:
1) This project is built using the ESP32 native toolchain ESP-IDF (based on FreeRTOS) and can be compiled using `idf.py build`

2) Download and flash precompiled firmware: <br>
https://github.com/Switchleg1/esp32-isotp-ble-bridge/releases/tag/v0.25

# Supported software

1) SimosTools, android based ecu flashing and logging software: <br>
https://play.google.com/store/apps/details?id=com.app.simostools

2) VW_Flash, python based ecu flashing and logging software: <br>
https://github.com/bri3d/VW_Flash

# Supported hardware

1) Genuine Macchina A0 <br>
https://www.macchina.cc/catalog/a0-boards/a0-under-dash <br>

2) DIY - ESP32 dev board "clone" <br>
Parts list (quantity 1 of each): <br>
Any ESP32 dev board <br>
https://www.amazon.ca/KeeYees-Development-Bluetooth-Microcontroller-ESP-WROOM-32/dp/B07PP1R8YK/ref=sr_1_20?crid=22P4LA1F8VBNC&keywords=esp32&qid=1643595911&sprefix=esp32%2Caps%2C105&sr=8-20 <br><br>
SN65HVD230 breakout board <br>
https://www.amazon.ca/CloverUS-SN65HVD230-Transceiver-Communication-Arduino/dp/B07XDNNLZ5/ref=sr_1_1?crid=2CQQY3K3UYASM&keywords=SN65HVD230&qid=1643596755&sprefix=sn65hvd230%2Caps%2C88&sr=8-1 <br><br>
Any MP1584 based buck converter <br>
https://www.amazon.ca/eBoot-MP1584EN-Converter-Adjustable-Module/dp/B01MQGMOKI/ref=sr_1_9?crid=192T7TLTFMKHE&keywords=buck+converter&qid=1643596906&sprefix=buck+converter%2Caps%2C105&sr=8-9 <br><br>
OBD2 connector (can also be taken from an elm adapter) <br>
https://www.sparkfun.com/products/9911 <br><br>
Schottky or fast recovery diode (1Amp 20v) 1N5819, SR100, UF4004, etc <br>
https://www.amazon.ca/1N5819-Schottky-Barrier-Rectifier-DO-204AL/dp/B07RL3SCL4/ref=mp_s_a_1_5?crid=AX2MPSXQXPUM&keywords=1+amp+schottky+diode&qid=1643646733&sprefix=1amp+schottky+diode%2Caps%2C267&sr=8-5
