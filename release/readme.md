To flash firmware to the A0
---------------------------

If the A0 comes up in windows device manager but the drivers are not found you can download them at:
https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers

Option 1
---------
Download the A0_FW_Updater.zip and run updater.bat


Option 2
---------
Download the flash download tool from Espressif Systems.
https://www.espressif.com/sites/default/files/tools/flash_download_tool_3.9.0_0.zip

Download the 3 bin files in this folder.

Using the flash tool select the 3 bin files and set the following addresses:<br />
bootloader.bin    @ 0x1000<br />
partition-table   @ 0x8000<br />
isotp_ble_bridge  @ 0x10000<br />

Select the baud rate of 115200 and com port (usually 3) and click start.
