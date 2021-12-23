import sys
import logging
import asyncio
import platform

from bleak import BleakClient
from bleak import BleakScanner
from bleak import _logger as logger

NOTIFY_CHARACTERISTIC_UUID = "0000abf2-0000-1000-8000-00805f9b34fb"  # <--- Change to the characteristic you want to enable notifications from.
WRITE_CHARACTERISTIC_UUID = "0000abf1-0000-1000-8000-00805f9b34fb"  # <--- Change to the characteristic you want to enable notifications from.

def notification_handler(sender, data):
    """Simple notification handler which prints the data received."""
    print("{0}: {1}".format(sender, data))

async def find_device_address():
    devices = await BleakScanner.discover()
    for d in devices:
        if d.name == "BLE_TO_ISOTP":
            return d.address

async def run(debug=False):
    if debug:
        import sys
        l = logging.getLogger("asyncio")
        l.setLevel(logging.DEBUG)
        h = logging.StreamHandler(sys.stdout)
        h.setLevel(logging.DEBUG)
        l.addHandler(h)
        logger.addHandler(h)

    address = await find_device_address()
    async with BleakClient(address) as client:
        logger.info(f"Connected: {client.is_connected}")
        services = await client.get_services()
        for service in services:
            print("service: ", service)
            for characteristic in service.characteristics:
                print("characteristic: ", characteristic)
        await client.start_notify(NOTIFY_CHARACTERISTIC_UUID, notification_handler)
        while(True):
            # TODO: BLE command 0x06 configure_isotp_link
            await client.write_gatt_char(WRITE_CHARACTERISTIC_UUID, bytes([
                0xe0, 0x07, 0x00, 0x00, # ECU RX_ID
                0xe8, 0x07, 0x00, 0x00, # ECU TX_ID
                0x22, 0xF1, 0x90 # UDS $22 F190 - read VIN
            ]))
            await asyncio.sleep(0.1)
        await asyncio.sleep(10)
        await client.stop_notify(NOTIFY_CHARACTERISTIC_UUID)


if __name__ == "__main__":
    import os
    os.environ["PYTHONASYNCIODEBUG"] = str(1)
    loop = asyncio.get_event_loop()
    # loop.set_debug(True)
    loop.run_until_complete(run(True))
