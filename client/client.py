import sys
import logging
import asyncio
import platform

from bleak import BleakClient
from bleak import _logger as logger


NOTIFY_CHARACTERISTIC_UUID = "0000abf2-0000-1000-8000-00805f9b34fb"  # <--- Change to the characteristic you want to enable notifications from.
WRITE_CHARACTERISTIC_UUID = "0000abf1-0000-1000-8000-00805f9b34fb"  # <--- Change to the characteristic you want to enable notifications from.

ADDRESS = (
    "24:71:89:cc:09:05"  # <--- Change to your device's address here if you are using Windows or Linux
    if platform.system() != "Darwin"
    else "5F3D5EFF-3BB9-40FA-8EAE-44D2DE1A2506"  # <--- Change to your device's address here if you are using macOS
)
if len(sys.argv) == 3:
    ADDRESS = sys.argv[1]
    CHARACTERISTIC_UUID = sys.argv[2]


def notification_handler(sender, data):
    """Simple notification handler which prints the data received."""
    print("{0}: {1}".format(sender, data))


async def run(address, debug=False):
    if debug:
        import sys

        l = logging.getLogger("asyncio")
        l.setLevel(logging.DEBUG)
        h = logging.StreamHandler(sys.stdout)
        h.setLevel(logging.DEBUG)
        l.addHandler(h)
        logger.addHandler(h)

    async with BleakClient(address) as client:
        logger.info(f"Connected: {client.is_connected}")
        services = await client.get_services()
        for service in services:
            print(service)
            for c in service.characteristics:
                print(c)
        await client.start_notify(NOTIFY_CHARACTERISTIC_UUID, notification_handler)
        while(True):
            await client.write_gatt_char(WRITE_CHARACTERISTIC_UUID, bytes([0x22, 0xF1, 0x90]))
            await asyncio.sleep(0.1)
        await asyncio.sleep(10)
        await client.stop_notify(NOTIFY_CHARACTERISTIC_UUID)


if __name__ == "__main__":
    import os

    os.environ["PYTHONASYNCIODEBUG"] = str(1)
    loop = asyncio.get_event_loop()
    # loop.set_debug(True)
    loop.run_until_complete(run(ADDRESS, True))
