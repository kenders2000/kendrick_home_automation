import asyncio
from pyartnet import ArtNetNode

async def main():
    node = ArtNetNode('192.168.1.191', port=6454)
    universe = node.add_universe(0)
    channel = universe.add_channel(start=3, width=4)
    channel.set_values([255, 255, 255, 255])  # White with dimmer
    await asyncio.sleep(2)
    channel.set_values([0, 0, 0, 0])  # White with dimmer
    await asyncio.sleep(2)



    channel = universe.add_channel(start=1, width=1)
    channel.set_values([255])  # White with dimmer
    await asyncio.sleep(5)
    channel.set_values([0])  # White with dimmer
    await asyncio.sleep(5)

    # channel.set_values([0])  # White with dimmer
    # await asyncio.sleep(2)
    # for ch in range(3, 16):  # Scan channels 1–32
    #     channel = universe.add_channel(start=ch, width=1)
    #     channel.set_values([255])
    #     print(f"Flashing channel {ch}")
    #     await asyncio.sleep(0.5)
    #     channel.set_values([0])
    #     await asyncio.sleep(0.1)

asyncio.run(main())