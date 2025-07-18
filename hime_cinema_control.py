import asyncio
from pyartnet import ArtNetNode

class StairLightingController:
    def __init__(self, ip='192.168.0.100', universe=0):
        self.ip = ip
        self.universe_id = universe
        self.num_steps = 14
        self.node = ArtNetNode(ip)
        self.universe = self.node.add_universe(universe)
        self.steps = []

        self._setup_lights()

    def _setup_lights(self):
        for i in range(self.num_steps):
            dmx_start = i * self.channels_per_step + 1
            dmx_channel = self.universe.add_channel(dmx_start, self.channels_per_step, byte_size=1)
            dmx_channel.add_fade(0, 0, 0)  # Initialize off
            self.steps.append(dmx_channel)

    async def set_step_color(self, step_idx, r, g, b, fade_time=0):
        if 0 <= step_idx < self.num_steps:
            await self.steps[step_idx].fade_to_value([r, g, b], fade_time)
    
    async def light_up_sequence(self, delay=0.1, color=(255, 255, 255), fade_time=0):
        for i in range(self.num_steps):
            await self.set_step_color(i, *color, fade_time)
            await asyncio.sleep(delay)

    async def light_down_sequence(self, delay=0.1, fade_time=0):
        for i in reversed(range(self.num_steps)):
            await self.set_step_color(i, 0, 0, 0, fade_time)
            await asyncio.sleep(delay)

    async def respond_to_depth_event(self, person_detected=True):
        if person_detected:
            await self.light_up_sequence()
        else:
            await self.light_down_sequence()

    async def blackout(self):
        for i in range(self.num_steps):
            await self.set_step_color(i, 0, 0, 0)

    async def run_forever(self):
        await self.node.start()
        print("DMX ArtNet node started.")
        try:
            while True:
                # Simulated event (replace with your depth sensor logic)
                await self.respond_to_depth_event(person_detected=True)
                await asyncio.sleep(3)
                await self.respond_to_depth_event(person_detected=False)
                await asyncio.sleep(3)
        except KeyboardInterrupt:
            await self.blackout()
            print("Shutting down...")