import asyncio
from pyartnet import ArtNetNode

class CinemaRoomController:
    def __init__(self, ip='192.168.1.95', universe=0):
        self.ip = ip
        self.universe_id = universe
        self.step_channels = list(range(2, 16))
        self.channels = {
            "steps": list(range(2, 16)),
            "stars_intensitiy": 0,
            "stars_speed": 1,
            "stars_led": 21,
            "acoustic_panels": [16, 17, 18, 19, 20],
        }
        # Each effect writes its value here
        self.layers = {
            "ambient": 0,
            "sensor": 0,
            "manual": 0
            "max_ambient": 155,
            "min_ambient": 30,
        }
        self.ambient_intensity_sequence =[0, 255, 0]
        self.ambient_delay_sequence=[2, 2, 2]

    async def setup(self):
        self.node = ArtNetNode(self.ip)
        await self.node.start()
        print("DMX initialized.")
        self.universe = self.node.add_universe(universe)
        self._setup_steps()
        self._setup_panels()
        
    async def _setup_panels(self):
        self.panels = []
        for panel_n, channel in enumerate(self.channels["panels"]):
            panel = self.universe.add_channel(start=channel, width=1)
            panel.add_fade(0, 100)
            self.panels.append(panel)
        
    async def _setup_steps(self):
        self.steps = []
        for step_n, channel in enumerate(self.channels["steps"]):
            led = self.universe.add_channel(start=channel, width=1)  # 1 
            led.add_fade(0, 100)
            self.steps.append(led)
            
    async def _setup_stars(self):
        self.stars = {}
        self.stars["intensity"] = self.universe.add_channel(
            start=self.channels["stars_intensity"], width=1
        )
        self.stars["speed"] = 

    async def pulse_lights(self):
        while self._running:
            for value in self.ambient_intensity_sequence:
                self.layers["ambient"] = value
                await asyncio.sleep(2)




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