# Standard Library Imports
import os
import sys
import time
import asyncio
import threading
import random
import logging

# Third-Party Imports
import numpy as np
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from pyartnet import ArtNetNode

# Local Module Imports
sys.path.append("..")  # Add parent directory for module imports
from tof import tof
from utilities import ExponentialAverager, asymmetric_gaussian

# Optional GUI/Plotting Imports (commented for headless systems)
# from PySide6.QtWidgets import QApplication, QWidget, QVBoxLayout, QSlider, QLabel, QPushButton
# from PySide6.QtCore import Qt
# import matplotlib.pyplot as plt

# x = np.linspace(0,14)
# y= asymmetric_gaussian(x, mu=6, sigma_left=0.1, sigma_right=6.0)
# plt.plot(x,y)
# plt.show()

class StepTOFController:
    def __init__(self, n_steps=14, start_pos_m=0.5, stop_pos_m=4.3, sigma_front=0.1, sigma_back=4.0, smoothing_alpha=0.1):
        """
        Initialize a controller for smoothing ToF sensor readings into step intensities.

        Args:
            n_steps (int): Total number of steps.
            start_pos_m (float): Distance (in meters) at bottom step.
            stop_pos_m (float): Distance (in meters) at top step.
            sigma_front (float): Gaussian spread in the movement direction.
            sigma_back (float): Gaussian spread against the movement direction.
            smoothing_alpha (float): Smoothing factor for exponential averager.
        """
        self.n_steps = n_steps
        self.start_pos_m = start_pos_m
        self.stop_pos_m = stop_pos_m
        self.sigma_front = sigma_front
        self.sigma_back = sigma_back
        self.smoothing = ExponentialAverager(alpha=smoothing_alpha)
        self.reset()

    def reset(self):
        """Reset controller state."""
        self.intensities = np.zeros(self.n_steps, dtype=int)
        self.smoothing.reset()

    def get_step_position(self, distance_m):
        """Convert physical distance to step index (can be fractional)."""
        span = self.stop_pos_m - self.start_pos_m
        step_position = self.n_steps * (distance_m - self.start_pos_m) / span
        return np.clip(step_position, 0, self.n_steps - 1)

    def asymmetric_gaussian_profile(self, step_position, direction="down"):
        """Return an asymmetric Gaussian profile centered at the given step index."""
        x = np.arange(self.n_steps)
        if direction == "down":
            return asymmetric_gaussian(x, mu=step_position, sigma_left=self.sigma_front, sigma_right=self.sigma_back)
        elif direction == "up":
            return asymmetric_gaussian(x, mu=step_position, sigma_left=self.sigma_back, sigma_right=self.sigma_front)
        else:
            raise ValueError(f"Invalid direction '{direction}', expected 'up' or 'down'.")

    def get_intensities(self, distance_m, direction="down"):
        """
        Calculate smoothed step intensities based on current distance measurement.

        Args:
            distance_m (float): Measured distance in meters.
            direction (str): Movement direction, 'up' or 'down'.

        Returns:
            np.ndarray: Smoothed intensity array of shape (n_steps,).
        """
        if distance_m is None:
            return np.zeros(self.n_steps, dtype=int)

        step_pos = self.get_step_position(distance_m)
        raw_intensities = (self.asymmetric_gaussian_profile(step_pos, direction) * 255).astype(int)
        smoothed = self.smoothing.update(raw_intensities).astype(int)
        self.intensities = smoothed

        return self.intensities


class CinemaRoomController:
    def __init__(self, ip='192.168.1.191', universe_id=0, port=6454, system="mac"):
        self.ip = ip
        self.port = port
        self.global_fade = 500
        self.universe_id = universe_id
        self.setup_delay = 1.0
        self.system = system
        if system == "pi":
            self.tof = tof()
            self.step_controller = StepTOFController()
        elif system == "mac":
            self.tof = None
            self.step_controller = None
        
        self.lineardriver_controller_mappings = {
            "steps": {
                "controller_n" : [0, 0, 0, 0, 1,1,1,1,2,2,2,2,3,3], 
                "local_channel_n": [0,1,2,3,0,1,2,3,0,1,2,3,0,1],
                "dmx_channel": list(range(3, 17)),
                },
            "panels": {
                "controller_n" : [4,4,4,4, 5,5,5,5], 
                "local_channel_n": [0,1,2,3,0,1,2,3],
                "dmx_channel": [19, 20, 21, 22, 23, 24, 25, 26],
                },
        }

        self.stairs_ambiant_pulse = [True] * 14 # + [False] * 2
        self.channels = {
            "steps": list(range(3, 18)),
            "stars_intensity": 1,
            "stars_speed": 2,
            "panels": [19, 20, 21, 22, 23, 24],
        }
        # Each layer stores intensity values for 
        self.panel_layers = {
            "ambient": [0,0,0,0,0,0],
            "sensor": [0,0,0,0,0,0],
            "manual": [0,0,0,0,0,0],
        }

        self.step_layers = {
            "ambient": [0 for _ in range(14)],
            "sensor":  [0 for _ in range(14)],
            "manual": [0 for _ in range(14)],
        }
        self.star_layers = {
            "ambient": [255],
            "sensor":  [0],
            "manual": [0],
        }

        self._running = True
        self.n_panels = 6
        self.n_steps = 14
        self.ambient_multiplier = 1.0

        self.ambient_panel_intensity_max = 210
        self.ambient_panel_intensity_min = 75
        self.ambient_panel_delay_max = 3
        self.panel_intensity_sequences, self.panel_delay_sequences = self.get_rand_sequences(
            intensity_min=self.ambient_panel_intensity_min, 
            intensity_max=self.ambient_panel_intensity_max, 
            delay_max=self.ambient_panel_delay_max, 
            sequence_length=100, 
            n_sequences=self.n_panels
        )
 

        self.ambient_step_intensity_max = 20
        self.ambient_step_intensity_min = 1
        self.ambient_step_delay_max = 5
        self.steps_intensity_sequences, self.steps_delay_sequences = self.get_rand_sequences(
            intensity_min=self.ambient_step_intensity_min, 
            intensity_max=self.ambient_panel_intensity_max, 
            delay_max=self.ambient_step_delay_max, 
            sequence_length=100, 
            n_sequences=self.n_steps
        )     

        self.ambient_star_intensity_max = 255
        self.ambient_star_intensity_min = 0
        self.ambient_star_delay_max = 5
        self.stars_intensity_sequences, self.stars_delay_sequences = self.get_rand_sequences(
            intensity_min=self.ambient_star_intensity_min, 
            intensity_max=self.ambient_panel_intensity_max, 
            delay_max=self.ambient_star_delay_max, 
            sequence_length=100, 
            n_sequences=1
        )     
                
    def set_ambient_multiplier(self, ambient_multiplier):
        self.ambient_multiplier = ambient_multiplier

    def get_rand_sequences(self, intensity_min=30, intensity_max=120, delay_max=3, sequence_length=100, n_sequences=1):
        intensity_sequences = []
        delay_sequences = []
        intensity_sequence = (intensity_min + (intensity_max - intensity_min) * np.random.random(sequence_length)).astype(int).tolist()
        delay_sequence = (np.random.random(100) * delay_max).tolist()
        for panel_n in range(n_sequences):
            # Create a shuffled copy
            ambient_intensity_sequence_copy = intensity_sequence[:]
            random.shuffle(ambient_intensity_sequence_copy)
            
            ambient_delay_sequence_copy = delay_sequence[:]
            random.shuffle(ambient_delay_sequence_copy)

            intensity_sequences.append(ambient_intensity_sequence_copy)
            delay_sequences.append(ambient_delay_sequence_copy)
        return intensity_sequences, delay_sequences

    async def detect_distance(self):
        while self._running:

            distance = self.tof.get_distance()  # ✅ now it's sync
            print("ToF distance:", distance)
            self.step_intensities = self.step_controller.get_intensities(distance)
            print("Step intensities:", self.step_intensities)
            self.step_layers["sensor"] = self.step_intensities
            await asyncio.sleep(0.01)
            # return distance

    async def _setup_linear_drive_dmx_controllers(self):
        linear_drive_dmx_controllers = []
        start_channel = 3
        for controller_n in range(0, 6):
            channel = self.universe.add_channel(start=start_channel + (controller_n * 4), width=4)
            channel.set_values([255, 255, 255, 255])  # White with dimmer
            await asyncio.sleep(self.setup_delay)
            channel.set_values([0, 0, 0, 0])  # White with dimmer
            await asyncio.sleep(self.setup_delay)
            linear_drive_dmx_controllers.append(channel)
        return linear_drive_dmx_controllers
    
    async def setup(self):
        self.node = ArtNetNode(self.ip, port=self.port)
        print(f"Connecting to {self.ip}")
        print(f"Connected to {self.ip}")
        print("Artnet node initialized.")
        self.universe = self.node.add_universe(self.universe_id)
        self.stars = await self._setup_stars()
        self.linear_drive_dmx_controllers = await self._setup_linear_drive_dmx_controllers()
        if self.system == "pi":
            self.step_controller.reset()

    async def _setup_stars(self):
        star_controllers = {}
        star_controllers["intensity"] = self.universe.add_channel(
            start=self.channels["stars_intensity"], width=1
        )
        star_controllers["speed"] = self.universe.add_channel(
            start=self.channels["stars_speed"], width=1
        )
        star_controllers["intensity"].set_values([255])  # White with dimmer
        await asyncio.sleep(self.setup_delay)
        star_controllers["speed"].set_values([255])  # White with dimmer
        await asyncio.sleep(self.setup_delay)
        star_controllers["speed"].set_values([0])  # White with dimmer
        await asyncio.sleep(self.setup_delay)
        star_controllers["intensity"].set_values([0])  # White with dimmer
        await asyncio.sleep(self.setup_delay)
        return star_controllers

    async def update_dmx(self):
        while self._running:
            panel_intensities = self.mix_intensity_panels()
            step_intensities = self.mix_intensity_steps()
            stars_intensity = self.mix_intensity_stars()
            stars_speed = stars_intensity.copy()

            # we have 6 linear_drive_dmx_controllers, each 4 channels
            controller_intensities = np.zeros((6,4)).astype(int)

            # steps
            for controller_n, local_channel_n, step_intensity in zip(
                self.lineardriver_controller_mappings["steps"]["controller_n"], 
                self.lineardriver_controller_mappings["steps"]["local_channel_n"],
                step_intensities
                ):
                controller_intensities[controller_n, local_channel_n] = step_intensity
            # panels
            for controller_n, local_channel_n, panel_intensity in zip(
                self.lineardriver_controller_mappings["panels"]["controller_n"], 
                self.lineardriver_controller_mappings["panels"]["local_channel_n"],
                panel_intensities
                ):
                controller_intensities[controller_n, local_channel_n] = panel_intensity

            # stars
            self.stars["intensity"].add_fade(stars_intensity, self.global_fade)
            self.stars["speed"].add_fade(stars_speed, self.global_fade)

            print("Step intensities:", step_intensities)
            print("Panel intensities:", panel_intensities)
            print("Star speed and intensity:", stars_intensity)

            for controller_n, controller in enumerate(self.linear_drive_dmx_controllers):
                controller.add_fade(controller_intensities[controller_n], self.global_fade)
                await asyncio.sleep(0.01)  # <-- Needed to pulse and allow fading
    

    def mix_intensity_panels(self):
        # Combine values from each layer (choose strategy):

        intensities = []
        for panel_n in range(self.n_panels):
            layers = {
                "ambient":self.panel_layers["ambient"][panel_n],
                "sensor": self.panel_layers["sensor"][panel_n],
                "manual": self.panel_layers["manual"][panel_n],
            }
            intensities.append( int(max(layers.values()) * self.ambient_multiplier) )
        return intensities
    
    def mix_intensity_stars(self):
        # Combine values from each layer (choose strategy):

        layers = {
            "ambient":self.star_layers["ambient"][0],
            "sensor": self.star_layers["sensor"][0],
            "manual": self.star_layers["manual"][0],
        }
        intensities = [ int(max(layers.values()) * self.ambient_multiplier) ]
        return intensities
    
    def mix_intensity_steps(self):
        # Combine values from each layer (choose strategy):
        intensities = []
        for panel_n in range(self.n_steps):
            layers = {
                "ambient": self.step_layers["ambient"][panel_n],
                "sensor": self.step_layers["sensor"][panel_n],
                "manual": self.step_layers["manual"][panel_n],
            }
            intensities.append( int(max(layers.values()) * self.ambient_multiplier) )
        return intensities

        # strongest wins
        # return int(sum(self.layers.values()[panel_n]) / len(self.layers))  # average
        # return min(255, self.layers["ambient"][panel_n] + self.layers["sensor"][panel_n])  # additive (capped)

    async def pulse_panels_intensities(self):
        tasks = [
            asyncio.create_task(self.pulse_panel_intensity(panel_n))
            for panel_n in range(self.n_panels)
        ]
        await asyncio.gather(*tasks)

    async def pulse_star_intensities(self):
        tasks = [
            asyncio.create_task(self.pulse_star())
        ]
        await asyncio.gather(*tasks)


    async def pulse_step_intensities(self):
        tasks = []
        for step_n in range(self.n_steps):
            if self.stairs_ambiant_pulse[step_n]:
                tasks.append(asyncio.create_task(self.pulse_step_intensity(step_n)))
        await asyncio.gather(*tasks)

    async def pulse_panel_intensity(self, panel_n=0):
        # pulse a signal panel
        while self._running:
            for intensity, delay in zip(self.panel_intensity_sequences[panel_n], self.panel_delay_sequences[panel_n]):
                self.panel_layers["ambient"][panel_n] = intensity
                await asyncio.sleep(delay)
                

    async def pulse_step_intensity(self, panel_n=0):
        # pulse a signal panel
        while self._running:
            for intensity, delay in zip(self.steps_intensity_sequences[panel_n], self.steps_delay_sequences[panel_n]):
                self.step_layers["ambient"][panel_n] = intensity
                await asyncio.sleep(delay)

    async def pulse_star(self):
        # pulse a signal panel
        while self._running:
            for intensity, delay in zip(self.stars_intensity_sequences[0], self.stars_delay_sequences[0]):
                self.star_layers["ambient"][0] = intensity
                await asyncio.sleep(delay)

                

    async def run(self):

        await self.setup()
        tasks = [
            self.pulse_panels_intensities(),
            self.pulse_step_intensities(),
            self.pulse_star_intensities(),
            self.update_dmx(),
        ]

        # if self.system == "pi":
        #     tasks.append(self.detect_distance())

        await asyncio.gather(*tasks)

def start_asyncio_loop(controller):
    asyncio.run(controller.run())

# Top level instance (used by uvicorn)
app = FastAPI()

# Add CORS for Home Assistant or frontend calls
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)
system = os.getenv("SYSTEM")
system = "pi "if system is None else system

# Global controller instance
controller = CinemaRoomController(system=system)

@app.get("/ambient_multiplier/{value}")
def set_ambient_multiplier(value: float):
    controller.set_ambient_multiplier(value)
    print("Changing ambient leve", value)
    return {"status": "ok", "ambient_multiplier": value}

@app.on_event("startup")
def start_controller():
    thread = threading.Thread(target=start_asyncio_loop, args=(controller,), daemon=True)
    thread.start()

if __name__ == "__main__":
    pass
    # Start asyncio loop in background thread
    # threading.Thread(target=start_asyncio_loop, args=(controller,), daemon=True).start()

    # app = QApplication(sys.argv)
    # ui = LightingUI(controller)
    # ui.show()
    # sys.exit(app.exec())

    # try:
    #     asyncio.run(controller.run())
    # except KeyboardInterrupt:
    #     print("Shutting down gracefully...")

