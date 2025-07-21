import asyncio
from pyartnet import ArtNetNode
import random
import logging
from pyartnet import ArtNetNode
import numpy as np
from PySide6.QtWidgets import QApplication, QWidget, QVBoxLayout, QSlider, QLabel, QPushButton
from PySide6.QtCore import Qt
import sys
import asyncio
import threading

class LightingUI(QWidget):
    def __init__(self, controller):
        super().__init__()
        self.controller = controller
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle("Cinema Lighting Control")

        layout = QVBoxLayout()

        self.label = QLabel("Step 0 Ambient Intensity: 0")
        layout.addWidget(self.label)

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMinimum(0)
        self.slider.setMaximum(255)
        self.slider.valueChanged.connect(self.slider_changed)
        layout.addWidget(self.slider)

        self.stop_button = QPushButton("Stop Lights")
        self.stop_button.clicked.connect(self.stop_lights)
        layout.addWidget(self.stop_button)

        self.setLayout(layout)

    def slider_changed(self, value):
        self.label.setText(f"Step 0 Ambient Intensity: {value}")
        self.controller.step_layers["ambient"][0] = value

    def stop_lights(self):
        self.controller._running = False

# Enable logging
# logging.basicConfig(level=logging.INFO)  # or INFO for less detail

class CinemaRoomController:
    def __init__(self, ip='192.168.1.191', universe_id=0, port=6454):
        self.ip = ip
        self.port = port
        self.global_fade = 1000
        self.universe_id = universe_id
        self.step_channels = list(range(3, 17))
        self.initial_levels = 30
        self.setup_delay = 0.1

        
        self.lineardriver_controller_mappings = {
            "steps": {
                "controller_n" : [0, 0, 0, 0, 1,1,1,1,2,2,2,2,3,3], 
                "local_channel_n": [0,1,2,3,0,1,2,3,0,1,2,3,0,1],
                "dmx_channel": list(range(3, 17)),
                },
            "panels": {
                "controller_n" : [4,4,4,4, 5,5,5,5], 
                "local_channel_n": [0,1,2,3,0,1,2,3],
                "dmx_channel": [17, 18, 19, 20, 21, 22, 23, 24],
                },
        }

        self.stairs_ambiant_pulse = [True] * 10 + [False] * 4
        self.channels = {
            "steps": list(range(3, 17)),
            "stars_intensity": 1,
            "stars_speed": 2,
            "panels": [17, 18, 19, 20, 21, 22],
        }
        # Each effect writes its value here
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

        self._running = True
        self.max_intensity = 165
        self.min_intensity = 0
        self.n_panels = 6
        self.n_steps = 14

        self.ambient_panel_intensity_max = 210
        self.ambient_panel_intensity_min = 150
        self.ambient_panel_delay_max = 5
        self.panel_intensity_sequences, self.panel_delay_sequences = self.get_rand_sequences(
            intensity_min=self.ambient_panel_intensity_min, 
            intensity_max=self.ambient_panel_intensity_max, 
            delay_max=self.ambient_panel_delay_max, 
            sequence_length=100, 
            n_sequences=self.n_panels
        )
        # self.ambient_panel_intensity_sequence = (self.ambient_panel_intensity_min + (self.ambient_panel_intensity_max - self.ambient_panel_intensity_min) * np.random.random(100)).astype(int).tolist()
        # self.ambient_delay_sequence = (np.random.random(100) * self.ambient_panel_delay_max).tolist()

        self.ambient_step_intensity_max = 100
        self.ambient_step_intensity_min = 10
        self.ambient_step_delay_max = 5
        self.stpes_intensity_sequences, self.steps_delay_sequences = self.get_rand_sequences(
            intensity_min=self.ambient_step_intensity_min, 
            intensity_max=self.ambient_panel_intensity_max, 
            delay_max=self.ambient_step_delay_max, 
            sequence_length=100, 
            n_sequences=self.n_steps
        )        
        
        
        
        
        # self.ambient_step_intensity_sequence = (self.ambient_step_intensity_min + (self.ambient_step_intensity_max - self.ambient_step_intensity_min) * np.random.random(100)).astype(int).tolist()
        # self.ambient_delay_sequence = (np.random.random(100) * self.ambient_step_delay_max).tolist()




        # self.panel_intensity_sequences = []
        # self.panel_delay_sequences = []
        # self.stpes_intensity_sequences = []
        # self.steps_delay_sequences = []
        # for panel_n in range(self.n_panels):
        #     # Create a shuffled copy
        #     ambient_intensity_sequence_copy = self.ambient_panel_intensity_sequence[:]
        #     random.shuffle(ambient_intensity_sequence_copy)
            
        #     ambient_delay_sequence_copy = self.ambient_delay_sequence[:]
        #     random.shuffle(ambient_delay_sequence_copy)

        #     self.panel_intensity_sequences.append(ambient_intensity_sequence_copy)
        #     self.panel_delay_sequences.append(ambient_delay_sequence_copy)

        # for panel_n in range(self.n_steps):
        #     # Create a shuffled copy
        #     ambient_intensity_sequence_copy = self.ambient_step_intensity_sequence[:]
        #     random.shuffle(ambient_intensity_sequence_copy)
            
        #     ambient_delay_sequence_copy = self.ambient_delay_sequence[:]
        #     random.shuffle(ambient_delay_sequence_copy)

        #     self.stpes_intensity_sequences.append(ambient_intensity_sequence_copy)
        #     self.steps_delay_sequences.append(ambient_delay_sequence_copy)


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

    async def _setup_stars(self):
        self.stars = {}
        self.stars["intensity"] = self.universe.add_channel(
            start=self.channels["stars_intensity"], width=1
        )
        self.stars["speed"] = self.universe.add_channel(
            start=self.channels["stars_speed"], width=1
        )
        self.stars["intensity"].set_values([255])  # White with dimmer
        await asyncio.sleep(self.setup_delay)
        self.stars["speed"].set_values([255])  # White with dimmer
        await asyncio.sleep(self.setup_delay)
        self.stars["speed"].set_values([0])  # White with dimmer
        await asyncio.sleep(self.setup_delay)
        self.stars["intensity"].set_values([0])  # White with dimmer
        await asyncio.sleep(self.setup_delay)

    async def update_dmx(self):
        while self._running:
            panel_intensities = self.mix_intensity_panels()
            step_intensities = self.mix_intensity_steps()
        
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

            
            for controller_n, controller in enumerate(self.linear_drive_dmx_controllers):
                controller.add_fade(controller_intensities[controller_n], self.global_fade)
                await asyncio.sleep(0.1)  # <-- Needed to pulse and allow fading
    

    def mix_intensity_panels(self):
        # Combine values from each layer (choose strategy):

        intensities = []
        for panel_n in range(self.n_panels):
            layers = {
                "ambient":self.panel_layers["ambient"][panel_n],
                "sensor": self.panel_layers["sensor"][panel_n],
                "manual": self.panel_layers["manual"][panel_n],
            }
            intensities.append( max(layers.values()))
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
            intensities.append( max(layers.values()))
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

    async def pulse_step_intensities(self):
        tasks = []
        for step_n in range(self.n_steps):
            if self.stairs_ambiant_pulse[step_n]:
                tasks.append(asyncio.create_task(self.pulse_step_intensity(step_n)))
        
# :       
# tasks = [
#             asyncio.create_task(self.pulse_step_intensity(step_n))
#             for step_n in range(self.n_steps)
#         ]
        await asyncio.gather(*tasks)

    async def pulse_panel_intensity(self, panel_n=0):
        # pulse a signal panel
        while self._running:
            for intensity, delay in zip(self.panel_intensity_sequences[panel_n], self.panel_delay_sequences[panel_n]):
                self.panel_layers["ambient"][panel_n] = intensity
                print(intensity)
                await asyncio.sleep(delay)
                

    async def pulse_step_intensity(self, panel_n=0):
        # pulse a signal panel
        while self._running:
            for intensity, delay in zip(self.stpes_intensity_sequences[panel_n], self.steps_delay_sequences[panel_n]):
                self.step_layers["ambient"][panel_n] = intensity
                print(self.step_layers["ambient"])
                await asyncio.sleep(delay)
                
    async def run(self):
        await self.setup()
        await asyncio.gather(
            self.pulse_panels_intensities(),
            self.pulse_step_intensities(),
            self.update_dmx(),
        )
def start_asyncio_loop(controller):
    asyncio.run(controller.run())


if __name__ == "__main__":
    controller = CinemaRoomController()
    # Start asyncio loop in background thread
    threading.Thread(target=start_asyncio_loop, args=(controller,), daemon=True).start()

    app = QApplication(sys.argv)
    ui = LightingUI(controller)
    ui.show()
    sys.exit(app.exec())

    # try:
    #     asyncio.run(controller.run())
    # except KeyboardInterrupt:
    #     print("Shutting down gracefully...")

