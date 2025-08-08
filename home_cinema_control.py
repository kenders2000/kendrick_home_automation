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
from fastapi import FastAPI, Query

from fastapi.middleware.cors import CORSMiddleware
from pyartnet import ArtNetNode
import sys
# from PySide6.QtWidgets import (
#     QApplication,
#     QWidget,
#     QVBoxLayout,
#     QSlider,
#     QLabel,
#     QPushButton,
#     QDoubleSpinBox,
# )
# from PySide6.QtCore import Qt, QTimer


# Local Module Imports
sys.path.append("..")  # Add parent directory for module imports
from tof import tof
from utilities import ExponentialAverager, asymmetric_gaussian, KalmanFilter1D


# Optional GUI/Plotting Imports (commented for headless systems)
# from PySide6.QtWidgets import QApplication, QWidget, QVBoxLayout, QSlider, QLabel, QPushButton
# from PySide6.QtCore import Qt
# import matplotlib.pyplot as plt

# x = np.linspace(0,14)
# y= asymmetric_gaussian(x, mu=6, sigma_left=0.1, sigma_right=6.0)
# plt.plot(x,y)
# plt.show()
import logging

logging.basicConfig(level=logging.INFO)


from pydantic import BaseModel
from typing import Optional


import json
from pathlib import Path

class SaveSettingsRequest(BaseModel):
    filename: str = "settings.json"


class ControllerSettings(BaseModel):
    ip: str = "192.168.1.191"
    universe_id: int = 0
    port: int = 6454
    system: str = "mac"

    global_fade: int = 600
    star_intensity_fade: int = 1000
    star_speed_fade: int = 500

    setup_delay: float = 0.1
    steps_wait_time: float = 0.1
    controller_delay: float = 0.1

    ambient_panel_intensity_max: int = 210
    ambient_panel_intensity_min: int = 75
    ambient_panel_delay_max: float = 10.0

    ambient_step_intensity_max: int = 50
    ambient_step_intensity_min: int = 5
    ambient_step_delay_max: float = 5.0

    ambient_star_intensity_max: int = 255
    ambient_star_intensity_min: int = 100
    ambient_star_delay_max: float = 5.0

    ambient_star_speed_max: int = 255
    ambient_star_speed_min: int = 0

    sequence_length: int = 100


def load_settings_from_file(path="settings.json") -> ControllerSettings:
    """
    Load controller settings from a JSON file and return a ControllerSettings object.

    Args:
        path (str): Path to the settings file.

    Returns:
        ControllerSettings: A validated settings object.
    """
    with open(path, "r") as f:
        data = json.load(f)
    return ControllerSettings(**data)


def save_settings_to_file(settings: ControllerSettings, path="settings.json"):
    """
    Save controller settings to a JSON file.

    Args:
        settings (ControllerSettings): Settings object to save.
        path (str): Output file path.
    """
    with open(path, "w") as f:
        f.write(settings.model_dump_json(indent=4))


class StepTOFController:
    def __init__(
        self,
        n_steps=14,
        step_top_position_m=0.5,
        step_bottom_position_m=5.5,
        step_sigma_front=0.5,
        step_sigma_back=10.0,
        step_intensity_smoothing_alpha=0.0,
        step_reset_time=10.0,
        steps_mid_position_threshold=2.0,
        steps_max_position_threshold=5.5,
        steps_min_position_threshold=1.0,
        distance_outlier_threshold=3.0,
        steps_top_initial_pos_for_kalman=1.0,
        steps_bottom_initial_pos_for_kalman=5.6,
        frame_time=0.1,
    ):
        """
        Initialize a controller for smoothing ToF sensor readings into step intensities.

        Args:
            n_steps (int): Total number of steps.
            step_top_position_m (float): Distance (in meters) at bottom step.
            step_bottom_position_m (float): Distance (in meters) at top step.
            step_sigma_front (float): Gaussian spread in the movement direction.
            step_sigma_back (float): Gaussian spread against the movement direction.
            step_intensity_smoothing_alpha (float): Smoothing factor for exponential averager.
            step_reset_time (float): Time in seconds to reset the step state if no person is detected.
            steps_mid_position_threshold (float): Distance threshold for mid-step position.
            steps_max_position_threshold (float): Distance threshold for maximum step position.
            steps_min_position_threshold (float): Distance threshold for minimum step position.
            distance_outlier_threshold (float): Threshold for distance outlier detection.
            steps_top_initial_pos_for_kalman (float): Initial position for top of steps.
            steps_bottom_initial_pos_for_kalman (float): Initial position for bottom of steps.
        """
        self.n_steps = n_steps
        self.step_top_position_m = step_top_position_m
        self.step_bottom_position_m = step_bottom_position_m
        self.step_sigma_front = step_sigma_front
        self.step_sigma_back = step_sigma_back
        self.step_intensity_smoothing_alpha = step_intensity_smoothing_alpha
        self.exp_smoother = ExponentialAverager(alpha=step_intensity_smoothing_alpha)
        self.step_reset_time = step_reset_time
        self.steps_mid_position_threshold = steps_mid_position_threshold
        self.steps_max_position_threshold = steps_max_position_threshold
        self.steps_min_position_threshold = steps_min_position_threshold
        self.distance_outlier_threshold = distance_outlier_threshold
        self.steps_top_initial_pos_for_kalman = steps_top_initial_pos_for_kalman
        self.steps_bottom_initial_pos_for_kalman = steps_bottom_initial_pos_for_kalman
        self.frame_time = frame_time
        self.reset()

    def initialise_kalman_filter(
        self,
        distance,
        max_speed=1,
    ):
        """Initialize the Kalman filter with the given distance.

        measurement_variance: The expected noise in your measurements (from the ToF sensor).
        - Lower value → more trust in the current measurement → faster tracking
        - Higher value → more trust in prediction → slower response
        - Decrease this to speed up the filter's response — but not too low, or you'll overreact to noisy spikes.
        process_variance: Your belief about how much the true distance can change between steps (i.e., system noise or model uncertainty).
        - Higher value → Kalman filter expects more change → faster adaptation
        - Lower value → assumes distance doesn’t change much → smoother but slower response
        - Increase this slightly to allow faster tracking of changes in distance.
        """
        # the max expected distance in a frame is the maximum speed multiplied by the frame time
        max_expected_distance_in_a_frame = max_speed * self.frame_time

        self.kalman_filter = KalmanFilter1D(
            initial_state=distance,
            initial_uncertainty=1.0,
            process_variance=max_expected_distance_in_a_frame,
            measurement_variance=0.2,
            outlier_threshold=self.distance_outlier_threshold,
        )

    def update_distance(self, new_distance, time_since_last_update):
        self.distance = new_distance
        self.update_steps_state(new_distance, time_since_last_update)

    def get_smoothed_distance(self):
        self.kalman_filter.predict()
        self.smoothed_distance = self.kalman_filter.update(self.distance)
        return self.smoothed_distance

    def update_steps_state(self, distance, time):
        if (
            self.steps_person_state == "down"
            and self.countdown_no_person_state < self.step_reset_time
            and distance < self.steps_max_position_threshold
        ):
            self.countdown_no_person_state += time
        elif (
            self.steps_person_state == "up"
            and self.countdown_no_person_state < self.step_reset_time
            and distance < self.steps_max_position_threshold
        ):
            self.countdown_no_person_state += time
        elif self.countdown_no_person_state < self.step_reset_time:
            # if the timer has not expired, increment the counter
            self.countdown_no_person_state += time
        else:
            # if the timer has expired, reset the counter and reset the steps state to no persons detected
            self.countdown_no_person_state = 0.0
            self.steps_person_state = "no_person"
            self.initialise_kalman_filter(0.0)

        if (
            distance > self.steps_min_position_threshold
            and distance < self.steps_mid_position_threshold
            and self.steps_person_state == "no_person"
        ):
            self.steps_person_state = "down"  # top or bottom
            self.countdown_no_person_state = 0.0
            self.initialise_kalman_filter(distance)

        elif (
            distance > self.steps_mid_position_threshold
            and distance < self.steps_max_position_threshold
            and self.steps_person_state == "no_person"
        ):
            self.steps_person_state = "up"  # top or bottom
            self.countdown_no_person_state = 0.0
            self.initialise_kalman_filter(distance)
        logging.info(
            f"person state: {self.steps_person_state} counter for reset time: {self.countdown_no_person_state}"
        )

    def reset(self):
        """Reset controller state."""
        self.intensities = np.zeros(self.n_steps, dtype=int)
        self.exp_smoother.reset()
        self.steps_person_state = "no_person"  # top or bottom
        self.countdown_no_person_state = 0.0
        self.initialise_kalman_filter(0.0)

    def get_step_position(self, distance_m):
        """Convert physical distance to step index (can be fractional)."""
        span = self.step_bottom_position_m - self.step_top_position_m
        step_position = self.n_steps * (distance_m - self.step_top_position_m) / span
        return np.clip(step_position, 0, self.n_steps - 1)

    def asymmetric_gaussian_profile(self, step_position, direction="down"):
        """Return an asymmetric Gaussian profile centered at the given step index."""
        x = np.arange(self.n_steps)
        if direction == "up":
            return asymmetric_gaussian(
                x,
                mu=step_position,
                sigma_left=self.step_sigma_front,
                sigma_right=self.step_sigma_back,
            )
        elif direction == "down":
            return asymmetric_gaussian(
                x,
                mu=step_position,
                sigma_left=self.step_sigma_back,
                sigma_right=self.step_sigma_front,
            )
        else:
            raise ValueError(
                f"Invalid direction '{direction}', expected 'up' or 'down'."
            )

    def get_intensities(self, distance_m):
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
        if self.steps_person_state == "no_person":
            return np.zeros(self.n_steps).astype(int)
        self.intensities = (
            self.asymmetric_gaussian_profile(
                step_pos, direction=self.steps_person_state
            )
            * 255
        ).astype(int)
        if self.step_intensity_smoothing_alpha > 0.0:
            self.intensities = self.exp_smoother.update(self.intensities).astype(int)

        return self.intensities


class CinemaRoomController:
    def __init__(
        self,
        ip="192.168.1.191",
        universe_id=0,
        port=6454,
        system="mac",
        global_fade=600,
        star_intensity_fade=1000,
        star_speed_fade=500,
        setup_delay=0.1,
        steps_wait_time=0.1,
        controller_delay=0.1,
        ambient_panel_intensity_max=210,
        ambient_panel_intensity_min=75,
        ambient_panel_delay_max=10,
        ambient_step_intensity_max=50,
        ambient_step_intensity_min=5,
        ambient_step_delay_max=5,
        ambient_star_intensity_max=255,
        ambient_star_intensity_min=100,
        ambient_star_delay_max=5,
        ambient_star_speed_max=255,
        ambient_star_speed_min=0,
        sequence_length=100,
        manual_distance_override=None,
        initialise=False,
    ):
        """
        Initialise the cinema room controller.
        Initialises controller parameters, DMX channel mapping, intensity layers, and random ambient lighting sequences.

        Args:
            ip (str): IP address of the controller. Defaults to "192.168.1.191".
            universe_id (int): DMX universe ID. Defaults to 0.
            port (int): Port number for communication. Defaults to 6454.
            system (str): System type, either "mac" or "pi". Defaults to "mac".
            global_fade (int): Global fade time in milliseconds. Defaults to 600.
            star_intensity_fade (int): Fade time for star intensity in milliseconds. Defaults to 1000.
            star_speed_fade (int): Fade time for star speed in milliseconds. Defaults to 500.
            setup_delay (float): Delay after setup in seconds. Defaults to 0.1.
            steps_wait_time (float): Wait time between step updates in seconds. Defaults to 0.1.
            controller_delay (float): Delay for controller loop in seconds. Defaults to 0.1, this is the update rate of the controllers
                this doesnt change the random delays between panels, just overall update rate of the RBBW controllers.
            ambient_panel_intensity_max (int): Maximum intensity for ambient panels. Defaults to 210.
            ambient_panel_intensity_min (int): Minimum intensity for ambient panels. Defaults to 75.
            ambient_panel_delay_max (int): Maximum delay for ambient panels in seconds. Defaults to 10.
            ambient_step_intensity_max (int): Maximum intensity for ambient steps. Defaults to 50.
            ambient_step_intensity_min (int): Minimum intensity for ambient steps. Defaults to 5.
            ambient_step_delay_max (int): Maximum delay for ambient steps in seconds. Defaults to 5.
            ambient_star_intensity_max (int): Maximum intensity for ambient stars. Defaults to 255.
            ambient_star_intensity_min (int): Minimum intensity for ambient stars. Defaults to 100.
            ambient_star_delay_max (int): Maximum delay for ambient stars in seconds. Defaults to 5.
            ambient_star_speed_max (int): Maximum speed for ambient stars. Defaults to 255.
            ambient_star_speed_min (int): Minimum speed for ambient stars. Defaults to 0.
            sequence_length (int): Length of the random intensity sequence for ambient lighting. Defaults to 100.
            manual_distance_override (float, optional): Manual override value for distance. Defaults to None.
        """
        self.ip = ip
        self.port = port
        self.global_fade = global_fade
        self.star_intensity_fade = star_intensity_fade
        self.star_speed_fade = star_speed_fade
        self.universe_id = universe_id
        self.setup_delay = setup_delay
        self.steps_wait_time = steps_wait_time
        self.controller_delay = controller_delay
        self.system = system
        self.step_controller = StepTOFController(frame_time=self.steps_wait_time)
        self._running = True
        self.n_panels = 6
        self.n_steps = 14
        # initialise the ambient multiplier at full intensity
        self.ambient_multiplier = 1.0
        self.ambient_panel_intensity_max = ambient_panel_intensity_max
        self.ambient_panel_intensity_min = ambient_panel_intensity_min
        self.ambient_panel_delay_max = ambient_panel_delay_max
        self.ambient_step_intensity_max = ambient_step_intensity_max
        self.ambient_step_intensity_min = ambient_step_intensity_min
        self.ambient_step_delay_max = ambient_step_delay_max
        self.ambient_star_intensity_max = ambient_star_intensity_max
        self.ambient_star_intensity_min = ambient_star_intensity_min
        self.ambient_star_delay_max = ambient_star_delay_max
        self.ambient_star_speed_max = ambient_star_speed_max
        self.ambient_star_speed_min = ambient_star_speed_min
        self.sequence_length = sequence_length
        self.manual_distance_override = manual_distance_override
        if system == "pi":
            self.tof = tof()
        elif system == "mac":
            self.tof = None
        if initialise:
            # initialise the dmx channels this sets the mapping for the DMX channels
            self.initialise_dmx_channels()
            # Generate random intensity sequences for ambient lighting
            self.get_random_intensity_sequences()
            # initialise the intensity layers (these allow seperate control of the light level via ambiant, sensor, and manual control)
            self.initialise_layers()

    def get_random_intensity_sequences(self):
        """
        Generates and assigns random intensity and delay sequences for panels, steps, and stars.

        This method calls `get_rand_sequences` multiple times with different parameters to generate
        random sequences for:
            - Panel intensities and delays
            - Step intensities and delays
            - Star intensities and delays
            - Star positions and their delays

        The generated sequences are stored in the following instance attributes:
            - panel_intensity_sequences, panel_delay_sequences
            - steps_intensity_sequences, steps_delay_sequences
            - stars_intensity_sequences, stars_delay_sequences
            - stars_position_sequences, stars_position_delay_sequences

        Each call to `get_rand_sequences` uses the corresponding min/max intensity and delay values,
        sequence length, and number of sequences as defined by the instance attributes.
        """
        self.panel_intensity_sequences, self.panel_delay_sequences = (
            self.get_rand_sequences(
                intensity_min=self.ambient_panel_intensity_min,
                intensity_max=self.ambient_panel_intensity_max,
                delay_max=self.ambient_panel_delay_max,
                sequence_length=self.sequence_length,
                n_sequences=self.n_panels,
            )
        )

        self.steps_intensity_sequences, self.steps_delay_sequences = (
            self.get_rand_sequences(
                intensity_min=self.ambient_step_intensity_min,
                intensity_max=self.ambient_step_intensity_max,
                delay_max=self.ambient_step_delay_max,
                sequence_length=self.sequence_length,
                n_sequences=self.n_steps,
            )
        )

        self.stars_intensity_sequences, self.stars_delay_sequences = (
            self.get_rand_sequences(
                intensity_min=self.ambient_star_intensity_min,
                intensity_max=self.ambient_star_intensity_max,
                delay_max=self.ambient_star_delay_max,
                sequence_length=self.sequence_length,
                n_sequences=1,
            )
        )

        self.stars_position_sequences, self.stars_position_delay_sequences = (
            self.get_rand_sequences(
                intensity_min=self.ambient_star_intensity_min,
                intensity_max=self.ambient_star_intensity_max,
                delay_max=self.ambient_star_delay_max,
                sequence_length=self.sequence_length,
                n_sequences=1,
            )
        )

    def initialise_layers(self):
        """
        Initializes the lighting control layers for the home cinema system.

        This method sets up three separate control layers—'ambient', 'sensor', and 'manual'—
        for three different lighting zones: panel, step, and star. Each layer is represented
        as a dictionary with keys for each control type, and values as lists of intensity values.

        - panel_layers: Contains intensity values for 6 panel lights.
        - step_layers: Contains intensity values for 14 step lights.
        - star_layers: Contains intensity values for 2 star lights.

        The 'ambient' layer is initialized with default values (e.g., full brightness for star lights),
        while 'sensor' and 'manual' layers are initialized to zero.
        """
        # Each layer stores intensity values for ambient, sensor, and manual control
        self.panel_layers = {
            "ambient": [0, 0, 0, 0, 0, 0],
            "sensor": [0, 0, 0, 0, 0, 0],
            "manual": [0, 0, 0, 0, 0, 0],
        }
        self.step_layers = {
            "ambient": [0 for _ in range(14)],
            "sensor": [0 for _ in range(14)],
            "manual": [0 for _ in range(14)],
        }
        self.star_layers = {
            "ambient": [255, 255],
            "sensor": [0, 0],
            "manual": [0, 0],
        }

    async def shutdown(self):
        """Force all channels to 0 before teardown."""
        for controller_n, controller in enumerate(
            self.linear_drive_dmx_controllers
        ):
            controller.set_values([0, 0, 0, 0])
            await asyncio.sleep(0.1)
        self.stars["intensity"] .set_values([0])
        self.stars["speed"] .set_values([0])
        logging.info("Shutting down CinemaRoomController, setting all channels to 0.")
        await asyncio.sleep(0.1)

    def initialise_dmx_channels(self):
        self.lineardriver_controller_mappings = {
            "steps": {
                "controller_n": [0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3],
                "local_channel_n": [0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3, 0, 1],
                "dmx_channel": list(range(3, 17)),
            },
            "panels": {
                "controller_n": [4, 4, 4, 4, 5, 5, 5, 5],
                "local_channel_n": [0, 1, 2, 3, 0, 1, 2, 3],
                "dmx_channel": [19, 20, 21, 22, 23, 24, 25, 26],
            },
        }
        self.channels = {
            "steps": self.lineardriver_controller_mappings["steps"]["dmx_channel"],
            "stars_intensity": 1,
            "stars_position": 2,
            "panels": self.lineardriver_controller_mappings["panels"]["dmx_channel"],
        }

    def set_from_settings(self, settings: ControllerSettings):
        self.ip = settings.ip
        self.universe_id = settings.universe_id
        self.port = settings.port
        self.system = settings.system

        self.global_fade = settings.global_fade
        self.star_intensity_fade = settings.star_intensity_fade
        self.star_speed_fade = settings.star_speed_fade

        self.setup_delay = settings.setup_delay
        self.steps_wait_time = settings.steps_wait_time
        self.controller_delay = settings.controller_delay

        self.ambient_panel_intensity_max = settings.ambient_panel_intensity_max
        self.ambient_panel_intensity_min = settings.ambient_panel_intensity_min
        self.ambient_panel_delay_max = settings.ambient_panel_delay_max

        self.ambient_step_intensity_max = settings.ambient_step_intensity_max
        self.ambient_step_intensity_min = settings.ambient_step_intensity_min
        self.ambient_step_delay_max = settings.ambient_step_delay_max

        self.ambient_star_intensity_max = settings.ambient_star_intensity_max
        self.ambient_star_intensity_min = settings.ambient_star_intensity_min
        self.ambient_star_delay_max = settings.ambient_star_delay_max

        self.ambient_star_speed_max = settings.ambient_star_speed_max
        self.ambient_star_speed_min = settings.ambient_star_speed_min

        # initialise the dmx channels this sets the mapping for the DMX channels
        self.initialise_dmx_channels()
        # Generate random intensity sequences for ambient lighting
        self.get_random_intensity_sequences()
        # initialise the intensity layers (these allow seperate control of the light level via ambiant, sensor, and manual control)
        self.initialise_layers()

    def set_manual_distance(self, value):
        print("set manual override", self.manual_distance_override)
        self.manual_distance_override = float(value)

    def set_ambient_multiplier(self, ambient_multiplier):
        """
        Set the ambient multiplier value.

        Parameters:
            ambient_multiplier (float): The new value to set for the ambient multiplier.
        """
        self.ambient_multiplier = ambient_multiplier
        logging.info(f"Ambient multiplier set to: {self.ambient_multiplier}")
        
    def get_rand_sequences(
        self,
        intensity_min=30,
        intensity_max=120,
        delay_max=3,
        sequence_length=100,
        n_sequences=1,
    ):
        """
        Generate randomized sequences of light intensities and delays.
        This method creates a specified number of randomized sequences for light intensity and delay values,
        which can be used for simulating or controlling lighting effects.

        Args:
            intensity_min (int, optional): Minimum intensity value for the sequence. Defaults to 30.
            intensity_max (int, optional): Maximum intensity value for the sequence. Defaults to 120.
            delay_max (float, optional): Maximum delay value (in seconds) for the sequence. Defaults to 3.
            sequence_length (int, optional): Number of elements in each sequence. Defaults to 100.
            n_sequences (int, optional): Number of randomized sequences to generate. Defaults to 1.

        Returns:
            tuple: A tuple containing two lists:
                - intensity_sequences (list of list of int): Randomized intensity sequences.
                - delay_sequences (list of list of float): Randomized delay sequences.
        """
        intensity_sequences = []
        delay_sequences = []
        intensity_sequence = (
            (
                intensity_min
                + (intensity_max - intensity_min) * np.random.random(sequence_length)
            )
            .astype(int)
            .tolist()
        )
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
        """
        Asynchronously detects and updates the distance measurement for the home cinema control system.

        Continuously runs while the controller is active, updating the distance value based on either a manual override,
        a Time-of-Flight (ToF) sensor (if running on a Raspberry Pi), or defaults to zero otherwise. Updates the step
        controller with the new distance, calculates smoothed distance and step intensities, and logs relevant information.
        Handles exceptions from the ToF sensor gracefully.

        The method also updates the step layers with the latest step intensities and waits for a specified interval
        before repeating.

        Raises:
            Logs a warning if an exception occurs during distance measurement.
        """
        while self._running:
            try:
                if self.manual_distance_override is not None:
                    self.distance = self.manual_distance_override
                    self.step_controller.update_distance(
                        self.distance, self.steps_wait_time
                    )
                    smoothed_distance = self.step_controller.get_smoothed_distance()
                    self.step_intensities = self.step_controller.get_intensities(
                        smoothed_distance
                    )

                elif self.system == "pi":
                    self.distance = self.tof.get_distance()
                    self.step_controller.update_distance(
                        self.distance, self.steps_wait_time
                    )
                    smoothed_distance = self.step_controller.get_smoothed_distance()
                else:
                    self.distance = 0.0
                    smoothed_distance = self.distance
                    self.step_intensities = np.zeros(self.n_steps)

            except Exception as e:
                logging.warning(f"ToF sensor error: {e}")

            logging.info(
                f"ToF distance: {self.distance}, Smoothed distance: {smoothed_distance} steps_person_state: {self.step_controller.steps_person_state}"
            )
            logging.info(f"Step intensities: {self.step_intensities}")
            self.step_layers["sensor"] = self.step_intensities

            await asyncio.sleep(self.steps_wait_time)

    async def _setup_linear_drive_dmx_controllers(self, pulse_on_start=True):
        """
        Asynchronously sets up linear drive DMX controllers by initializing and configuring six DMX channels.

        For each controller:
            - Adds a DMX channel with a width of 4, starting from channel 3 and incrementing by 4 for each controller.
            - Sets the channel values to [255, 255, 255, 255] (representing white with dimmer).
            - Waits for a setup delay.
            - Resets the channel values to [0, 0, 0, 0].
            - Waits for a setup delay.
            - Appends the configured channel to the controllers list.

        Returns:
            list: A list of configured DMX channel objects.
        """
        linear_drive_dmx_controllers = []
        start_channel = 3
        for controller_n in range(0, 6):
            channel = self.universe.add_channel(
                start=start_channel + (controller_n * 4), width=4
            )
            if pulse_on_start:
                # Pulse the channel on start
                channel.set_values([255, 255, 255, 255])  # White with dimmer
                await asyncio.sleep(self.setup_delay)
                channel.set_values([0, 0, 0, 0])  # White with dimmer
                await asyncio.sleep(self.setup_delay)
            linear_drive_dmx_controllers.append(channel)
        return linear_drive_dmx_controllers

    async def setup(self):
        self.node = ArtNetNode(self.ip, port=self.port)
        logging.info(f"Connecting to {self.ip}")
        logging.info(f"Connected to {self.ip}")
        logging.info("Artnet node initialized.")
        self.universe = self.node.add_universe(self.universe_id)
        self.stars = await self._setup_stars()
        self.linear_drive_dmx_controllers = (
            await self._setup_linear_drive_dmx_controllers()
        )
        if self.system == "pi":
            self.step_controller.reset()

    async def _setup_stars(self, pulse_on_start=True):
        star_controllers = {}
        star_controllers["intensity"] = self.universe.add_channel(
            start=self.channels["stars_intensity"], width=1
        )
        star_controllers["speed"] = self.universe.add_channel(
            start=self.channels["stars_position"], width=1
        )
        if pulse_on_start:
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
            stars_intensity, stars_position = self.mix_intensity_stars()
            # stars_position = stars_intensity.copy()

            # we have 6 linear_drive_dmx_controllers, each 4 channels
            controller_intensities = np.zeros((6, 4)).astype(int)

            # steps
            for controller_n, local_channel_n, step_intensity in zip(
                self.lineardriver_controller_mappings["steps"]["controller_n"],
                self.lineardriver_controller_mappings["steps"]["local_channel_n"],
                step_intensities,
            ):
                controller_intensities[controller_n, local_channel_n] = step_intensity
            # panels
            for controller_n, local_channel_n, panel_intensity in zip(
                self.lineardriver_controller_mappings["panels"]["controller_n"],
                self.lineardriver_controller_mappings["panels"]["local_channel_n"],
                panel_intensities,
            ):
                controller_intensities[controller_n, local_channel_n] = panel_intensity
            # stars
            self.stars["intensity"].add_fade(
                [stars_intensity], self.star_intensity_fade
            )
            self.stars["speed"].add_fade([stars_position], self.star_speed_fade)

            logging.info(f"Step intensities: {step_intensities}")
            logging.info(f"Panel intensities: {panel_intensities}")
            logging.info(f"Star intensity: {stars_intensity}")
            logging.info(f"Star position: {stars_position}")
            for controller_n, controller in enumerate(
                self.linear_drive_dmx_controllers
            ):
                controller.add_fade(
                    controller_intensities[controller_n], self.global_fade
                )
                await asyncio.sleep(self.controller_delay)
                # await asyncio.sleep(
                #     self.controller_delay
                # )  # <-- Needed to pulse and allow fading

    def mix_intensity_panels(self):
        # Combine values from each layer (choose strategy):

        intensities = []
        for panel_n in range(self.n_panels):
            layers = {
                "ambient": self.panel_layers["ambient"][panel_n],
                "sensor": self.panel_layers["sensor"][panel_n],
                "manual": self.panel_layers["manual"][panel_n],
            }
            intensities.append(int(max(layers.values()) * self.ambient_multiplier))
        return intensities

    def mix_intensity_stars(self):
        # Combine values from each layer (choose strategy):
        intensities = []
        for n in range(len(self.star_layers["ambient"])):
            layers = {
                "ambient": self.star_layers["ambient"][n],
                "sensor": self.star_layers["sensor"][n],
                "manual": self.star_layers["manual"][n],
            }
            intensities.append(int(max(layers.values()) * self.ambient_multiplier))
        return intensities[0], intensities[1]  # Return intensity and position

    def mix_intensity_steps(self):
        # Combine values from each layer (choose strategy):
        intensities = []
        for panel_n in range(self.n_steps):
            layers = {
                "ambient": self.step_layers["ambient"][panel_n],
                "sensor": self.step_layers["sensor"][panel_n],
                "manual": self.step_layers["manual"][panel_n],
            }
            intensities.append(int(max(layers.values()) * self.ambient_multiplier))
        return intensities

        # strongest wins
        # return int(sum(self.layers.values()[panel_n]) / len(self.layers))  # average
        # return min(255, self.layers["ambient"][panel_n] + self.layers["sensor"][panel_n])  # additive (capped)

    async def pulse_panels_intensities(self):
        while self._running:
            tasks = [
                asyncio.create_task(self.pulse_panel_intensity(panel_n))
                for panel_n in range(self.n_panels)
            ]
            await asyncio.gather(*tasks)

    async def pulse_star_intensities(self):
        while self._running:
            tasks = [
                asyncio.create_task(self.pulse_star()),
                asyncio.create_task(self.move_star()),
            ]
            await asyncio.gather(*tasks)

    async def pulse_step_intensities(self):
        while self._running:
            tasks = []
            for step_n in range(self.n_steps):
                tasks.append(asyncio.create_task(self.pulse_step_intensity(step_n)))
            await asyncio.gather(*tasks)

    async def pulse_panel_intensity(self, panel_n=0):
        # pulse a signal panel
        # Make shuffled copies each time
        while self._running:
            intensities = self.panel_intensity_sequences[panel_n]
            delays = self.panel_delay_sequences[panel_n]
            random.shuffle(intensities)
            random.shuffle(delays)
            for intensity, delay in zip(intensities, delays):
                self.panel_layers["ambient"][panel_n] = intensity
                await asyncio.sleep(delay)

    async def pulse_step_intensity(self, panel_n=0):
        # pulse a signal panel

        while self._running:
            intensities = self.steps_intensity_sequences[panel_n][:]
            delays = self.steps_delay_sequences[panel_n][:]
            random.shuffle(intensities)
            random.shuffle(delays)

            for intensity, delay in zip(intensities, delays):
                self.step_layers["ambient"][panel_n] = intensity
                await asyncio.sleep(delay)

    async def pulse_star(self):
        # pulse a signal panel
        while self._running:
            for intensity, delay in zip(
                self.stars_intensity_sequences[0], self.stars_delay_sequences[0]
            ):
                self.star_layers["ambient"][0] = intensity
                await asyncio.sleep(delay)

    async def move_star(self):
        # pulse a signal panel
        while self._running:
            for intensity, delay in zip(
                self.stars_position_sequences[0], self.stars_position_delay_sequences[0]
            ):
                self.star_layers["ambient"][1] = intensity
                await asyncio.sleep(delay)

    async def run(self):
        tasks = [
            self.pulse_panels_intensities(),
            self.pulse_step_intensities(),
            self.pulse_star_intensities(),
            self.detect_distance(),
            self.update_dmx(),
        ]
        await asyncio.gather(*tasks)


def start_asyncio_loop(controller):
    asyncio.run(controller.run())

from fastapi import Body

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
ip = os.getenv("IP")

system = "mac" if system is None else system
ip = "192.168.1.191" if ip is None else ip
ip = "127.0.0.1"
# Global controller instance
try:
    settings = load_settings_from_file("settings.json")
    controller = CinemaRoomController(**settings.dict())
    logging.info("Loaded controller settings from JSON.")
except Exception as e:
    logging.warning(f"Failed to load settings.json, using defaults. Error: {e}")
    controller = CinemaRoomController(system=system, ip=ip)


@app.post("/reload_settings")
def reload_settings():
    try:
        settings = load_settings_from_file("settings.json")
        controller.set_from_settings(settings)
        return {"status": "ok", "settings": settings.dict()}
    except Exception as e:
        return {"status": "error", "message": str(e)}


@app.get("/settings")
def get_settings():
    return ControllerSettings(**controller.__dict__)

@app.post("/settings")
async def update_settings(settings: ControllerSettings):
    global controller, controller_thread

    if controller:
        controller._running = False
        await controller.shutdown()  # <--- add this
        # if controller_thread and controller_thread.is_alive():
        #     controller_thread.join(timeout=5)
    # await asyncio.sleep(0.5)  # Allow DMX to settle to zero

    # Create a new controller with the updated settings
    # controller = CinemaRoomController()
    controller.set_from_settings(settings)
    controller._running = True 
    # await controller.setup()
    # await asyncio.sleep(2.0)

    # Restart controller loop in new thread
    controller_thread = threading.Thread(
        target=start_asyncio_loop, args=(controller,), daemon=True
    )
    controller_thread.start()

    return {"status": "ok", "message": "Settings updated and controller restarted."}

@app.post("/save_settings")
def save_settings(filename: str = Query("settings.json")):
    try:
        settings = ControllerSettings(
            ip=controller.ip,
            universe_id=controller.universe_id,
            port=controller.port,
            system=controller.system,
            global_fade=controller.global_fade,
            star_intensity_fade=controller.star_intensity_fade,
            star_speed_fade=controller.star_speed_fade,
            setup_delay=controller.setup_delay,
            steps_wait_time=controller.steps_wait_time,
            controller_delay=controller.controller_delay,
            ambient_panel_intensity_max=controller.ambient_panel_intensity_max,
            ambient_panel_intensity_min=controller.ambient_panel_intensity_min,
            ambient_panel_delay_max=controller.ambient_panel_delay_max,
            ambient_step_intensity_max=controller.ambient_step_intensity_max,
            ambient_step_intensity_min=controller.ambient_step_intensity_min,
            ambient_step_delay_max=controller.ambient_step_delay_max,
            ambient_star_intensity_max=controller.ambient_star_intensity_max,
            ambient_star_intensity_min=controller.ambient_star_intensity_min,
            ambient_star_delay_max=controller.ambient_star_delay_max,
            ambient_star_speed_max=controller.ambient_star_speed_max,
            ambient_star_speed_min=controller.ambient_star_speed_min,
            sequence_length=controller.sequence_length,
        )
        save_settings_to_file(settings, path=filename)
        return {"status": "ok", "path": filename}
    except Exception as e:
        return {"status": "error", "message": str(e)}

@app.post("/save_settings_body")
def save_settings_body(request: SaveSettingsRequest):
    try:
        save_settings(filename=request.filename)
        return {"status": "ok", "path": request.filename}
    except Exception as e:
        return {"status": "error", "message": str(e)}

@app.get("/ambient_multiplier/{value}")
def set_ambient_multiplier(value: float):
    controller.set_ambient_multiplier(value)
    logging.info(f"Changing ambient level: {value}")
    return {"status": "ok", "ambient_multiplier": value}

@app.on_event("startup")
async def start_controller():
    global controller, controller_thread
    settings = load_settings_from_file("default.json")
    controller = CinemaRoomController()
    controller.set_from_settings(settings)

    await controller.setup()
    controller_thread = threading.Thread(
        target=start_asyncio_loop, args=(controller,), daemon=True
    )
    controller_thread.start()
