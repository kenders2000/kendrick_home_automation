import asyncio
from pyartnet import ArtNetNode
import random
import logging
from pyartnet import ArtNetNode
import numpy as np
# from PySide6.QtWidgets import QApplication, QWidget, QVBoxLayout, QSlider, QLabel, QPushButton
# from PySide6.QtCore import Qt
import sys
import asyncio
import threading
from fastapi import FastAPI
import uvicorn
import asyncio
# import matplotlib.pyplot as plt

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import threading
#coding: UTF-8
import sys 
import time
import os
sys.path.append("..")  # Append the parent directory to the system path for module imports
# import TOF_Sense  # Import the TOF_Sense module from the 'lib' directory


import serial 
import time

TOF_FRAME_HEADER = 0x57  #Define frame header 定义帧头
TOF_FUNCTION_MARK = 0x00 #Define function code 定义功能码

#Store decoded data 存放解码后的数据
TOF_system_time = 0     #The time after the TOF module is powered on, unit: ms TOF模块上电后经过的时间，单位：ms
TOF_distance = 0        #The distance output by the TOF module, unit: mm TOF模块输出的距离，单位:mm
TOF_status = 0          #The distance status indication output by the TOF module: 0 is invalid, 1 is valid TOF模块输出的距离状态指示:0为无效,1为有效
TOF_signal_strength = 0 #The signal strength output by the TOF module TOF模块输出的信号强度
TOF_range_precision = 0 #The repeatability accuracy reference value output by the TOF module is invalid for C, D and Mini types. Unit: cm TOF模块输出的重复测距精度参考值，对于C型,D型和Mini型是无效的，单位:cm

TOF_rx_data=[0] * 16 #Create a list with 16 members 创建一个拥有16个成员的列表
TOF_tx_data=[0x57,0x10,0xff,0xff,0x00,0xff,0xff,0x63] #Query the command with ID 0 查询ID为0的命令


class TOF_Sense():
    #Open the Raspberry Pi serial port device 打开树莓派串口设备
    def __init__(self, dev = '/dev/ttyS0',baud = 921600):
        self.TOF_peek = 0 #Temporary storage of data 临时存放数据

        self.count_i = 0 #Loop count variable 循环计数变量
        self.check_sum = 0 #Checksum 校验和
        self.ser = serial.Serial(dev,baud)
        self.ser.flushInput()#Clear the serial port input register 清空串口输入寄存器
    
    #Test active output mode 测试主动输出模式
    def TOF_Active_Decoding(self):    
        if self.ser.inWaiting() > 0: #Waiting for serial port data 等待串口数据
            self.TOF_peek = ord(self.ser.read(1))  #Read a byte and convert it into an integer 读取一个字节并转换成整数
            if self.TOF_peek == TOF_FRAME_HEADER: #If it is a frame header, restart the loop count 如果是帧头,则重新开始循环计数
                self.count_i = 0
                TOF_rx_data[self.count_i] = self.TOF_peek #Store the read data into a tuple for later decoding 将读取到的数据存入元组中，用于后面解码使用
            else:
                TOF_rx_data[self.count_i] = self.TOF_peek #Store the read data into a tuple for later decoding 将读取到的数据存入元组中，用于后面解码使用

            self.count_i = self.count_i + 1 #Loop count +1 循环计数+1

            if self.count_i > 15:#If the number of received data is greater than 15, the count variable can be cleared and a decoding can be performed. 接收数量大于15,则可以将计数变量清零并进行一次解码
                self.count_i = 0
                for i in range (0,15):
                    self.check_sum = (self.check_sum + TOF_rx_data[i]) & 0xFF #Calculate the checksum and take the lowest byte 计算检验和并取最低一个字节

                #Determine whether the decoding is correct 判断解码是否正确
                if (TOF_rx_data[0] == TOF_FRAME_HEADER) and (TOF_rx_data[1] == TOF_FUNCTION_MARK) and (self.check_sum == TOF_rx_data[15]):
                    print("TOF id is: "+ str(TOF_rx_data[3]))  #ID of the TOF module TOF 模块的 ID

                    TOF_system_time = TOF_rx_data[4] | TOF_rx_data[5]<<8 | TOF_rx_data[6]<<16 | TOF_rx_data[7]<<24
                    print("TOF system time is: "+str(TOF_system_time)+'ms') #The time after the TOF module is powered on TOF模块上电后经过的时间        

                    TOF_distance = (TOF_rx_data[8]) | (TOF_rx_data[9]<<8) | (TOF_rx_data[10]<<16)
                    print("TOF distance is: "+str(TOF_distance)+'mm') #The distance output by the TOF module TOF模块输出的距离   
            
                    TOF_status = TOF_rx_data[11]
                    print("TOF status is: "+str(TOF_status)) #Distance status indication output by TOF module TOF模块输出的距离状态指示
            
                    TOF_signal_strength = TOF_rx_data[12] | TOF_rx_data[13]<<8
                    print("TOF signal strength is: "+str(TOF_signal_strength)) #The signal strength output by the TOF module TOF模块输出的信号强度

                    TOF_range_precision = TOF_rx_data[14]
                    print("TOF range precision is: "+str(TOF_range_precision)) #The repeatability accuracy reference value output by the TOF module is invalid for Type C, Type D and Mini. TOF模块输出的重复测距精度参考值，对于C型,D型和Mini型是无效的
                    
                    print("")
                    self.ser.flushInput() #Clear the serial port input register 清空串口输入寄存器
                else:
                    print("Verification failed.")
            self.check_sum = 0 #Clear Checksum 清空校验和
        else:
            print("The serial port does not receive data.") 

    #Test query output mode 测试查询输出模式
    def TOF_Inquire_Decoding(self, id):
        TOF_tx_data[4] = id #Add the ID you want to query to the command 将需要查询的ID添加到命令中
        TOF_tx_data[7] = id + 0x63 #Update Checksum 更新校验和

        self.ser.flushInput() #Clear the serial port buffer 清空串口缓存
        self.ser.write(bytearray(TOF_tx_data)) #Start query 开始查询
        time.sleep(0.01) #Waiting for the sensor to return data 等待传感器返回数据
        TOF_rx_data = list(self.ser.read(16)) #Reading sensor data 读取传感器数据

        for i in range (0,15):
            self.check_sum = (self.check_sum + TOF_rx_data[i]) & 0xFF #Calculate the checksum and take the lowest byte 计算检验和并取最低一个字节

        #Determine whether the decoding is correct 判断解码是否正确
        if (TOF_rx_data[0] == TOF_FRAME_HEADER) and (TOF_rx_data[1] == TOF_FUNCTION_MARK) and (self.check_sum == TOF_rx_data[15]):
            # print("TOF id is: "+ str(TOF_rx_data[3]))  #ID of the TOF module TOF 模块的 ID

            TOF_system_time = TOF_rx_data[4] | TOF_rx_data[5]<<8 | TOF_rx_data[6]<<16 | TOF_rx_data[7]<<24
            # print("TOF system time is: "+str(TOF_system_time)+'ms') #The time after the TOF module is powered on TOF模块上电后经过的时间        

            TOF_distance = (TOF_rx_data[8]) | (TOF_rx_data[9]<<8) | (TOF_rx_data[10]<<16)
            # print("TOF distance is: "+str(TOF_distance)+'mm') #The distance output by the TOF module TOF模块输出的距离   
    
            TOF_status = TOF_rx_data[11]
            # print("TOF status is: "+str(TOF_status)) #Distance status indication output by TOF module TOF模块输出的距离状态指示
    
            TOF_signal_strength = TOF_rx_data[12] | TOF_rx_data[13]<<8
            # print("TOF signal strength is: "+str(TOF_signal_strength)) #The signal strength output by the TOF module TOF模块输出的信号强度

            TOF_range_precision = TOF_rx_data[14]
            # print("TOF range precision is: "+str(TOF_range_precision)) #The repeatability accuracy reference value output by the TOF module is invalid for Type C, Type D and Mini. TOF模块输出的重复测距精度参考值，对于C型,D型和Mini型是无效的
            
            # print("")
            self.ser.flushInput() #Clear the serial port input register 清空串口输入寄存器
        else:
            print("Verification failed.")
        self.check_sum = 0 #Clear Checksum 清空校验和 
        return TOF_distance


def detect_pi_model():
    """
    Detects the Raspberry Pi model by reading the device tree model file.
    
    Returns:
        str: The model string of the Raspberry Pi.
    """
    with open('/proc/device-tree/model') as f:  # Open the device tree model file
        model = f.read().strip()  # Read and strip the model string
    return model

class tof:
    def __init__(self, delay=0.01):
        self.delay = delay
        self.tof_sense = self.configure_tof()
        self.distance = 0.0

    def configure_tof(self):
        if "Raspberry Pi 5" in detect_pi_model():
            return TOF_Sense('/dev/ttyAMA0', 921600)
        else:
            print("not a raspberry pi 5")
            return TOF_Sense('/dev/ttyS0', 921600)

    def get_distance(self):
        try:
            # Use synchronous decoder and capture the printed distance
            distance = self.tof_sense.TOF_Inquire_Decoding(0)  # ← MODIFY THIS FUNCTION TO RETURN DISTANCE
            self.distance = distance
            return self.distance
        except Exception as e:
            print("[ERROR] Failed to get ToF distance:", e)
            return self.distance        
        
# Enable logging
# logging.basicConfig(level=logging.INFO)  # or INFO for less detail


def asymmetric_gaussian(x, mu=0, sigma_left=0.5, sigma_right=2.0):
    return np.where(
        x < mu,
        np.exp(-0.5 * ((x - mu) / sigma_left) ** 2),
        np.exp(-0.5 * ((x - mu) / sigma_right) ** 2)
    )
# x = np.linspace(0,14)
# y= asymmetric_gaussian(x, mu=6, sigma_left=0.1, sigma_right=6.0)
# plt.plot(x,y)
# plt.show()
class ExponentialAverager:
    def __init__(self, alpha=0.1):
        """
        alpha: smoothing factor, 0 < alpha <= 1
               higher alpha = more weight to new data
        """
        self.alpha = alpha
        self.avg = None  # Will hold the current average

    def reset(self):
        self.avg = None  # Will hold the current average

    def update(self, new_frame: np.ndarray):
        """
        Update the moving average with a new frame.
        Returns the updated average.
        """
        if self.avg is None:
            self.avg = new_frame.astype(np.float32)  # Initialize with first frame
        else:
            self.avg = self.alpha * new_frame + (1 - self.alpha) * self.avg
        return self.avg


class StepTOFController:
    def  __init__(self, n_steps=14, step_start_position=.500, step_stop_position=4.3):
        self.n_steps=n_steps
        self.step_start_position = step_start_position
        self.step_stop_position = step_stop_position
        self.sigma_infront = 0.1
        self.sigma_behind = 4.0
        self.smoothing = ExponentialAverager(alpha=0.1)
        self.reset()
    
    def asymmetric_gaussian(self, step_position, direction="down"):
        x = np.linspace(0, 14)
        if direction == "down":
            y = asymmetric_gaussian(x, mu=step_position, sigma_left=self.sigma_infront, sigma_right=self.sigma_behind)
        elif direction == "up":
            y = asymmetric_gaussian(x, mu=step_position, sigma_left=self.sigma_behind, sigma_right=self.sigma_infront)
        return y
    
    def reset(self):
        self.intensities = None
        self.n_frames = 0
        self.smoothing.reset()

    def get_step_n_from_position(self, position_m):
        print(position_m,  self.n_steps, self.step_start_position, self.step_stop_position)
        step_n = self.n_steps * (position_m - self.step_start_position) / (self.step_stop_position - self.step_start_position)
        return step_n
   
    def get_intensities(self, position_m, direction="down"):
        # if position_m is None:
        #     return np.zeros(self.n_steps)
        step_position = self.get_step_n_from_position(position_m)
        intensities_new = self.asymmetric_gaussian(step_position, direction=direction)
        self.intensities = self.smoothing.update(intensities_new)
        return self.intensities

class CinemaRoomController:
    def __init__(self, ip='192.168.1.191', universe_id=0, port=6454, system="mac"):
        self.distance = 0.0
        self.ip = ip
        self.port = port
        self.global_fade = 500
        self.universe_id = universe_id
        self.step_channels = list(range(3, 17))
        self.initial_levels = 30
        self.setup_delay = 0.2
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
                "dmx_channel": [17, 18, 19, 20, 21, 22, 23, 24],
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
 

        self.ambient_step_intensity_max = 100
        self.ambient_step_intensity_min = 10
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

            self.distance = self.tof.get_distance()  # ✅ now it's sync
            print("ToF distance:", self.distance)
            self.step_intensities = self.step_controller.get_intensities(self.distance)
            print("Step intensities:", self.step_intensities)

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

        if self.system == "pi":
            tasks.append(self.detect_distance())

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

