https://www.waveshare.com/wiki/TOF_Laser_Range_Sensor_(C)#I2C_example


```
sudo apt install gpiod libgpiod-dev
wget "https://files.waveshare.com/wiki/TOF-Laser-Range-Sensor-(C)/TOF_Laser_Range_Sensor_demo.zip"
unzip TOF_Laser_Range_Sensor_demo.zip
cd TOF_Laser_Range_Sensor_demo/

sudo raspi-config nonint do_serial 2
```

raspberry pi 5 1 - home assistent server

Raspberry pi 4 - dmx server - light fixture for all dmx

- 


```
sudo raspi-config
# Go to Interface Options > Serial Port:
# Login shell over serial: No
# Enable serial port hardware: Yes


python -m venv env
source env/bin/activate
pip install numpy --prefer-binary
pip install pyserial pyartnet fastapi uvicorn
sudo apt install libopenblas0
```