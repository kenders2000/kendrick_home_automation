#coding: UTF-8
import sys 
import time
sys.path.append("..")
from lib import TOF_Sense
tof = TOF_Sense.TOF_Sense(1)
  
try:
    while True:
        tof.TOF_Inquire_I2C_Decoding()  # Get TOF data through I2C and decode it 通过I2C获取TOF数据，并进行解码
        # tof.IIC_Change_Mode_To_UART() # Switch I2C mode to UART query mode I2C模式切换成UART查询模式
        time.sleep(0.02)               

except KeyboardInterrupt:
    print("Quit.")    
        
    





