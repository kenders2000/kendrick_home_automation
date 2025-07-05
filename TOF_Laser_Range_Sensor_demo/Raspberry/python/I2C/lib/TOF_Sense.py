# /*****************************************************************************
# * | File        :	  tof_sense.py
# * | Author      :   Waveshare team
# * | Function    :   TOF drive function
# * | Info        :
# *----------------
# * | This version:   V1.0
# * | Date        :   2024-09-11
# * | Info        :   
# ******************************************************************************
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documnetation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to  whom the Software is
# furished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS OR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#

import smbus 
import time

TOF_addr = 0x08 #7-bit slave address=ID+0x08 ID=0

TOF_REGISTER_TOTAL_SIZE = 48 #Total length of all registers

TOF_ADDR_MODE = 0x0c #pattern variable address
TOF_SIZE_MODE = 1 #The number of bytes occupied by the pattern variable

TOF_ADDR_ID = 0x0d #ID variable address
TOF_SIZE_ID = 1 #The number of bytes occupied by the ID variable

TOF_ADDR_UART_BAUDRATE = 0x10 #UART baud rate variable address
TOF_SIZE_UART_BAUDRATE = 4 #The number of bytes occupied by the UART baud rate variable

TOF_ADDR_SYSTEM_TIME = 0x20 #System time variable address
TOF_SIZE_SYSTEM_TIME = 4 #The number of bytes occupied by the system time variable

TOF_ADDR_DIS = 0x24 #distance variable address
TOF_SIZE_DIS = 4 #The number of bytes occupied by the distance variable

TOF_ADDR_DIS_STATUS = 0x28 #Distance status indication variable address
TOF_SIZE_DIS_STATUS = 2 #The number of bytes occupied by the distance status indicator variable

TOF_ADDR_SIGNAL_STRENGTH = 0x2a #Signal strength variable address
TOF_SIZE_SIGNAL_STRENGTH = 2 #The number of bytes occupied by the signal strength variable

TOF_ADDR_RANGE_PRECISION = 0x2c #Ranging accuracy variable address
TOF_SIZE_RANGE_PRECISION = 1 #The number of bytes occupied by the ranging precision variable

IIC_CHANGE_TO_UART_DATA = 0x00 #Change the communication mode to the byte data that needs to be sent by UART

#Store decoded data 存放解码后的数据
TOF_system_time = 0     #The time after the TOF module is powered on, unit: ms TOF模块上电后经过的时间，单位：ms
TOF_distance = 0        #The distance output by the TOF module, unit: mm TOF模块输出的距离，单位:mm
TOF_status = 0          #The distance status indication output by the TOF module: 0 is invalid, 1 is valid TOF模块输出的距离状态指示:0为无效,1为有效
TOF_signal_strength = 0 #The signal strength output by the TOF module TOF模块输出的信号强度
TOF_range_precision = 0 #The repeatability accuracy reference value output by the TOF module is invalid for C, D and Mini types. Unit: cm TOF模块输出的重复测距精度参考值，对于C型,D型和Mini型是无效的，单位:cm
TOF_interface_mode = 0  #communication interface mode,0-UART，1-CAN，2-I/O，3-IIC 通讯接口模式,0-UART，1-CAN，2-I/O，3-IIC
TOF_uart_baudrate = 0   #UART baud rate 串口波特率



class TOF_Sense():
    #Open the Raspberry Pi serial port device 打开树莓派串口设备
    def __init__(self, bus = 1):
        self.bus = smbus.SMBus(bus)
    
    def read(self,reg):
        data = self.bus.read_i2c_block_data(TOF_addr, reg, 1)
        return data[0]

    def read_word(self,reg):
        data = self.bus.read_i2c_block_data(TOF_addr, reg, 2)
        return ((data[1] * 256 ) + data[0])

    def write(self,reg,data):
        temp = [0]
        temp[0] = data & 0xFF
        self.bus.write_i2c_block_data(TOF_addr,reg,temp)

    def write_word(self,reg,data):
        temp = [0,0]
        temp[0] = data & 0xFF
        temp[1] =(data & 0xFF00) >> 8
        self.bus.write_i2c_block_data(TOF_addr,reg,temp)

    def I2C_Read_Nbyte(self,reg,num):
        data = self.bus.read_i2c_block_data(TOF_addr, reg, num)
        return data

    #Test active output mode 测试I2C通信模式
    def TOF_Inquire_I2C_Decoding(self):
        pdata = self.I2C_Read_Nbyte(0x00,TOF_REGISTER_TOTAL_SIZE // 2) + self.I2C_Read_Nbyte(0x18,TOF_REGISTER_TOTAL_SIZE // 2) #Read all sensor data 读取传感器全部数据
        
        TOF_interface_mode=pdata[TOF_ADDR_MODE]&0x07 # Working mode of TOF module TOF 模块的工作模式

        TOF_id=pdata[TOF_ADDR_ID] # ID of the TOF module TOF 模块的 ID

        TOF_uart_baudrate=((pdata[TOF_ADDR_UART_BAUDRATE])|(pdata[TOF_ADDR_UART_BAUDRATE+1]<<8)|#  TOF module serial port baud rate TOF 模块的串口波特率
				                                       (pdata[TOF_ADDR_UART_BAUDRATE+2]<<16)|(pdata[TOF_ADDR_UART_BAUDRATE+3]<<24))
        
        TOF_system_time=((pdata[TOF_ADDR_SYSTEM_TIME])|(pdata[TOF_ADDR_SYSTEM_TIME+1]<<8)|# The time after the TOF module is powered on TOF模块上电后经过的时间
				                                     (pdata[TOF_ADDR_SYSTEM_TIME+2]<<16)|(pdata[TOF_ADDR_SYSTEM_TIME+3]<<24))
        
        TOF_distance=(float)((pdata[TOF_ADDR_DIS])|(pdata[TOF_ADDR_DIS+1]<<8)|# The distance output by the TOF module TOF模块输出的距离
				                          (pdata[TOF_ADDR_DIS+2]<<16)|(pdata[TOF_ADDR_DIS+3]<<24))

        TOF_status=((pdata[TOF_ADDR_DIS_STATUS])|(pdata[TOF_ADDR_DIS_STATUS+1]<<8))# Distance status indication output by TOF module TOF模块输出的距离状态指示
        
        TOF_signal_strength=((pdata[TOF_ADDR_SIGNAL_STRENGTH])|(pdata[TOF_ADDR_SIGNAL_STRENGTH+1]<<8))# The signal strength output by the TOF module TOF模块输出的信号强度

        TOF_range_precision=pdata[TOF_ADDR_RANGE_PRECISION] #The repeatability accuracy reference value output by the TOF module is invalid for Type C, Type D and Mini. TOF模块输出的重复测距精度参考值，对于C型,D型和Mini型是无效的
        
        # Print data through the terminal 通过终端打印数据
        print("id: %d"%TOF_id)
        print("system_time:%d "%TOF_system_time)
        print("dis: %d mm"%TOF_distance)
        print("dis_status: %d "%TOF_status)
        print("signal_strength: %d "%TOF_signal_strength)
        print("range_precision: %d "%TOF_range_precision)

        if TOF_interface_mode == 0:# communication interface mode,0-UART，1-CAN，2-I/O，3-IIC 通讯接口模式,0-UART，1-CAN，2-I/O，3-IIC
            print("UART Mode.")
        elif TOF_interface_mode == 1:     
            print("CAN Mode.")
        elif TOF_interface_mode == 2:
            print("I/O Mode.")
        else:
            print("I2C Mode.")

        print("TOF uart baudrate is:%d\n"%TOF_uart_baudrate)
    
    #Switch I2C mode to UART query mode I2C模式切换成UART查询模式
    def IIC_Change_Mode_To_UART(self):
        self.write(TOF_ADDR_MODE,IIC_CHANGE_TO_UART_DATA)
        print("Modification successful.")
    