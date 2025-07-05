from gpiozero import Button
from time import sleep

IO_H = Button(6, pull_up=False) #Define the high signal input pin, no pull-up, make sure to read the information consistent with the level, otherwise it will be reversed 定义高信号输入脚，无上拉，确定读取跟电平一致的信息，否则会反向
IO_L = Button(5, pull_up=False) #Define the low signal input pin, no pull-up, make sure to read the information consistent with the level, otherwise it will be reversed 定义低信号输入脚，无上拉，确定读取跟电平一致的信息，否则会反向
try:
    while True:
        if (IO_H.is_pressed == True) and (IO_L.is_pressed == False):#Read the signals of the two pins to determine whether they are within the set distance range 读取两个引脚的信号，判断是否在设定的距离范围内
            print("In range.")
        else:
            print("Not in range.")
        sleep(0.02) #Set the reading time according to the refresh rate, 50HZ is set to 0.02S 按照刷新率去设置读取时间，50HZ设置为0.02S
except KeyboardInterrupt:
    print("Quit.")    