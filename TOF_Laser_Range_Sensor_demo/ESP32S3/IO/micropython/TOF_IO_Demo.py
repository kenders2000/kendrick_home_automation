from machine import Pin
from time import sleep

IO_H = Pin(16, Pin.IN)
IO_L = Pin(15, Pin.IN)
try:
    while True:
        if IO_H.value() == True and IO_L.value() == False:
            print("In range.")
        else:
            print("No in range.")
        sleep(0.2)
except KeyboardInterrupt:
    print("Quit.")