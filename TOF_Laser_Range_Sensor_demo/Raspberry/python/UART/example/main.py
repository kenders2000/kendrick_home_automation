#coding: UTF-8
import sys 
import time
sys.path.append("..")  # Append the parent directory to the system path for module imports
from lib import TOF_Sense  # Import the TOF_Sense module from the 'lib' directory

# Function to detect the Raspberry Pi model
def detect_model():
    """
    Detects the Raspberry Pi model by reading the device tree model file.
    
    Returns:
        str: The model string of the Raspberry Pi.
    """
    with open('/proc/device-tree/model') as f:  # Open the device tree model file
        model = f.read().strip()  # Read and strip the model string
    return model

# Detect the Raspberry Pi model and initialize the TOF_Sense object with the appropriate serial port
if "Raspberry Pi 5" in detect_model():  # Check if the model is Raspberry Pi 5
    tof = TOF_Sense.TOF_Sense('/dev/ttyAMA0', 921600)  # Initialize TOF_Sense with ttyAMA0 for Raspberry Pi 5
else:
    tof = TOF_Sense.TOF_Sense('/dev/ttyS0', 921600)  # Initialize TOF_Sense with ttyS0 for other models

# Main loop to continuously perform TOF (Time-of-Flight) decoding
try:
    while True:  # Infinite loop to keep the program running
        tof.TOF_Active_Decoding()  # Perform active TOF decoding (Active Output Example)
        # tof.TOF_Inquire_Decoding(0)  # Uncomment this line to perform query-based TOF decoding (Example query output)
        time.sleep(0.01)  # Sleep for 0.02 seconds (default refresh rate is 50Hz; for 100Hz, use 0.01 seconds)

except KeyboardInterrupt:  # Handle the KeyboardInterrupt exception to allow graceful exit
    print("Quit.")  # Print a message indicating the program is quitting