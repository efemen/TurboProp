import serial
import time
import subprocess
import numpy as np

# Description: This file contains the tools for the active grid class

class active_grid():
    def __init__(self):
        # Initialize the serial port
        self.com_port = "COM4"
        self.baud_rate = 9600
        self.matlab_script = "C:/Users/Lab7/Desktop/Users/dwc/Active_Grid_Scripts/Active_Grid_Scripts/run_test.m"  # Replace with the correct path to your .m file
        self.matlab_script_shear = "C:/Users/Lab7/Desktop/Users/dwc/Active_Grid_Scripts/Active_Grid_Scripts/oscillate_motors_position.m"

    def go_home(self):
        # Open the serial port
        self.ser = serial.Serial(self.com_port, self.baud_rate)
        # Send the home command
        
        # Send commands
        self.ser.write(b"AC25\r")  # AC25 command with carriage return
        self.ser.write(b"DE25\r")  # DE25 command with carriage return
        self.ser.write(b"VE4\r")   # VE4 command with carriage return
        self.ser.write(b"DI0\r")   # DI0 command with carriage return

        # Pause for 0.5 seconds
        time.sleep(0.5)

        # Send the FP0 command
        self.ser.write(b"FP0\r")   # FP0 command with carriage return

        self.ser.close()
        print("All motors adjusted and zeroed.")
        time.sleep(1)

    def stop(self):
        # Open the serial port
        self.ser = serial.Serial(self.com_port, self.baud_rate)
        # Send the stop command
        self.ser.write(b"SJ\r")
        self.ser.write(b"SJ\r")
        self.ser.write(b"SJ\r")
        self.ser.close()
        time.sleep(1)

    def run_case(self, case_number):
        if case_number not in range(1, 5):
            print("Invalid case number.")
            return
        
        matlab_executable = 'matlab'  # Or use 'matlab -batch' for newer versions

        with open(self.matlab_script, "r") as file:
            lines = file.readlines()

        with open(self.matlab_script, "w") as file:
            for line in lines:
                if "load" in line:
                    file.write(f"load('Efe_Case{case_number}.mat');\n")
                else:
                    file.write(line)

        time.sleep(1)
        
        # Create the command to run the MATLAB script
        command = [matlab_executable, '-batch', f"run('{self.matlab_script}')"]

        # Run the MATLAB script via subprocess
        print("Running MATLAB script...")
        self.process = subprocess.Popen(command)

    def run_shear_case(self, case_number):
        if case_number not in range(1, 5):
            print("Invalid case number.")
            return
        
        matlab_executable = 'matlab'  # Or use 'matlab -batch' for newer versions

        with open(self.matlab_script_shear, "r") as file:
            lines = file.readlines()

        with open(self.matlab_script_shear, "w") as file:
            for line in lines:
                if "load" in line:
                    file.write(f"load('Efe_ShearCase{case_number+2}.mat');\n")
                else:
                    file.write(line)

        time.sleep(1)
        
        # Create the command to run the MATLAB script
        command = [matlab_executable, '-batch', f"run('{self.matlab_script_shear}')"]

        # Run the MATLAB script via subprocess
        print("Running MATLAB script...")
        self.process = subprocess.Popen(command)

    def stop_case(self):
        subprocess.run(['taskkill', '/F', '/IM', 'matlab.exe'], check=True)
        print("MATLAB script terminated.")
        time.sleep(5)  # Wait for 5 seconds
        self.stop()
