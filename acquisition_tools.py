# This module contains the acquisition tools for an NI USB-6212 DAQ device for force/torque sensing, encoder reading and ESC control.
# Efe Sen, 12/02/2025


import numpy as np
import nidaqmx
from nidaqmx.constants import AcquisitionType, Level
from nidaqmx.stream_readers import AnalogMultiChannelReader
from nidaqmx.types import CtrFreq
from nidaqmx import constants
from nidaqmx.stream_readers import AnalogMultiChannelReader
import time
from matplotlib.widgets import Button
import matplotlib.pyplot as plt
import sys
from pymodbus.client import ModbusTcpClient


class wind_tunnel():
    def __init__(self, ip = '192.168.0.10', port = 502):
        self.ip = ip
        self.port = port
        self.client = ModbusTcpClient(ip, port)
        self.fit = [2.0882, -4.1089]
    
    def set_U0(self, u):
        if u == 0:
            v = 0
        else:
            v = round(self.fit[0] * u + self.fit[1], 1)
        
        write_address = 1
        write_data = int(v * 10)
        
        if v > 96:
            raise ValueError('Do not set the fan speed to > 96%.')
        else:
            self.client.write_register(write_address, write_data)
    
    def stop(self):
        write_address = 0
        write_data = 0
        self.client.write_register(write_address, write_data)
    
    def close(self):
        self.client.close()

class analog_reader():
    """
    A class to represent a force sensor using NI-DAQmx.
    Attributes
    ----------
    task : nidaqmx.Task
        The NI-DAQmx task for the force sensor.
    stream : nidaqmx.stream_readers.AnalogMultiChannelReader
        The stream reader for the analog input channels.
    Methods
    -------
    __init__(fs_rate=4000, buffer_size=100)
    Initializes the force sensor with the specified sampling rate and buffer size.

    Constructs all the necessary attributes for the force sensor object.
    Parameters
    ----------
    fs_rate : int, optional
        The sampling rate in samples per second (default is 4000).
    buffer_size : int, optional
        The buffer size in samples per channel (default is 1000).
    """


    def __init__(self, fs_rate=4000, buffer_size=100):
        self.task = nidaqmx.Task()
        self.task.ai_channels.add_ai_voltage_chan("Dev1/ai1:7")  # ATI Mini40 Force/Torque Sensor has 6 channels and the encoder
        self.task.timing.cfg_samp_clk_timing(rate=fs_rate, sample_mode=constants.AcquisitionType.CONTINUOUS,
                                        samps_per_chan=buffer_size)
        self.stream = AnalogMultiChannelReader(self.task.in_stream)


        self.cal_matrix = np.array([
            [0.00371,  -0.02167,   0.00473,   3.15117,   0.02060,  -3.17213], # ATI Mini 40 IP66 calibration matrix SOTON BLWT, 12/02/2025
            [0.00433,  -3.79734,  -0.01068,   1.75137,  -0.00602,   1.87770], 
            [5.14105,  -0.08149,   5.23352,  -0.13159,   5.39405,  -0.12217],
            [-0.00134,  -0.04024,   0.07490,   0.01583,  -0.07733,   0.02243], 
            [-0.08373,   0.00302,   0.04110,  -0.03512,   0.04666,   0.03224],
            [0.00106,  -0.04752,   0.00002,  -0.04370,  -0.00002,  -0.04564]])
                                        
    def start(self):
        self.task.start()

    def raw2force(self, raw):
        # raw is a 6x1 numpy array
        # Conversion from voltage to force in Newtons

        return self.cal_matrix @ raw

    def raws2force(self, raws):
        return np.matmul(self.cal_matrix, raws)

    def stop(self):
        self.task.stop()
        self.task.close()


class esc():
    """
    A class to control an ESC using NI-DAQmx.
    Methods
    -------
    __init__():
        Initializes the ESC task with a pulse channel and continuous timing.
    start():
        Starts the ESC task.
    stop():
        Stops the ESC task.
    write_pwm_ms(pwm_ms):
        Writes a PWM signal with the specified pulse width in milliseconds.
    write_throttle(throttle):
        Converts a throttle value to a PWM signal and writes it (0, 1)
    """

    def __init__(self):
        self.task = nidaqmx.Task()
        self.task.co_channels.add_co_pulse_chan_freq(
            "Dev1/ctr0", idle_state=Level.LOW, initial_delay=0.0,  # By default this is PFI12, which corresponds to pin 38 (P.2.4) on NI-USB-6212
            freq=50.0, duty_cycle=0.05  # Start at 0°
        )
        self.task.timing.cfg_implicit_timing(sample_mode=AcquisitionType.CONTINUOUS)
        time.sleep(1)

    def start(self):
        self.task.start() 
        self.write_pwm_ms(1000) # Start at 0°

    def stop(self):
        self.task.stop() 
        self.task.close()

    def write_pwm_ms(self, pwm_ms): 
        try:
            if pwm_ms >= 1000 and pwm_ms <= 2000:
                duty = pwm_ms / 20000 # 20 ms period
                self.task.write(CtrFreq(50, duty)) # 50 Hz frequency
            else:
                print("PWM value must be between 1000 and 2000 us.")
        except:
            pass

    def write_throttle(self, throttle):
        if throttle >= 0 and throttle <= 1:
            pwm = 1060 + throttle * 1000 # normalized (0, 1) to (1000, 2000) us
            pwm = max(1000, min(pwm, 2000))
            self.write_pwm_ms(pwm)
        else:
            print("Throttle value must be between 0 and 1.")


class PID_Controller():
    """
    A PID (Proportional-Integral-Derivative) controller class for feedback control systems.
    Attributes:
        kp (float): Proportional gain.
        ki (float): Integral gain.
        kd (float): Derivative gain.
        command (float): Desired setpoint value.
        sample_time (float): Time interval between updates.
        last_error (float): Error value from the previous update.
        integral (float): Accumulated integral of the error.
    Methods:
        __init__(kp, ki, kd, fs_rate):
            Initializes the PID controller with the given gains and sampling rate.
        update(feedback):
            Computes the control output based on the feedback value.
        set_command(command):
            Sets the desired setpoint value.
        reset():
            Resets the integral and last error values.
    """


    def __init__(self, kp, ki, kd, fs_rate):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.command = 0
        self.sample_time = 1 / fs_rate
        self.last_error = 0
        self.integral = 0
        self.output = 0
        self.min_output = 0
        self.max_output = 1
    

    def update(self, feedback):
        error = self.command - feedback
        self.integral += error * self.sample_time
        derivative = (error - self.last_error) / self.sample_time
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.last_error = error
        self.output = self.clip(output)
        return self.clip(output)

    def set_command(self, command):
        self.command = command

    def clip(self, value):
        return max(self.min_output, min(value, self.max_output))

    def reset(self):
        self.last_error = 0
        self.integral = 0

class indi_controller():
    def __init__(self, kp):
        self.kp = kp # Proportional gain
        self.command = 0 # Desired setpoint value
        self.u = 0 # Control output
        self.min_output = 0
        self.max_output = 1
    
    def set_command(self, command):
        self.command = command

    def update(self, feedback):
        error = self.command - feedback # Error value
        output = self.u + self.kp * error # Control output
        self.u = self.clip(output)  # Update the control output
        return self.clip(output) # Return the control output
    
    def clip(self, value):
        return max(self.min_output, min(value, self.max_output))

def ask_user(in_string):
    global running
    input(in_string)
    print("\n")
    running = False


def estimate_rpm(signal, Fs):
    # Find indices where the signal transitions from low to high (rising edges)
    # rising_edges = np.where((signal[:-1] < 3.5) & (signal[1:] >= 3.5))[0] + 1
    time_arr = np.linspace(0, len(signal) / Fs, len(signal))
    threshold = 2 # V
    indices = np.where(np.diff(signal) > threshold)[0]
    
    if len(indices) < 2:
        return 0
    
    min_gap = 1  # Adjust based on your data (e.g., number of samples)
    filtered_indices = indices[np.insert(np.diff(indices) > min_gap, 0, True)]

    rising_edges = time_arr[filtered_indices]

    # Compute time differences between rising edges
    time_diff = np.diff(rising_edges)[-1]

    # Compute average time between rising edges
    # avg_time = np.mean(time_diffs)
    rpm = 60 / time_diff
    return rpm

def J2RPM(J, V, D):
    n = V / (J*D)
    RPM = n * 60
    return RPM

def RPM2J(RPM, V, D):
    n = RPM / 60
    J = V / (n * D)
    return J 

def air_density(T, P, humidity):
    """
    Calculate the air density using temperature, pressure, and humidity.
    Parameters:
    T (float): Temperature in Celsius.
    P (float): Pressure in Pascals.
    humidity (float): Relative humidity in percentage (0-100).
    Returns:
    float: Air density in kg/m^3.
    """
    # Constants
    R_specific = 287.05  # Specific gas constant for dry air in J/(kg·K)
    R_vapor = 461.495  # Specific gas constant for water vapor in J/(kg·K)
    T_kelvin = T + 273.15  # Convert temperature to Kelvin

    # Saturation vapor pressure (in Pascals) using Tetens formula
    e_s = 6.112 * np.exp((17.67 * T) / (T + 243.5)) * 100

    # Actual vapor pressure (in Pascals)
    e = humidity / 100 * e_s

    # Partial pressure of dry air (in Pascals)
    P_dry = P - e

    # Air density calculation
    density = (P_dry / (R_specific * T_kelvin)) + (e / (R_vapor * T_kelvin))
    return density


def plot_exp_input(t_arr, J, rpm_sweep):
    fig, ax1 = plt.subplots(figsize = (12, 6))
    # plt.subplots_adjust(top=0.7, bottom=0.6)

    color = 'tab:blue'
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Advance Ratio (J)', color=color)
    ax1.plot(t_arr, J, '-', color=color, linewidth=2)
    ax1.tick_params(axis='y', labelcolor=color)

    ax2 = ax1.twinx()  # instantiate a second axes that shares the same x-axis
    color = 'tab:red'
    ax2.set_ylabel('RPM', color=color)  # we already handled the x-label with ax1
    ax2.plot(t_arr, rpm_sweep, '-', color=color, linewidth=2)
    ax2.tick_params(axis='y', labelcolor=color)

    fig.tight_layout()  # otherwise the right y-label is slightly clipped
    plt.title('Propeller RPM and Advance Ratio vs Time')
    plt.grid()

    # Add buttons
    ax_ack = plt.axes([0.6, 0.05, 0.1, 0.075])
    ax_term = plt.axes([0.81, 0.05, 0.1, 0.075])

    btn_ack = Button(ax_ack, 'Acknowledge', color='green', hovercolor='#aec7e8')
    btn_term = Button(ax_term, 'Terminate', color='red', hovercolor='#ff9896')

    plt.subplots_adjust(top=0.95, bottom=0.2, left=0.1, right=0.9)  # Add more padding

    def on_close(event):
        print("Experiment terminated:")
        print("Window closed manually.")
        sys.exit()  # Ensure the script exits when closed manually

    def terminate(event):
        print("Experiment terminated.")
        plt.close(fig)
        sys.exit()

    close_id = fig.canvas.mpl_connect('close_event', on_close)

    def acknowledge(event):
        print("Experiment acknowledged.")
        fig.canvas.mpl_disconnect(close_id)
        plt.close(fig)

    btn_ack.on_clicked(acknowledge)
    btn_term.on_clicked(terminate)

    plt.show()

