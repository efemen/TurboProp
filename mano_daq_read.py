from acquisition_tools import *
from scipy import signal, io
import time
from datetime import datetime
import matplotlib.pyplot as plt


D_inch = 10 # Propeller Diameter
D_m = D_inch * 0.0254 # Propeller diameter in meters
pitch = 5 # Propeller Pitch

num_ops = 1 # Number of operating points to be tested
J_min = 0.4
J_max = 0.4
station_time = 3 # seconds per operating point

#------------------------------ File Variables -----------------------------#

t = 0
datapoint = 0
Fs_control = 50 # Hz
Fs_mano = 300 # Hz

dt_control = 1/Fs_control
duration = num_ops * station_time # seconds
N = Fs_control * duration


# ----------------------------- Init Arrays ----------------------------- #

t_arr = np.zeros((1, 1))


buffer_in_size = 250

channels_mano = 2

buffer_in_mano = np.zeros((1, buffer_in_size))

data_mano_daq = np.zeros((channels_mano, 1))



analog_mano_in = analog_mano_reader(fs_rate= Fs_mano, buffer_size=buffer_in_size)


def reading_task_callback_mano(task_idx, event_type, num_samples, callback_data):  # bufsize_callback is passed to num_samples
    global data_mano_daq
    global buffer_in_mano

    if running:
        # It may be wiser to read slightly more than num_samples here, to make sure one does not miss any sample,
        # see: https://documentation.help/NI-DAQmx-Key-Concepts/contCAcqGen.html
        buffer_in_mano = np.zeros((channels_mano, num_samples)) 
        analog_mano_in.stream.read_many_sample(buffer_in_mano, num_samples, timeout=constants.WAIT_INFINITELY)

        data_mano_daq = np.append(data_mano_daq, buffer_in_mano, axis=1)  # appends buffered data to total variable data
        print("hehe")

    return 0  # Absolutely needed for this callback to be well defined (see nidaqmx doc).

analog_mano_in.task.register_every_n_samples_acquired_into_buffer_event(buffer_in_size, reading_task_callback_mano)


# --------------------------------- Main Loop --------------------------------- #
start_time = time.perf_counter()
analog_mano_in.start()

running = True


try:
    while t < duration:
        t = time.perf_counter() - start_time
        t_arr = np.append(t_arr, t)


        datapoint += 1
        if datapoint >= N:
            break
        target_time = start_time + (datapoint * dt_control)
        while time.perf_counter() < target_time:
            pass  # Active wait


except KeyboardInterrupt:
    print("Interrupted by user.")
    

finally:

    print("ESC stopped.")
    print("Task stopped.")
    running = False

analog_mano_in.stop()


# import matplotlib.pyplot as plt
# fig, ax = plt.subplots()


# # ax.plot(data[5, :].T, ".-")
# ax.plot(t_arr, data_mano_daq, ".-")
# plt.show()

