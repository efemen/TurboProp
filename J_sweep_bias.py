from acquisition_tools import *
from scipy import io
import time
from datetime import datetime

# def read_bias(prop_name):
#------------------------------ File Variables -----------------------------#

prop_name = "APC_10x10"
current_time = datetime.now().strftime("%Y%m%d_%H%M%S")
filename = prop_name + "_bias" + "_" + current_time + ".mat"


t = 0
datapoint = 0
Fs_analog = 1000 # Hz
Fs_control = 50 # Hz
dt_control = 1/Fs_control
duration = 10 # seconds
N = Fs_control * duration

description = "Propeller " + prop_name + " bias measurement."


# ----------------------------- Init Arrays ----------------------------- #

t_arr = np.zeros((1, 1))


buffer_in_size = Fs_analog // Fs_control 
# buff_t_arr = np.linspace(0, buffer_in_size/Fs_analog, buffer_in_size)
channels = 8

buffer_in = np.zeros((1, buffer_in_size))
data = np.zeros((channels, 1)) 

print("Starting bias measurement...")
# ----------------------------- Acquisition ----------------------------- #
analog_in = analog_force_reader(fs_rate= Fs_analog, buffer_size=buffer_in_size)


def reading_task_callback(task_idx, event_type, num_samples, callback_data):  # bufsize_callback is passed to num_samples
    global data
    global buffer_in

    if running:
        # It may be wiser to read slightly more than num_samples here, to make sure one does not miss any sample,
        # see: https://documentation.help/NI-DAQmx-Key-Concepts/contCAcqGen.html
        buffer_in = np.zeros((channels, num_samples)) 
        analog_in.stream.read_many_sample(buffer_in, num_samples, timeout=constants.WAIT_INFINITELY)

        data = np.append(data, buffer_in, axis=1)  # appends buffered data to total variable data

    return 0  # Absolutely needed for this callback to be well defined (see nidaqmx doc).

analog_in.task.register_every_n_samples_acquired_into_buffer_event(buffer_in_size, reading_task_callback)


# --------------------------------- Main Loop --------------------------------- #
start_time = time.perf_counter()
analog_in.start()
running = True
esc_1 = esc()
esc_1.start()
time.sleep(2)

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
    print("Task stopped.")
    esc_1.stop()
    running = False


analog_in.stop()


force_raw = analog_in.raw2force(data[0:6, :]).T

io.savemat("bias/" + filename, 
            {"t": t_arr, 
            "data": data.T,
            "force": force_raw,
            "mean_force": np.mean(force_raw, axis=0),
            "description" : description})
print("Data saved.")