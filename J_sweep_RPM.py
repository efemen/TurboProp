from acquisition_tools import *
from scipy import signal, io
import time
from datetime import datetime
import matplotlib.pyplot as plt


#------------------------------ Experiment Variables -----------------------------#
WT = wind_tunnel()
U_inf = 6 # m/s

print("Reading WT temperature and pressure...")
temperature, pressure = WT.read_PLC()
humidty = 0.71 # Relative humidity

rho = air_density(temperature, pressure, humidty) * 100 # Air density
q_target = 0.5 * rho * U_inf**2
print("Target q = ", str(q_target))

D_inch = 10 # Propeller Diameter
D_m = D_inch * 0.0254 # Propeller diameter in meters
pitch = 10 # Propeller Pitch

num_ops = 6 # Number of operating points to be tested
J_min = 0.2
J_max = 0.32
station_time = 30 # seconds per operating point

#------------------------------ File Variables -----------------------------#
prop_name = str(D_inch) + "x" + str(pitch)
current_time = datetime.now().strftime("%Y%m%d_%H%M%S")
file_name = prop_name + "_J_" + str(J_min) + "_" + str(J_max) + "_" + current_time + ".mat"


# bias_file = get_latest_file()
bias_file = run_bias()
print("Bias file: ", bias_file)
bias_data = io.loadmat(bias_file)
bias_mean = bias_data["mean_force"]

t = 0
datapoint = 0
Fs_analog = 20000 # Hz
Fs_control = 20 # Hz
Fs_mano = 1000 # Hz

dt_control = 1/Fs_control
duration = num_ops * station_time # seconds
N = Fs_control * duration
N_analog = Fs_analog * duration
N_mano = Fs_mano * duration

description = "Propeller " + prop_name + " J sweep from " + str(J_min) + " to " +  str(J_max) + " "+ str(duration) + " seconds."
# description = "Only weight."

# ----------------------------- Init Arrays ----------------------------- #

t_arr = np.zeros((1, 1))

J_sweep = np.flip(np.repeat(np.linspace(J_min, J_max, num_ops), N//num_ops)) # Advance ratio sweep
rpm_sweep = J2RPM(J_sweep, U_inf, D_m) # RPM sweep

rpm_commands = rpm_sweep
rpm_command = rpm_commands[0]

rpms = np.zeros((1, 1))

buffer_in_size_force = 1000
buffer_in_size_mano = 100

channels_force = 8
channels_mano = 2

last_index_force = 0
last_index_mano = 0

buffer_in_force = np.zeros((1, buffer_in_size_force))
buffer_in_mano = np.zeros((1, buffer_in_size_mano))

# data_force_daq = np.zeros((channels_force, 1)) 
# data_mano_daq = np.zeros((channels_mano, 1))

data_force_daq = np.zeros((channels_force, int(N_analog * 1.05))) 
data_mano_daq = np.zeros((channels_mano, int(N_mano * 1.05)))

# ----------------------------- Controller  ----------------------------- #

kp_indi = 4e-6 # APC 10x5
controller = indi_controller(kp_indi)

kp_tunnel = 2e-2
wt_controller = indi_controller(kp_tunnel, min_output = 0, max_output = 20)

# ----------------------------- Filter  ----------------------------- #

dt = 1/Fs_control
order = 2
cutoff = 5
nyq = 0.5 * Fs_control
normal_cutoff = cutoff / nyq

b, a = signal.butter(order, normal_cutoff)
z_l = z_d = signal.lfilter_zi(b, a)


# #------------------------------ Define equipment classes -----------------------------#
plot_exp_input_rpm(np.linspace(0, duration, N), J_sweep, rpm_sweep)


esc_1 = esc()
esc_1.start()

# WT.set_U0(U_inf)
# wt_controller.u = round(WT.fit[0] * U_inf + WT.fit[1], 1)

WT.set_fan_speed(17.5)
wt_controller.u = 17.5

print("Wind tunnel starting...")
time.sleep(25)

print("Throttling up...")
esc_1.write_throttle(0.1)
print("Working RPM set.")
time.sleep(3)

analog_force_in = analog_force_reader(fs_rate= Fs_analog, buffer_size=buffer_in_size_force)
analog_mano_in = analog_mano_reader(fs_rate= Fs_mano, buffer_size=buffer_in_size_mano)

def reading_task_callback_force(task_idx, event_type, num_samples, callback_data):  # bufsize_callback is passed to num_samples
    global data_force_daq
    global buffer_in_force
    global last_index_force

    if running:
        # It may be wiser to read slightly more than num_samples here, to make sure one does not miss any sample,
        # see: https://documentation.help/NI-DAQmx-Key-Concepts/contCAcqGen.html
        buffer_in_force = np.zeros((channels_force, num_samples)) 
        analog_force_in.stream.read_many_sample(buffer_in_force, num_samples, timeout=constants.WAIT_INFINITELY)

        # data_force_daq = np.append(data_force_daq, buffer_in_force, axis=1)  # appends buffered data to total variable data
        data_force_daq[:, last_index_force:last_index_force + num_samples] = buffer_in_force
        last_index_force += num_samples

    return 0  # Absolutely needed for this callback to be well defined (see nidaqmx doc).


def reading_task_callback_mano(task_idx, event_type, num_samples, callback_data):  # bufsize_callback is passed to num_samples
    global data_mano_daq
    global buffer_in_mano
    global last_index_mano

    if running:
        # It may be wiser to read slightly more than num_samples here, to make sure one does not miss any sample,
        # see: https://documentation.help/NI-DAQmx-Key-Concepts/contCAcqGen.html
        buffer_in_mano = np.zeros((channels_mano, num_samples)) 
        analog_mano_in.stream.read_many_sample(buffer_in_mano, num_samples, timeout=constants.WAIT_INFINITELY)

        # data_mano_daq = np.append(data_mano_daq, buffer_in_mano, axis=1)  # appends buffered data to total variable data
        data_mano_daq[:, last_index_mano:last_index_mano + num_samples] = buffer_in_mano
        last_index_mano += num_samples

    return 0  # Absolutely needed for this callback to be well defined (see nidaqmx doc).

analog_force_in.task.register_every_n_samples_acquired_into_buffer_event(buffer_in_size_force, reading_task_callback_force)
analog_mano_in.task.register_every_n_samples_acquired_into_buffer_event(buffer_in_size_mano, reading_task_callback_mano)


# --------------------------------- Main Loop --------------------------------- #
start_time = time.perf_counter()
analog_force_in.start()
analog_mano_in.start()

running = True

controller.set_command(rpm_command)
wt_controller.set_command(q_target)

try:
    while t < duration:
        t = time.perf_counter() - start_time
        t_arr = np.append(t_arr, t)

        # Do whatever you want here:

        # Estimate RPM
        rpm = estimate_rpm(data_force_daq[6, :], Fs_analog)
        rpms = np.append(rpms, rpm)
        rpm_filtered, z_l = signal.lfilter(b, a, rpms, zi=z_l)

        throttle_command = controller.update(rpm_filtered[-1])
        controller.set_command(rpm_commands[datapoint])
        esc_1.write_throttle(throttle_command)

        # WT control
        if datapoint % 10  == 0 and datapoint > 30:
            # q = np.mean(data_mano_daq[1, -100:]) * 200
            q = np.mean(data_mano_daq[1, data_mano_daq[1, :] != 0][-10:]) * 200
            wt_command = wt_controller.update(q)
            WT.set_fan_speed(round(wt_command, 2))
            print(1/np.mean(np.diff(t_arr[-9:-3])), q, wt_controller.error)
            # print("RPM: ", rpms[datapoint], "Error: ", controller.error, "Throttle: ", throttle_command)

        datapoint += 1
        if datapoint >= N:
            break
        target_time = start_time + (datapoint * dt_control)
        while time.perf_counter() < target_time:
            pass  # Active wait


except KeyboardInterrupt:
    print("Interrupted by user.")
    

finally:
    esc_1.stop()
    print("ESC stopped.")
    print("Task stopped.")
    WT.stop()
    WT.close()
    running = False

analog_force_in.stop()
analog_mano_in.stop()
data_force_daq = data_force_daq[:, 0:last_index_force]
data_mano_daq = data_mano_daq[:, 0:last_index_mano]

force_raw = analog_force_in.raw2force(data_force_daq[0:6, :]).T

import matplotlib.pyplot as plt
fig, ax = plt.subplots()

ax.plot(t_arr, rpms, ".-")
plt.grid()
plt.xlabel("Time [s]")
plt.ylabel("RPM")
plt.show()

io.savemat("results/" + prop_name + "/" + file_name, 
            {"t": t_arr, 
                "data": data_force_daq.T,
                "data_mano": data_mano_daq.T,
                "station_time": station_time,
                "force": force_raw,
                "force_net": force_raw - bias_mean,
                "rpm": rpms, 
                "rpm_command": rpm_commands,
                "Fs_analog" : Fs_analog, 
                "Fs_control" : Fs_control, 
                "kp_indi" : kp_indi,
                "J_sweep" : J_sweep,
                "rpm_sweep" : rpm_sweep,
                "D_inch" : D_inch,
                "pitch" : pitch,
                "U_inf" : U_inf,
                "temperature" : temperature,
                "pressure" : pressure,
                "humidity" : humidty,
                "rho" : rho,
                "bias_mean" : bias_mean,
                "description" : description})
print("Data saved.")