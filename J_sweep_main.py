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
print("Target q = ", 0.5 * rho * U_inf**2)

D_inch = 10 # Propeller Diameter
D_m = D_inch * 0.0254 # Propeller diameter in meters
pitch = 5 # Propeller Pitch

num_ops = 1 # Number of operating points to be tested
J_min = 0.4
J_max = 0.4
station_time = 200 # seconds per operating point

#------------------------------ File Variables -----------------------------#
prop_name = str(D_inch) + "x" + str(pitch)
current_time = datetime.now().strftime("%Y%m%d_%H%M%S")
file_name = prop_name + "_J_" + str(J_min) + "_" + str(J_max) + "_" + current_time + ".mat"


bias_file = get_latest_file()
print("Bias file: ", bias_file)
bias_data = io.loadmat(bias_file)
bias_mean = bias_data["mean_force"]

t = 0
datapoint = 0
Fs_analog = 20000 # Hz
Fs_control = 50 # Hz
dt_control = 1/Fs_control
duration = num_ops * station_time # seconds
N = Fs_control * duration

description = "Propeller " + prop_name + " J sweep from " + str(J_min) + " to " +  str(J_max) + " "+ str(duration) + " seconds."


# ----------------------------- Init Arrays ----------------------------- #

t_arr = np.zeros((1, 1))

J_sweep = np.flip(np.repeat(np.linspace(J_min, J_max, num_ops), N//num_ops)) # Advance ratio sweep
rpm_sweep = J2RPM(J_sweep, U_inf, D_m) # RPM sweep

rpm_commands = rpm_sweep
rpm_command = rpm_commands[0]

rpms = np.zeros((1, 1))

buffer_in_size = 2500
# buff_t_arr = np.linspace(0, buffer_in_size/Fs_analog, buffer_in_size)
channels = 7

buffer_in = np.zeros((1, buffer_in_size))
data = np.zeros((channels, 1)) 

# ----------------------------- Controller  ----------------------------- #

kp_indi = 5e-6 # APC 10x5
controller = indi_controller(kp_indi)

# ----------------------------- Filter  ----------------------------- #

dt = 1/Fs_control
order = 2
cutoff = 20
nyq = 0.5 * Fs_control
normal_cutoff = cutoff / nyq

b, a = signal.butter(order, normal_cutoff)
z_l = z_d = signal.lfilter_zi(b, a)


# #------------------------------ Define equipment classes -----------------------------#
plot_exp_input_rpm(np.linspace(0, duration, N), J_sweep, rpm_sweep)


esc_1 = esc()
esc_1.start()

WT.set_U0(U_inf)
print("Wind tunnel starting...")
time.sleep(20)

print("Throttling up...")
esc_1.write_throttle(0.3)
print("Working RPM set.")
time.sleep(3)

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

controller.set_command(rpm_command)

try:
    while t < duration:
        t = time.perf_counter() - start_time
        t_arr = np.append(t_arr, t)

        # Do whatever you want here:

        rpm = estimate_rpm(data[6, :], Fs_analog)
        rpms = np.append(rpms, rpm)

        rpm_filtered, z_l = signal.lfilter(b, a, rpms, zi=z_l)

        # rpms = np.append(rpms, rpm)
        # esc_1.write_pwm_ms(pwm_commands[datapoint])
    
        throttle_command = controller.update(rpm_filtered[-1])
        controller.set_command(rpm_commands[datapoint])
        esc_1.write_throttle(throttle_command)

        print("RPM: ", rpms[datapoint], "Throttle: ", throttle_command)

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

analog_in.stop()

force_raw = analog_in.raw2force(data[0:6, :]).T

import matplotlib.pyplot as plt
fig, ax = plt.subplots()


# ax.plot(data[5, :].T, ".-")
ax.plot(t_arr, rpms, ".-")
plt.show()

io.savemat("results/" + prop_name + "/" + file_name, 
            {"t": t_arr, 
                "data": data.T,
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