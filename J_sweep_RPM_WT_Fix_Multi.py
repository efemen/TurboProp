import numpy as np
import subprocess


J_mins = np.array([0.24, .28, 0.32, .36, .40, .44, .48, .60])
# J_mins = np.array([0.18])
J_maxs = J_mins + 0.06

print(J_mins)
print(J_maxs)

for current_run in range(0, len(J_mins)):

    with open("J_sweep_RPM_WT_Fix.py", "r") as file:
        lines = file.readlines()

    with open("J_sweep_RPM_WT_Fix.py", "w") as file:
        for line in lines:
            if "J_min =" in line:
                file.write("J_min = " + str(J_mins[current_run]) + "\n")
            elif "J_max =" in line:
                file.write("J_max = " + str(J_maxs[current_run]) + "\n")
            else:
                file.write(line)

    print("Running J_min = ", J_mins[current_run], " J_max = ", J_maxs[current_run])
    subprocess.run(['M:/Efe/TurboProp/venv/Scripts/python.exe', 'J_sweep_RPM_WT_Fix.py'])