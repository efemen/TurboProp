import numpy as np
import subprocess


J_mins = np.array([0.16, 0.2, 0.24, 0.28, .32, .36, .40, .44, .48, .52, .56])
# J_mins = np.array([.40, .44, .48, .52, .56, .60])
# J_mins = np.array([.15, 0.19, 0.23, 0.27, 0.31, 0.35])
# J_mins = np.array([0.16, 0.13,])
J_maxs = np.round(J_mins + 0.02, 2)

print(J_mins)
print(J_maxs)

for current_run in range(0, len(J_mins)):

    with open("J_sweep_Turbulent.py", "r") as file:
        lines = file.readlines()

    with open("J_sweep_Turbulent.py", "w") as file:
        for line in lines:
            if "J_min =" in line:
                file.write("J_min = " + str(J_mins[current_run]) + "\n")
            elif "J_max =" in line:
                file.write("J_max = " + str(J_maxs[current_run]) + "\n")
            else:
                file.write(line)

    print("Running J_min = ", J_mins[current_run], " J_max = ", J_maxs[current_run])
    subprocess.run(['M:/Efe/TurboProp/venv/Scripts/python.exe', 'J_sweep_Turbulent.py'])