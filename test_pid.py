import time
import numpy as np
import cv2
import matplotlib.pyplot as plt
from PID import PID
from PN import PN
import seaborn as sn
import pandas as pd

from environment import Environment

# Initial variables
delay = 0 # How many discrete time steps of delay
trajectory = 0 # 0 for straight line, 1 for sine wave
draw = 0 # Illustration of pursuit shown

# Stored data
time_to_capture = np.zeros((15,10))

# Counters
P_count = 0
K_count = 0

for P in np.arange(0.1, 1.6, 0.1):
    P_count += 1
    K_count = 0
    for K in np.arange(0.5, 5.5, 0.5):
        K_count += 1
        
        print(P_count, K_count)
        # Create an environment
        environment = Environment(magnification=3, target_speed=0.095, traj=trajectory)

        # Creating controllers
        pid = PID(P, 0.0, K, s_steps=10)

        # Saving initial states of robot and target
        robot_state = environment.init_state
        target_state = environment.target_init_state

        # Initial robot action (will be determined by PID controller)
        action = np.array([0.0, 0.2])

        # Action history (used to simulate delay in discrete steps)
        actions = list()
        for i in range(1,delay):
            actions.append([0, 0])

        # Appending initial action
        actions.append(action)

        # Variable when simulation is finished
        sim_done = False

        # Counter during simulation
        count = 0

        # List contain error angles and distance throughout pursuit
        errors = list()
        distances = list()

        while not sim_done:
            # Stepping simulation
            robot_state, target_state, target_action, diff_vector, sim_done = environment.step(robot_state, actions[count], target_state)
            # Showing illustration of pursuit
            if draw:
                environment.show(robot_state, target_state)

            # Capture criteria
            if np.linalg.norm(robot_state - target_state) < 1.5:
                sim_done = True

            # Calculating robot action based off PID
            action, err = pid.PID_control(action, diff_vector, np.deg2rad(0), count)

            # Appending new action to list
            actions.append(action)

            # Appending latest error to list
            errors.append(err)
            distances.append(np.linalg.norm(diff_vector))

            count += 1
        
        time_to_capture[P_count-1][K_count-1] = float(count)

df_cm = pd.DataFrame(time_to_capture, [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4, 1.5], 
                                        np.arange(0.5, 5.5, 0.5))
sn.set(font_scale=1.0) # for label size
#sn.heatmap(df_cm, annot=True, annot_kws={"size": 10}, cmap="Blues", fmt='g', vmin=9.44, vmax=9.65, cbar=False)
sn.heatmap(df_cm, annot=True, annot_kws={"size": 10}, vmin=670, vmax=735, fmt='g', cbar=False)
plt.xlabel(r'$K_{D}$')
plt.ylabel(r'$K_{P}$', rotation=0)
plt.yticks(rotation=0) 

#plt.savefig('./nowhitespace.eps', bbox_inches='tight', pad_inches=0.01)
plt.show()