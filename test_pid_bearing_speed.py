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
delay = 10 # How many discrete time steps of delay
trajectory = 1 # 0 for straight line, 1 for sine wave
draw = 0 # Illustration of pursuit shown

count1 = 0
count2 = 0

best_bearing = np.zeros((5,6))

for agent_speed in np.arange(0.04, 0.24, 0.04):
    count1 += 1
    count2 = 0
    for target_speed in np.arange(0.1, 0.22, 0.02):
        count2 += 1
        # Saving counts
        counts = list()
        for bearing in np.arange(0, 65, 5):
            print(count1, count2, bearing)

            # Create an environment
            environment = Environment(magnification=3, target_speed=target_speed, traj=trajectory)

            # Creating controllers
            pid = PID(1.4, 0.0, 0.5, s_steps=10)

            # Saving initial states of robot and target
            robot_state = environment.init_state
            target_state = environment.target_init_state

            # Initial robot action (will be determined by PID controller)
            action = np.array([0.0, agent_speed])

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
                action, err = pid.PID_control(action, diff_vector, np.deg2rad(bearing), count)

                # Appending new action to list
                actions.append(action)

                # Appending latest error to list
                errors.append(err)
                distances.append(np.linalg.norm(diff_vector))

                count += 1

            counts.append(count)

        
        # Append best performing angle to best_bearing
        repeating = all(element == counts[0] for element in counts)
        print(repeating)
        if repeating:
            best_bearing[count1 - 1][count2 - 1] = -1
        else:
            best_bearing[count1 - 1][count2 - 1] = min(counts)

# Plotting results
df_cm = pd.DataFrame(best_bearing, np.arange(0.04, 0.24, 0.04), 
                                        [0.1, 0.12, 0.14, 0.16, 0.18, 0.2])
sn.set(font_scale=1.0) # for label size
#sn.heatmap(df_cm, annot=True, annot_kws={"size": 10}, cmap="Blues", fmt='g', vmin=9.44, vmax=9.65, cbar=False)
sn.heatmap(df_cm, annot=True, annot_kws={"size": 10}, fmt='g', cbar=False)
plt.ylabel(r'Agent Speed')
plt.xlabel(r'Target Speed')
plt.yticks(rotation=0) 

#plt.savefig('./PID_speed_sine_10delay.eps', bbox_inches='tight', pad_inches=0.01)
plt.show()
