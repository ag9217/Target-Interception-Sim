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

# Saving counts
counts = list()

for bearing in np.arange(0, 60, 5):
    
    # Create an environment
    environment = Environment(magnification=3, target_speed=0.15, traj=trajectory)

    # Creating controllers
    pid = PID(1.4, 0.0, 0.5, s_steps=10)

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
        action, err = pid.PID_control(action, diff_vector, np.deg2rad(bearing), count)

        # Appending new action to list
        actions.append(action)

        # Appending latest error to list
        errors.append(err)
        distances.append(np.linalg.norm(diff_vector))

        count += 1

    
    #plt.plot(range(1,count), errors[:-1])
    plt.plot(range(1,count-9), np.convolve(errors[:-1], np.ones(10), 'valid') / 10, label=str(bearing) + "°")
    counts.append(count)

print(counts)

# Plotting results
# Error angle vs Time
plt.ylabel('Error angle (°)')
plt.xlabel('Time (steps) ')
plt.grid(linestyle="--")
plt.legend()
plt.savefig('./constant_bearing.eps', bbox_inches='tight', pad_inches=0.01)
plt.show()
