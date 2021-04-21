import time
import numpy as np
import cv2
import matplotlib.pyplot as plt
from PID import PID
from PN import PN

from environment import Environment

# Initial variables
delay = 0 # How many discrete time steps of delay
trajectory = 0 # 0 for straight line, 1 for sine wave

# Create an environment
environment = Environment(magnification=3, target_speed=0.2, traj=trajectory)

# Creating controllers
pid = PID(0.1, 0.0, 0.0)
pn = PN(3)

# Saving initial states of robot and target
robot_state = environment.init_state
target_state = environment.target_init_state

# Initial robot action (will be determined by PID controller)
action = np.array([0.3, 0.3])

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

# List contain error angles throughout pursuit
errors = list()

while not sim_done:
    # Stepping simulation
    robot_state, target_state, target_action, diff_vector, sim_done = environment.step(robot_state, actions[count], target_state)
    # Showing illustration of pursuit
    environment.show(robot_state, target_state)

    # Capture criteria
    if np.linalg.norm(robot_state - target_state) < 0.5:
        sim_done = True

    # Calculating robot action based off PID
    action, err = pid.PID_control(action, diff_vector, np.deg2rad(0))

    # Calculating robot action based off PN
    #action = pn.PN_control(action, diff_vector)

    # Appending new action to list
    actions.append(action)

    # Appending latest error to list
    errors.append(err)

    count += 1

cv2.waitKey(1)

# Plotting results
plt.plot(range(1,count), errors[:-1])
plt.show()