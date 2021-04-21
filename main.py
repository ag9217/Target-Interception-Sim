import time
import numpy as np
import cv2
import matplotlib.pyplot as plt
from PID import PID
from PN import PN

from environment import Environment

# Initial variables
delay = 0 # How many discrete time steps of delay
sim_done = False # Variable when simulation is finished
count = 0 # Counter during simulation

# Create an environment
environment = Environment(magnification=3, target_speed=0.2, traj=0)

# Creating controllers
pid = PID(0.7, 0.0, 0.60)
pn = PN(3)

# Saving initial states of robot and target
robot_state = environment.init_state
target_state = environment.target_init_state

# Initial robot action (will be determined by PID controller)
action = np.array([0.3,0.3])

# Action history (used to simulate delay in discrete steps)
actions = list()
for i in range(1,delay):
    actions.append([0, 0])

# Appending initial action
actions.append(action)

while not sim_done:
    robot_state, target_state, target_action, diff_vector, sim_done = environment.step(robot_state, actions[count], target_state)
    environment.show(robot_state, target_state)

    if np.linalg.norm(robot_state - target_state) < 0.5:
        sim_done = True

    # Calculating robot action based off PID
    action = pid.PID_control(action, diff_vector, np.deg2rad(0))

    # Calculating robot action based off PN
    #action = pn.PN_control(action, diff_vector)

    # Appending new action to list
    actions.append(action)

    count += 1

cv2.waitKey(0)