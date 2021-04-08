import time
import numpy as np
import cv2
from PID import PID
from PN import PN

from environment import Environment

# Create an environment
environment = Environment(magnification=3, target_speed=0.2)

# Creating controllers
pid = PID(0.7, 0.0, 0.60)
pn = PN(3)

# Saving initial states of robot and target
robot_state = environment.init_state
target_state = environment.target_init_state

# Robot action (will be determined by PID controller)
action = np.array([0.3,0.3])

# Variable when simulation is finished
sim_done = False

while not sim_done:
    robot_state, target_state, target_action, diff_vector, sim_done = environment.step(robot_state, action, target_state)
    environment.show(robot_state, target_state)

    if np.linalg.norm(robot_state - target_state) < 0.1:
        sim_done = True

    # Calculating robot action based off PID
    #action = pid.PID_control(action, diff_vector)

    # Calculating robot action based off PN
    action = pn.PN_control(action, diff_vector)

cv2.waitKey(0)
