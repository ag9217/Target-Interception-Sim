import time
import numpy as np
import cv2
from PID import PID

from environment import Environment

# Create an environment
environment = Environment(magnification=3, target_speed=0.2)

# Creating PID controller
pid = PID(1, 0.0, 0.02)

# Saving initial states of robot and target
robot_state = environment.init_state
target_state = environment.target_init_state

# Robot action (will be determined by PID controller)
action = np.array([0.25,0.25])

# Variable when simulation is finished
sim_done = False

while not sim_done:
    robot_state, target_state, target_action, diff_vector, sim_done = environment.step(robot_state, action, target_state)
    environment.show(robot_state, target_state)

    # Calculating robot action based off PID
    action = pid.PID_control(action, diff_vector)

cv2.waitKey(0)
