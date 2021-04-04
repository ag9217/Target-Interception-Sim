import time
import numpy as np
import cv2

from environment import Environment

# Create an environment
environment = Environment(magnification=3, target_speed=0.2)

# Saving initial states of robot and target
robot_state = environment.init_state
target_state = environment.target_init_state

# Robot action (will be determined by PID controller)
action = np.array([0.1,0.1])

# Variable when simulation is finished
sim_done = False

while not sim_done:
    robot_state, target_state, target_action, dist, sim_done = environment.step(robot_state, action, target_state)
    environment.show(robot_state, target_state)

cv2.waitKey(0)
