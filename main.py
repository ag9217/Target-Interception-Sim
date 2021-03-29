import time
import numpy as np

from environment import Environment

# Create an environment
environment = Environment(magnification=3, target_speed=0.1)

# Saving initial states of robot and target
robot_state = environment.init_state
target_state = environment.target_init_state

# Robot action (will be determined by PID controller)
action = np.array([0.1,0.1])

while True:
    robot_state, target_state, dist = environment.step(robot_state, action, target_state)
    environment.show(robot_state, target_state)
    print(dist)