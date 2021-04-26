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
draw = 1 # Illustration of pursuit shown
controller = 'PID' # Choose which controller to use, 'PN' for proportional nav
bearing_angle = 10 # Bearing angle for PID

# Create an environment
environment = Environment(magnification=3, target_speed=0.1549, traj=trajectory)

# Creating controllers
pid = PID(1.6, 0.0, 0, s_steps=10)
pn = PN(5, s_steps = 1)

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
    if controller == 'PID':
        action, err = pid.PID_control(action, diff_vector, np.deg2rad(bearing_angle), count)

    # Calculating robot action based off PN
    if controller == 'PN':
        action, err = pn.PN_control(action, diff_vector, count)

    # Appending new action to list
    actions.append(action)

    # Appending latest error to list
    errors.append(err)
    distances.append(np.linalg.norm(diff_vector))

    count += 1

cv2.waitKey(1)


# Plotting results
# Error angle vs Time
plot1 = plt.figure(1)
plt.plot(range(1,count), errors[:-1], 'b')
plt.ylabel('Error angle (°)')
plt.xlabel('Time (steps) ')
plt.grid(linestyle="--")

# Distance vs Time
plot1 = plt.figure(2)
plt.plot(range(1,count), distances[:-1], 'b')
plt.ylim([0,180])
plt.ylabel('Distance (cm)')
plt.xlabel('Time (steps) ')
plt.grid(linestyle="--")

print("Steps to capture: ", count)
plt.show()