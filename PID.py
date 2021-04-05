import numpy as np
import math

def PID_control(robot_action, robot_target_diff):
    # PID Gains
    K_p = 1
    K_I = 0.1
    K_D = 0.1

    # Creating unit vectors
    robot_action_unit = robot_action / np.linalg.norm(robot_action)
    robot_target_diff_unit = robot_target_diff / np.linalg.norm(robot_target_diff)

    # Computing dot product
    dot_product = np.dot(robot_action_unit, robot_target_diff_unit)
    # Computing angle between unit vectors based on dot product (in degrees)
    error_angle = np.arccos(dot_product)

    # Condition when robot should turn left
    if robot_action_unit[0] < robot_target_diff_unit[0] or robot_action_unit[1] > robot_target_diff_unit[1]:
        error_angle = -error_angle

    # Calculating PID output
    PID_h = K_p * error_angle

    # Creating rotation matrix to rotate robot action
    rot = np.array([[math.cos(PID_h), -math.sin(PID_h)], [math.sin(PID_h), math.cos(PID_h)]])

    # Rotating robot action vector
    robot_new_action = np.dot(rot,robot_action)

    return robot_new_action
