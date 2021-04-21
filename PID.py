import numpy as np
import math

class PID:

    def __init__(self, Kp, Ki, Kd):
        # PID Gains
        self.K_p = Kp
        self.K_i = Ki
        self.K_d = Kd

        # Memory for I and D controllers
        self.prev_angle = 0
        self.error_sum = 0

    def PID_control(self, robot_action, robot_target_diff, const_err):

        # Creating unit vectors
        robot_action_unit = robot_action / np.linalg.norm(robot_action)
        robot_target_diff_unit = robot_target_diff / np.linalg.norm(robot_target_diff)

        # Computing dot product
        dot_product = np.dot(robot_action_unit, robot_target_diff_unit)
        # Computing angle between unit vectors based on dot product
        error_angle = np.arccos(dot_product)
        print(np.rad2deg(error_angle))

        # Saving error for integral term
        self.error_sum += error_angle

        # Condition when robot should turn left
        if robot_action_unit[0] < robot_target_diff_unit[0] or robot_action_unit[1] > robot_target_diff_unit[1]:
            error_angle = -error_angle

        error_angle =  error_angle - const_err

        # Calculating PID output
        PID_h = self.K_p * error_angle + self.K_i * self.error_sum + self.K_d * (error_angle - self.prev_angle)

        # Creating rotation matrix to rotate robot action
        rot = np.array([[math.cos(PID_h), -math.sin(PID_h)], [math.sin(PID_h), math.cos(PID_h)]])

        # Rotating robot action vector
        robot_new_action = np.dot(rot,robot_action)

        self.prev_angle = error_angle

        return robot_new_action
