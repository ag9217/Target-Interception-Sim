import numpy as np
import math
import time

class PID:

    def __init__(self, Kp, Ki, Kd, s_time):
        # PID Gains
        self.K_p = Kp
        self.K_i = Ki
        self.K_d = Kd

        # Memory for I and D controllers
        self.prev_angle = 0
        self.error_sum = 0

        # Times
        self.current_time = time.time()
        self.prev_time = self.current_time
        self.sample_time = s_time # How often to update PID

    def PID_control(self, robot_action, robot_target_diff, const_err):

        robot_new_action = robot_action # Will be changed once PID is called

        # Creating unit vectors
        robot_action_unit = robot_action / np.linalg.norm(robot_action)
        robot_target_diff_unit = robot_target_diff / np.linalg.norm(robot_target_diff)
        
        # Current time when doing PID
        self.current_time = time.time()
        delta_time = self.current_time - self.prev_time

        if delta_time >= self.sample_time:

            # Creating variables with smaller names
            a = robot_action_unit
            b = robot_target_diff_unit

            # Computing angle between unit vectors based on dot product
            error_angle = np.arctan2(a[0]*b[1] - a[1]*b[0], a[0]*b[0]+a[1]*b[1]) - const_err

            # Saving error for integral term
            self.error_sum += error_angle

            # Calculating PID output
            PID_h = self.K_p * error_angle + self.K_i * self.error_sum + self.K_d * (error_angle - self.prev_angle)

            # Creating rotation matrix to rotate robot action
            rot = np.array([[math.cos(PID_h), -math.sin(PID_h)], [math.sin(PID_h), math.cos(PID_h)]])

            # Rotating robot action vector
            robot_new_action = np.dot(rot,robot_action)

            self.prev_angle = error_angle

            self.prev_time = self.current_time

        # Information for plotting later
        robot_action_unit = robot_action / np.linalg.norm(robot_action)
        # Computing dot product
        dot_product = np.dot(robot_action_unit, robot_target_diff_unit)
        # Computing angle between unit vectors based on dot product
        err = np.rad2deg(np.arccos(dot_product))

        return robot_new_action, err
