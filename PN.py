# Proportional navigation class
import numpy as np
import math

class PN:

    def __init__(self,gain):
        # Variable to keep previous diff vector
        self.robot_target_diff_old = np.array([-79.8, 119.75])

        # Gain
        self.N = gain

    def PN_control(self, robot_action, robot_target_diff):
        
        # Normalising input vector and old vector diff
        robot_target_diff_unit = robot_target_diff / np.linalg.norm(robot_target_diff)
        robot_target_diff_old_unit = self.robot_target_diff_old / np.linalg.norm(self.robot_target_diff_old)

        # Computing dot product
        #dot_product = np.dot(robot_target_diff_unit, robot_target_diff_old_unit)
        a = robot_target_diff_unit
        b = robot_target_diff_old_unit
        # Computing angle between unit vectors based on dot product (in degrees)
        #error_angle = np.arctan2(a[1],a[0]) - np.arctan2(b[1],b[0])
        error_angle = -np.arctan2(a[0]*b[1] - a[1]*b[0], a[0]*b[0]+a[1]*b[1])

        print(error_angle)

        error_angle = self.N * error_angle

        # Creating rotation matrix to rotate robot action
        rot = np.array([[math.cos(error_angle), -math.sin(error_angle)], [math.sin(error_angle), math.cos(error_angle)]])

        # Rotating robot action vector
        robot_new_action = np.dot(rot,robot_action)

        self.robot_target_diff_old = robot_target_diff

        return robot_new_action