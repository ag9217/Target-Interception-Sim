import numpy as np
import cv2

class Environment:
    # Initialising environment object
    def __init__(self, magnification, target_speed):
        # Set magnification factor of the display
        self.magnification = magnification
        # Set width and height of environment
        self.width = 330.0
        self.height = 160.0
        # Creating image used to display the environment
        self.image = np.zeros([int(self.magnification * self.height), int(self.magnification * self.width), 3], dtype=np.uint8)
        # Defining initial variables for environment
        self.init_state = None
        self.target_init_state = None
        self.target_speed = target_speed
        self._create_environment_space()
        # Variables for saving coordinates and plotting later
        self.robot_pos = []
        self.target_pos = []
        self.distances = []
        # Flag for when simulation is done
        self.sim_is_done = False

    # Creating environment space
    def _create_environment_space(self):
        # Set the initial state of the agent
        init_state_x = 80.0
        init_state_y = 0.0
        self.init_state = np.array([init_state_x, init_state_y], dtype=np.float32)

        target_init_state_x = 0.0
        target_init_state_y = 120.0
        self.target_init_state = np.array([target_init_state_x, target_init_state_y], dtype=np.float32)

    # Function to reset the environment, returns initial state of pursuiter
    def reset(self):
        return self.init_state

    def step(self, state, action, target_state):
        # Determine the next state if the action with be used
        next_state = state + action
        # If robot state is outside the environment, the make robot stay still
        if next_state[0] < 0.0 or next_state[0] > 330.0 or next_state[1] < 0.0 or next_state[1] > 160.0:
            next_state = state

        # Calculating new position of target (following sine wave)
        t_mov_speed_x = self.target_speed
        target_next_state = np.array([target_state[0] + t_mov_speed_x, self.target_init_state[1] + 40 * np.sin((target_state[0] * np.pi)/60)], dtype=np.float32)
        # Computing distance to target (not seen by robot, just for debugging)
        distance_to_target = np.linalg.norm(next_state - target_next_state)

        # Flag for when simulation is done
        if target_next_state[0] < 0.0 or target_next_state[0] > 330.0 or target_next_state[1] < 0.0 or target_next_state[1] > 160.0:
            self.sim_is_done = True

        return next_state, target_next_state, distance_to_target, self.sim_is_done

    def show(self, robot_state, target_state):
        # Create background
        window_top_left = (0, 0)
        window_bottom_right = (self.magnification * int(self.width), self.magnification * int(self.height))
        cv2.rectangle(self.image, window_top_left, window_bottom_right, (255, 255, 255), thickness=cv2.FILLED)

        # Drawing robot / pursuer
        robot_centre = (int(robot_state[0] * self.magnification), int((int(self.height) - robot_state[1]) * self.magnification))
        robot_radius = int(3 * self.magnification)
        robot_colour = (230, 20, 70)
        cv2.circle(self.image, robot_centre, robot_radius, robot_colour, cv2.FILLED)

        # Draw target
        target_centre = (int(target_state[0] * self.magnification), int((int(self.height) - target_state[1]) * self.magnification))
        target_radius = int(3 * self.magnification)
        target_colour = (0, 0, 200)
        cv2.circle(self.image, target_centre, target_radius, target_colour, cv2.FILLED)

        # Save position of robot and target for plotting later
        self.robot_pos.append(robot_centre)
        self.target_pos.append(target_centre)

        # Drawing trajectories of robot and target
        # Robot trajectory
        for i in range(1,len(self.robot_pos)-1,5):
            cv2.line(self.image, self.robot_pos[i], self.robot_pos[i+1], robot_colour, 2)

        # Target trajectory
        for i in range(1,len(self.target_pos)-1,5):
            cv2.line(self.image, self.target_pos[i], self.target_pos[i+1], target_colour, 2)

        # Show image
        cv2.imshow("Environment", self.image)
        cv2.waitKey(1)