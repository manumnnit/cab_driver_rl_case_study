# Import routines

import numpy as np
import math
import random
from itertools import permutations

# Defining hyperparameters
m = 5 # number of cities, ranges from 1 ..... m
t = 24  # number of hours, ranges from 0 .... t-1
d = 7  # number of days, ranges from 0 ... d-1
C = 5 # Per hour fuel and other costs
R = 9 # per hour revenue from a passenger


class CabDriver():

    def __init__(self):
        """initialise your state and define your action space and state space"""
        self.action_space = [(0,0)]+ list(permutations( [i for i in range(m)], 2))
        self.state_space =[[i,j,k] for i in range(m) for j in range(t) for k in range(d)]
        self.state_init =random.choice(self.state_space)

        # Start the first round
        self.reset()


    ## Encoding state (or state-action) for NN input

    def state_encod_arch1(self, state):
        """convert the state into a vector so that it can be fed to the NN.
        This method converts a given state into a vector format. Hint: The vector is of size m + t + d."""
        state_encod=[0 for i in range(m+t+d)]
        state_encod[state[0]]=1
        state_encod[m+state[1]]=1
        state_encod[m+t+state[2]]=1

        return state_encod


    # Use this function if you are using architecture-2 
    # def state_encod_arch2(self, state, action):
    #     """convert the (state-action) into a vector so that it can be fed to the NN. This method converts a given state-action pair into a vector format. Hint: The vector is of size m + t + d + m + m."""

        
    #     return state_encod


    ## Getting number of requests



    def requests(self, state):
        """Determining the number of requests basis the location. 
        Use the table specified in the MDP and complete for rest of the locations"""
        location = state[0]
        if location == 0:
            requests = np.random.poisson(2)
        if location == 1:
            requests=np.random.poisson(12)
        if location ==2:
            requests = np.random.poisson(4)
        if location == 3:
            requests = np.random.poisson(7)
        if location ==4:
            requests =np.random.poisson(8)

        if requests >15:
            requests =15

        possible_actions_index = random.sample(range(1, (m-1)*m +1), requests) + [0] # (0,0) is not considered as customer request
        actions = [self.action_space[i] for i in possible_actions_index]

        
        actions.append((0,0))

        return possible_actions_index,actions





    def reward_func(self, state, action, Time_matrix):
        """Takes in state, action and Time-matrix and returns the reward"""
        if action == [(0, 0)]:
            reward = 0 - C
        else:
            if (state[0] == action[0]):
                # if current location and pickup location is same then next state will be drop location
                pickup_time = 0  # time taken from current location to pickup location
                ride_time = Time_matrix[action[0]][action[1]][state[1]][state[2]]
            else:
                pickup_time = Time_matrix[state[0]][action[0]][state[1]][state[2]]
                t, d = self.update(state[0], action[0], state[1], state[2], Time_matrix)
                ride_time = Time_matrix[action[0]][action[1]][int(t)][d]
            total_time = pickup_time + ride_time
            reward = R * ride_time - C * total_time
        return reward








    def next_state_func(self, state, action, Time_matrix):
        """Takes state and action as input and returns next state"""
        if action == [(0, 0)]:
            next_state = state
        else:
            if (state[0] == action[0]):
                # if current location and pickup location is same then next state will be drop location
                t, d = self.update(action[0], action[1], state[1], state[2], Time_matrix)
                next_state = action[1], int(t), d
            else:
                # if current location and pickup location is not same
                # first cab driver will go from curr_location to pickup location
                t, d = self.update(state[0], action[0], state[1], state[2], Time_matrix)
                # now driver will move from pickup location to drop location
                t, d = self.update(action[0], action[1], int(t), d, Time_matrix)
                next_state = action[1], int(t), d
        return next_state

    def update(self,location1,location2,curr_time,curr_day,Time_matrix):
        time_taken = Time_matrix[location1][location2][curr_time][curr_day]
        if time_taken + curr_time >= 24:
            time = (time_taken + curr_time) % 24
            day = (curr_day + 1) % 7
        else:
            time = time_taken + curr_time
            day = curr_day
        return time, day




    def reset(self):
        return self.action_space, self.state_space, self.state_init






			

		






