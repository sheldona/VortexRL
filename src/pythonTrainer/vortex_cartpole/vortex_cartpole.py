import numpy as np
import torch
import cartPoleEnv as cp
import gym
from gym.spaces.box import Box

maxPoleAngle = 0.4;     # radians
maxCartPos = 2.5;       # metres
maxCartVel = 10.0;      # rads per second

class VortexCartPole(gym.Env):
    def __init__(self, **kwargs):
        headless = kwargs.get('headless', False)
        self.random_reset = kwargs.get('random_reset', False)
        self.cartpole_sim = cp.CartPoleEnvironment('../../resources/scenes/cartPole/cartPole.vxscene', headless)
        self.observation_space = Box(-np.inf, np.inf, shape=(4,), dtype=np.float32)
        self.action_space = Box(-maxCartVel, maxCartVel, shape=(1,), dtype=np.float32)
        self.seed()

    def seed(self, seed=None):
        self.np_random, seed = gym.utils.seeding.np_random(seed)
        return [seed]

    def reward(self, state):
        return 1.0

    def step(self, action):
        # Call Vortex simulation to advance the simulation 
        # and return the next state
        self.state = self.cartpole_sim.step(action)

        # Compute the reward using current state
        rw = self.reward(self.state)
        cartPos = self.state[0]
        cartVel = self.state[1]
        poleAngle = self.state[2]
        poleVel = self.state[3]

        # Determine if terminal state
        done = False
        if cartPos < -maxCartPos or cartPos > maxCartPos or poleAngle < -maxPoleAngle or poleAngle > maxPoleAngle:
            done = True
        return self.state, rw, done, {}

    def reset(self):
        self.state = self.cartpole_sim.reset()
        if self.random_reset :
            action = [ (maxCartVel) * self.np_random.random_sample() - 0.5*maxCartVel ];
            self.state = self.cartpole_sim.step(action)
        return self.state
