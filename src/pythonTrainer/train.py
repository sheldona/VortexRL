import torch
import gym
import numpy as np
from spinup import ppo_pytorch as ppo

def env_fn():
    import vortex_cartpole
    # We can pass a dictionary of arguments to the environment using kwargs
    #  headless : True or False, selects whether or not to use graphics rendering
    #  random_reset : if True, a random state is induced when the environment is reset
    #
    kwargs = { "headless" : False, "random_reset" : True }
    env = gym.make('VortexCartPole-v0', **kwargs)
    return env;

# Test training am agent using pytorch PPO
ac_kwargs = dict(hidden_sizes=[32,32], activation=torch.nn.ReLU)
ppo(env_fn, steps_per_epoch=1000, epochs=50, gamma=0.99, pi_lr=1e-3, vf_lr=1e-3, ac_kwargs=ac_kwargs)

