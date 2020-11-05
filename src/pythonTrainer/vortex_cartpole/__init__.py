from gym.envs.registration import register

register(
    id='VortexCartPole-v0',
    entry_point='vortex_cartpole.vortex_cartpole:VortexCartPole',
)