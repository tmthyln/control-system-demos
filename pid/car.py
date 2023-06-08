import numpy as np
from gymnasium.envs.classic_control.continuous_mountain_car import Continuous_MountainCarEnv

from controllers import PIDController

env = Continuous_MountainCarEnv(render_mode="human")
env.max_speed = 0.1
env.power = 0.0025
env.min_position = -2
env.max_position = 3
env.screen_width = 1200
observation, _ = env.reset(seed=17)


controller = PIDController()

for _ in range(2000):
    position = observation[0]
    
    control = controller.compute(position, env.goal_position)
    observation, *_ = env.step(np.array([control]))

env.close()
