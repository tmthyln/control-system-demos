from gymnasium.envs.classic_control.pendulum import PendulumEnv
from math import sqrt
import numpy as np

from controllers import PIDController

env = PendulumEnv(render_mode="human", g=9.82)
env.max_torque = 5
observation, _ = env.reset(seed=17)


controller = PIDController(p=50, d=-0.75)

for _ in range(2000):
    x, y, w = observation
    dist = (-1 if y < 0 else 1) * sqrt((x-1)**2 + y**2)
    
    control = controller.compute(dist, 0)
    observation, *_ = env.step(np.array([control]))
    
env.close()
