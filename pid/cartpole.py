import gymnasium as gym
import numpy as np

from controllers import PIDController

env = gym.make('InvertedPendulum-v4', render_mode='human')
observation, _ = env.reset(seed=17)


controller = PIDController()

for _ in range(2000):
    x, theta, v, w = observation
    control = controller.compute(theta, 0)
    observation, *_ = env.step(np.array([-control]))

env.close()
