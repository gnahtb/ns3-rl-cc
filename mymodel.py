from typing import Callable

import torch
from stable_baselines3 import A2C, PPO, TD3


def ppo_model(env,
              lr: float | Callable[[float], float]=0.0003,
              batch_size=64):
    policy_kwargs = dict(activation_fn=torch.nn.GELU,
                         net_arch=dict(pi=[128, 128], vf=[128, 128]))
  
    model = PPO("MlpPolicy",
            env,
            learning_rate=lr,
            batch_size=batch_size,
            verbose=1,
            policy_kwargs=policy_kwargs)
    return model