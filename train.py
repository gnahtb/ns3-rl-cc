import torch
import numpy as np
import os

from typing import Callable

from stable_baselines3 import A2C, PPO, TD3
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common.logger import configure
from stable_baselines3.common.callbacks import CallbackList, BaseCallback
from stable_baselines3.common.results_plotter import load_results, ts2xy, plot_results
from stable_baselines3.common.vec_env import VecMonitor


from tqdm import tqdm

from ns3gym import ns3env

from gym import spaces

import myns3env
import mymodel

from datetime import datetime

import utils


# Get the current time
current_time = datetime.now()

# Format the timestamp
formatted_time = current_time.strftime('%Y-%m-%d-%H-%M-%S')

# Log directory
log_dir = f"./sb3_log_{formatted_time}/"


class SaveOnBestTrainingRewardCallback(BaseCallback):
    """
    Callback for saving a model (the check is done every ``check_freq`` steps)
    based on the training reward (in practice, we recommend using ``EvalCallback``).

    :param check_freq:
    :param log_dir: Path to the folder where the model will be saved.
      It must contains the file created by the ``Monitor`` wrapper.
    :param verbose: Verbosity level: 0 for no output, 1 for info messages, 2 for debug messages
    """
    def __init__(self, history_len: int, check_freq: int, log_dir: str, verbose: int = 1):
        super().__init__(verbose)
        self.check_freq = check_freq
        self.log_dir = log_dir
        self.best_save_path = os.path.join(log_dir, "best_model")
        self.latest_save_path = os.path.join(log_dir, "latest_model")
        self.best_mean_reward = -np.inf
        self.history_len = history_len

    def _init_callback(self) -> None:
        # Create folder if needed
        if self.best_save_path is not None:
            os.makedirs(self.best_save_path, exist_ok=True)
        if self.latest_save_path is not None:
            os.makedirs(self.latest_save_path, exist_ok=True)

    def _on_step(self) -> bool:
        if self.n_calls % self.check_freq == 0:
            # Save latest model
            if self.verbose >= 1:
                print(f"Saving latest model to {self.latest_save_path}")
            self.model.save(self.latest_save_path)
          
            # Retrieve training reward
            x, y = ts2xy(load_results(self.log_dir), "timesteps")
            if len(x) > 0:
                # Mean training reward over the last 100 episodes
                mean_reward = np.mean(y[-self.history_len:])
                if self.verbose >= 1:
                    print(f"Num timesteps: {self.num_timesteps}")
                    print(f"Best mean reward: {self.best_mean_reward:.2f} - Last mean reward per episode: {mean_reward:.2f}")
  
                # New best model, you could save the agent here
                if mean_reward > self.best_mean_reward:
                    self.best_mean_reward = mean_reward
                    # Example for saving best model
                    if self.verbose >= 1:
                        print(f"Saving new best model to {self.best_save_path}")
                    self.model.save(self.best_save_path)

        return True


def step_schedule(initial_value: float, n_steps: int) -> Callable[[float], float]:
    """
    Step learning rate schedule.

    :param initial_value: Initial learning rate.
    :return: schedule that computes
      current learning rate depending on remaining progress
    """
    def func(progress_remaining: float) -> float:
        """
        Progress will decrease from 1 (beginning) to 0.

        :param progress_remaining:
        :return: current learning rate
        """
        lr = initial_value
        for i in range(n_steps):
            if progress_remaining < (i / n_steps):
                lr -= initial_value / n_steps
        return lr

    return func


startSim = True
port = 0
simTime = 60 # seconds
stepTime = 0.5  # seconds
seed = 12
transport_prot = "TcpRl"
data_to_transmit = 1000000
error_p = 0.0
mtu = 1500
debug = True

# error_p = [0.0, 0.02, 0.1, 0.3, 0.5]
bottleneck_delay = ["0.01ms", "1ms", "10ms", "50ms", "100ms", "300ms"]
# bottleneck_delay = "0.01ms"
bottleneck_bandwidth = ["0.5Mbps", "2Mbps", "10Mbps", "50Mbps", "100Mbps"]
# bottleneck_bandwidth = "10Mbps"

simArgs = {
    "--duration": simTime,
    "--transport_prot": transport_prot,
    # "--data": data_to_transmit,
    "--mtu": mtu,
    "--rl_env": "TcpRlTimeBased",
    "--envTimeStep": stepTime,
    "--error_p": error_p,
    "--bottleneck_delay": bottleneck_delay,
    "--bottleneck_bandwidth": bottleneck_bandwidth
}

env_kwargs = {
    'port': port,
    'stepTime': stepTime,
    'startSim': startSim,
    'simSeed': seed,
    'simArgs_space': simArgs,
    'debug': debug
}

monitored_vec_env = VecMonitor(make_vec_env(myns3env.MyNs3Env, n_envs=3, seed=12, env_kwargs=env_kwargs),
                               filename=f'{log_dir}/monitor.csv')

utils.save_artifact_checkpoint(log_dir)

model = mymodel.ppo_model(monitored_vec_env, step_schedule(3e-4, 10))
model.set_logger(configure(log_dir, ["stdout", "csv", "tensorboard"]))

save_callback = SaveOnBestTrainingRewardCallback(history_len=1000,
                                                 check_freq=1000,
                                                 log_dir=log_dir)

model.learn(total_timesteps=500000,
            callback=CallbackList([save_callback]),
            progress_bar=True)

model.save(f"{log_dir}/lastrun_{formatted_time}/")

monitored_vec_env.close()