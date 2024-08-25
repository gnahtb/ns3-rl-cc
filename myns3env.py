import torch
import numpy as np
import random

from ns3gym import ns3env

from gym import spaces


class MyNs3Env(ns3env.Ns3Env):
    def sample_simArgs(self):
        simArgs_sample = {}
        for k, v in self.simArgs_space.items():
            if isinstance(v, list):
                simArgs_sample[k] = random.choice(v)
            else:
                simArgs_sample[k] = v
        self.simArgs = simArgs_sample
        return simArgs_sample
    
    def __init__(self, simArgs_space, stepTime, *args, **kwargs):
        self.step_time = stepTime
        simArgs_space["--envTimeStep"] = stepTime
        self.simArgs_space = simArgs_space
        super().__init__(simArgs=self.sample_simArgs(), *args, **kwargs)

        # Redefine obs space
        self.observation_space = spaces.Box(0.0, 1000000000.0, (4,), np.float64)
        print("Observation space:", self.observation_space)

        # Redefine action space
        # self.action_space = spaces.Box(1.0, 1000.0, (1,), np.uint64)
        self.action_space = spaces.Discrete(1000, start=1)
        print("Action space:", self.action_space)

        # Define supporting variables
        self.total_segments_acked = 0
        self.total_bytes_tx = 0
        self.total_bytes_rx = 0
        self.segment_size = 1

    def transform_obs(self, obs):
        curr_cwnd = obs[4]
        segment_size = obs[5]
        self.segment_size = np.int64(segment_size)
        segments_acked = obs[9]
        rtt = obs[10]
        bytes_tx = obs[6]
        bytes_in_flight = obs[8]
        loss = 1.0 - (1.0 if self.total_bytes_tx == 0.0 else self.total_segments_acked * segment_size / self.total_bytes_tx)
        tput_pkt = segments_acked / self.step_time
        new_obs = np.array([curr_cwnd, tput_pkt, rtt, loss])
        return new_obs

    def transform_action(self, action):
        new_action = [np.uint64(action * self.segment_size), np.uint64(0)]
        return new_action

    def get_reward(self, obs):
        curr_cwnd = obs[4]
        segment_size = obs[5]
        segments_acked = obs[9]
        self.total_segments_acked += segments_acked
        rtt = obs[10] / 1000000
        bytes_tx = obs[6]
        self.total_bytes_tx += bytes_tx
        bytes_rx = obs[7]
        self.total_bytes_rx += bytes_rx
        bytes_in_flight = obs[8]
        # tput = segments_acked * segment_size / self.step_time
        tput = bytes_rx / self.step_time
        # loss = 1.0 - (1.0 if self.total_bytes_tx == 0.0 else self.total_bytes_rx / self.total_bytes_tx)
        rx_rate = 0.0 if self.total_bytes_tx == 0.0 else self.total_bytes_rx / self.total_bytes_tx
        return 500000 * rx_rate

    def reset(self):
        self.sample_simArgs()
        obs = super().reset()
        return self.transform_obs(obs)
  
    def step(self, action):
        obs, reward, done, info = super().step(self.transform_action(action))
        return (self.transform_obs(obs), self.get_reward(obs), done, {'info': info, 'raw_obs': obs})
