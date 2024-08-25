import os
import shutil

from stable_baselines3.common.env_util import make_vec_env

import matplotlib.pyplot as plt
import numpy as np

import importlib.util


def import_module_from_path(path):
    """
    Example usage:
    module_path = '/path/to/your/module.py'
    module = import_module_from_path(module_path)
    """
    # Create a module spec from the given path
    spec = importlib.util.spec_from_file_location("module_name", path)

    # Load the module from the created spec
    module = importlib.util.module_from_spec(spec)

    # Execute the module to make its attributes accessible
    spec.loader.exec_module(module)

    # Return the imported module
    return module


def save_artifact_checkpoint(ckpt_dir):
    if not os.path.exists(ckpt_dir):
        os.makedirs(ckpt_dir)
    for item in os.listdir():
        if item.endswith(('.py', '.h', '.cc')):
            shutil.copyfile(item, f'{ckpt_dir}/ckpt_{item}')
            print(f'Saved {item}')


def exp_cc(log_dir,
           model_file,
           env_file,
           saved_model_file,
           simTime=60,
           transport_prot="TcpRl",
           error_p=0.0,
           mtu=1500,
           rl_env="TcpRlTimeBased",
           stepTime=0.5,
           bottleneck_delay="0.01ms",
           bottleneck_bandwidth="10Mbps",
           port=0,
           debug=True,
           startSim=True,
           seed=12):
    simArgs = {
        "--duration": simTime,
        "--transport_prot": transport_prot,
        # "--data": data_to_transmit,
        "--error_p": error_p,
        "--mtu": mtu,
        "--rl_env": "TcpRlTimeBased",
        "--envTimeStep": stepTime,
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

    # copy env files
    for item in os.listdir():
        if item.endswith(('.h', '.cc')):
            os.rename(item, 'bk_' + item)
    for item in os.listdir(log_dir):
        if item.endswith(('.h', '.cc')):
            shutil.copyfile(f'{log_dir}/{item}', item[5:]) # remove ckpt_ prefix
  
    model_mod = import_module_from_path(f'{log_dir}/{model_file}')
    env_mod = import_module_from_path(f'{log_dir}/{env_file}')
    
    vec_env = make_vec_env(env_mod.MyNs3Env, n_envs=1, seed=12, env_kwargs=env_kwargs)    
    model = model_mod.ppo_model(vec_env)
    model = model.load(f'{log_dir}/{saved_model_file}')
    obss= vec_env.reset()
    obs_list = []
    raw_obs_list = []
    reward_list = []
    while True:
        actions, _ = model.predict(obss)
        obss, rewards, dones, infos = vec_env.step(actions)
        obs_list.append(obss[0])
        raw_obs_list.append(infos[0]['raw_obs'])
        reward_list.append(rewards[0])
        if dones[0]:
            break

    # return env files
    for item in os.listdir():
        if item.endswith(('.h', '.cc')) and not item.startswith('bk_'):
            os.remove(item)
    for item in os.listdir():
        if item.endswith(('.h', '.cc')):
            os.rename(item, item[3:])
  
    return np.array(obs_list[:-1]), np.array(raw_obs_list[:-1]), np.array(reward_list[:-1])


def plot_series(x, y, label, xlabel, savefile):
    plt.clf()
    plt.plot(x, y, label=label, linewidth=1, color='r', marker='.')
    plt.xlabel(xlabel)
    plt.title(f'Information of {label}')
    plt.savefig(savefile)


def plot_many_series(xs, ys, labels, xlabel, title, savefile):
    plt.clf()
    for i in range(len(xs)):
        plt.plot(xs[i], ys[i], label=labels[i], linewidth=1, marker='.')
    plt.xlabel(xlabel)
    plt.title(title)
    plt.legend()
    plt.savefig(savefile)