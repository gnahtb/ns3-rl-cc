import utils
import csv
import numpy as np


LOG_DIR = "sb3_log_2024-08-04-17-23-29/"
# MODEL_FILE = f"{LOG_DIR}/best_model.zip"
MODEL_FILE = "best_model.zip"


def plot_reward(log_dir):
    # open the file in read mode
    filename = open(f'{log_dir}/progress.csv', 'r')
     
    # creating dictreader object
    file = csv.DictReader(filename)
     
    # creating empty lists
    r = []
    t = []
     
    # iterating over each row and append
    # values to empty list
    for col in file:
        if col['rollout/ep_rew_mean']:
            r.append(float(col['rollout/ep_rew_mean']))
            t.append(int(col['time/total_timesteps']))

    utils.plot_series(t, r, "reward", "timesteps", f"{log_dir}/reward.png")


def plot_obs(obs_arr, x_idx, y_idxs, x_label, y_labels, savedir):
    for i in range(len(y_idxs)):
        utils.plot_series(obs_arr[:,x_idx], obs_arr[:,y_idxs[i]], y_labels[i], x_label, f"{savedir}/{y_labels[i]}")


def calculate_total_acks(acks):
    res = [0]
    for i in range(acks.size):
        res.append(res[-1] + acks[i])
    return np.array(res[1:])


plot_reward(LOG_DIR)

# print(evaluate_policy(model, vec_env, return_episode_rewards=True))

obs, raw_obs, reward = utils.exp_cc(log_dir=LOG_DIR,
                                    model_file="ckpt_mymodel.py",
                                    env_file="ckpt_myns3env.py",
                                    saved_model_file=MODEL_FILE,
                                    transport_prot='TcpRl')
obs_1, raw_obs_1, reward_1 = utils.exp_cc(log_dir=LOG_DIR,
                                    model_file="ckpt_mymodel.py",
                                    env_file="ckpt_myns3env.py",
                                    saved_model_file="best_model-tput",
                                    transport_prot='TcpRl')
obs_cubic, raw_obs_cubic, reward_cubic = utils.exp_cc(log_dir=LOG_DIR,
                                                      model_file="ckpt_mymodel.py",
                                                      env_file="ckpt_myns3env.py",
                                                      saved_model_file=MODEL_FILE,
                                                      transport_prot='TcpCubic')

utils.plot_many_series(xs=[raw_obs[:,1], raw_obs_cubic[:,1], raw_obs_1[:,1]],
                       ys=[raw_obs[:,9], raw_obs_cubic[:,9], raw_obs_1[:,9]],
                       labels=["TcpRl-2", "TcpCubic", "TcpRl-1"],
                       xlabel="Time (us)",
                       title="Compare ACKs",
                       savefile=f"{LOG_DIR}/ACKs_cmp_new.png")
utils.plot_many_series(xs=[raw_obs[:,1], raw_obs_cubic[:,1], raw_obs_1[:,1]],
                       ys=[raw_obs[:,4], raw_obs_cubic[:,4], raw_obs_1[:,4]],
                       labels=["TcpRl-2", "TcpCubic", "TcpRl-1"],
                       xlabel="Time (us)",
                       title="Compare CWNDs",
                       savefile=f"{LOG_DIR}/CWNDs_cmp_new.png")
utils.plot_many_series(xs=[raw_obs[:,1], raw_obs_cubic[:,1], raw_obs_1[:,1]],
                       ys=[obs[:,3], obs_cubic[:,3], np.absolute(obs_1[:,3])],
                       labels=["TcpRl-2", "TcpCubic", "TcpRl-1"],
                       xlabel="Time (us)",
                       title="Compare LOSS",
                       savefile=f"{LOG_DIR}/LOSS_cmp_new.png")
utils.plot_many_series(xs=[raw_obs[:,1], raw_obs_cubic[:,1], raw_obs_1[:,1]],
                       ys=[obs[:,2], obs_cubic[:,2], obs_1[:,2]],
                       labels=["TcpRl-2", "TcpCubic", "TcpRl-1"],
                       xlabel="Time (us)",
                       title="Compare RTT",
                       savefile=f"{LOG_DIR}/RTT_cmp_new.png")
utils.plot_many_series(xs=[raw_obs[:,1], raw_obs_cubic[:,1], raw_obs_1[:,1]],
                       ys=[calculate_total_acks(raw_obs[:,9]), calculate_total_acks(raw_obs_cubic[:,9]), calculate_total_acks(raw_obs_1[:,9])],
                       labels=["TcpRl-2", "TcpCubic", "TcpRl-1"],
                       xlabel="Time (us)",
                       title="Compare total ACKs",
                       savefile=f"{LOG_DIR}/total_ACKs_cmp_new.png")
utils.plot_many_series(xs=[raw_obs[:,1], raw_obs_cubic[:,1], raw_obs_1[:,1]],
                       ys=[calculate_total_acks(raw_obs[:,7]), calculate_total_acks(raw_obs_cubic[:,7]), calculate_total_acks(raw_obs_1[:,7])],
                       labels=["TcpRl-2", "TcpCubic", "TcpRl-1"],
                       xlabel="Time (us)",
                       title="Compare total RX bytes",
                       savefile=f"{LOG_DIR}/total_rx_cmp_new.png")
utils.plot_many_series(xs=[raw_obs[:,1], raw_obs_cubic[:,1], raw_obs_1[:,1]],
                       ys=[reward, reward_cubic, reward_1],
                       labels=["TcpRl-2", "TcpCubic", "TcpRl-1"],
                       xlabel="Time (us)",
                       title="Compare step reward",
                       savefile=f"{LOG_DIR}/step_reward_cmp_new.png")

# utils.plot_many_series(xs=[raw_obs[:,1], raw_obs_cubic[:,1]],
#                        ys=[raw_obs[:,9], raw_obs_cubic[:,9]],
#                        labels=["TcpRl", "TcpCubic"],
#                        xlabel="Time (us)",
#                        title="Compare ACKs",
#                        savefile=f"{LOG_DIR}/ACKs_cmp_new.png")
# utils.plot_many_series(xs=[raw_obs[:,1], raw_obs_cubic[:,1]],
#                        ys=[raw_obs[:,4], raw_obs_cubic[:,4]],
#                        labels=["TcpRl", "TcpCubic"],
#                        xlabel="Time (us)",
#                        title="Compare CWNDs",
#                        savefile=f"{LOG_DIR}/CWNDs_cmp_new.png")
# utils.plot_many_series(xs=[raw_obs[:,1], raw_obs_cubic[:,1]],
#                        ys=[obs[:,3], obs_cubic[:,3]],
#                        labels=["TcpRl", "TcpCubic"],
#                        xlabel="Time (us)",
#                        title="Compare LOSS",
#                        savefile=f"{LOG_DIR}/LOSS_cmp_new.png")
# utils.plot_many_series(xs=[raw_obs[:,1], raw_obs_cubic[:,1]],
#                        ys=[obs[:,2], obs_cubic[:,2]],
#                        labels=["TcpRl", "TcpCubic"],
#                        xlabel="Time (us)",
#                        title="Compare RTT",
#                        savefile=f"{LOG_DIR}/RTT_cmp_new.png")
# utils.plot_many_series(xs=[raw_obs[:,1], raw_obs_cubic[:,1]],
#                        ys=[calculate_total_acks(obs[:,2]), calculate_total_acks(obs_cubic[:,2])],
#                        labels=["TcpRl", "TcpCubic"],
#                        xlabel="Time (us)",
#                        title="Compare total ACKs",
#                        savefile=f"{LOG_DIR}/total_ACKs_cmp_new.png")
# utils.plot_many_series(xs=[raw_obs[:,1], raw_obs_cubic[:,1]],
#                        ys=[calculate_total_acks(raw_obs[:,7]), calculate_total_acks(raw_obs_cubic[:,7])],
#                        labels=["TcpRl", "TcpCubic"],
#                        xlabel="Time (us)",
#                        title="Compare total RX bytes",
#                        savefile=f"{LOG_DIR}/total_rx_cmp_new.png")
# utils.plot_many_series(xs=[raw_obs[:,1], raw_obs_cubic[:,1]],
#                        ys=[reward, reward_cubic],
#                        labels=["TcpRl", "TcpCubic"],
#                        xlabel="Time (us)",
#                        title="Compare step reward",
#                        savefile=f"{LOG_DIR}/step_reward_cmp_new.png")