import gymnasium as gym
import ns3ai_gym_env
import os

import matplotlib.pyplot as plt

from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.logger import configure
from stable_baselines3.common.callbacks import BaseCallback


if __name__ == "__main__":
    log_dir = "./logs"
    os.makedirs(log_dir, exist_ok=True)

    target="modelTeste"
    ns3Path="/home/felipe/Downloads/ns-allinone-3.40/fork"
    ns3Settings = {
        'use-train': 'true',
        'db-file': 'db'
        # 'plot': 'true',
        # 'verbose': 'true',
        }

    # env = gym.make_vec("ns3ai_gym_env/Ns3-v0", num_envs=2 ,targetName=target, ns3Path=ns3Path, ns3Settings=ns3Settings)
    env = gym.make("ns3ai_gym_env/Ns3-v0", targetName=target, ns3Path=ns3Path, ns3Settings=ns3Settings)
    env = Monitor(env, log_dir)

    # new_logger = configure(log_dir, ["stdout", "csv"])
    model = PPO("MlpPolicy", env, verbose=1, ent_coef=0.1, learning_rate=1e-4, tensorboard_log=log_dir)
    # env = Monitor(env, "./save")
    checkpoint_callback = CheckpointCallback(save_freq=10000, save_path="./save", name_prefix="ppo_ns3ai_gym_env")

    # model.set_logger(new_logger)
    model.learn(total_timesteps=1_000_000, callback=checkpoint_callback, log_interval=10, progress_bar=True)
    model.save("./save/ppo_ns3ai_gym_env")

    # model = PPO.load("./save/ppo_ns3ai_gym_env_30000_steps.zip")

    # plot_results(["./save"], 100000, results_plotter.X_TIMESTEPS, "PPO NS3AI GYM ENV")
    # plt.show()

    # model.set_env(env)
    # vec_env = model.get_env()
    # obs = vec_env.reset()
    # while True:
    #     print(obs)
    #     action, _states = model.predict(obs)
    #     obs, rewards, dones, info = vec_env.step(action)
    #     print(rewards)
    #     print(action)
