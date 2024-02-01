import gymnasium as gym
import ns3ai_gym_env
import os

import numpy as np
import matplotlib.pyplot as plt

from stable_baselines3 import PPO
from stable_baselines3.common import results_plotter
from stable_baselines3.common.results_plotter import load_results, ts2xy, plot_results
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.monitor import Monitor


class SaveOnBestTrainingRewardCallback(BaseCallback):
    """
    Callback for saving a model (the check is done every ``check_freq`` steps)
    based on the training reward (in practice, we recommend using ``EvalCallback``).

    :param check_freq:
    :param log_dir: Path to the folder where the model will be saved.
      It must contains the file created by the ``Monitor`` wrapper.
    :param verbose: Verbosity level: 0 for no output, 1 for info messages, 2 for debug messages
    """
    def __init__(self, check_freq: int, log_dir: str, verbose: int = 1):
        super().__init__(verbose)
        self.check_freq = check_freq
        self.log_dir = log_dir
        self.save_path = os.path.join(log_dir, "best_model")
        self.best_mean_reward = -np.inf

    def _init_callback(self) -> None:
        # Create folder if needed
        if self.save_path is not None:
            os.makedirs(self.save_path, exist_ok=True)

    def _on_step(self) -> bool:
        if self.n_calls % self.check_freq == 0:

          # Retrieve training reward
          x, y = ts2xy(load_results(self.log_dir), "timesteps")
          if len(x) > 0:
              # Mean training reward over the last 100 episodes
              mean_reward = np.mean(y[-100:])
              if self.verbose >= 1:
                print(f"Num timesteps: {self.num_timesteps}")
                print(f"Best mean reward: {self.best_mean_reward:.2f} - Last mean reward per episode: {mean_reward:.2f}")

              # New best model, you could save the agent here
              if mean_reward > self.best_mean_reward:
                  self.best_mean_reward = mean_reward
                  # Example for saving best model
                  if self.verbose >= 1:
                    print(f"Saving new best model to {self.save_path}")
                  self.model.save(self.save_path)

        return True

# Create log dir
log_dir = "tmp/"
os.makedirs(log_dir, exist_ok=True)

target="modelTeste"
ns3Path="/home/felipe/Downloads/ns-allinone-3.40/fork"
ns3Settings = {
    'use-train': 'true',
    # 'plot': 'true',
    # 'verbose': 'true',
    }

env = gym.make("ns3ai_gym_env/Ns3-v0", targetName=target, ns3Path=ns3Path, ns3Settings=ns3Settings)
# model = PPO("MlpPolicy", env, verbose=1, ent_coef=0.1)
# env = Monitor(env, "./save")
# callback = SaveOnBestTrainingRewardCallback(check_freq=100, log_dir="./save")
# model.learn(total_timesteps=100_000, progress_bar=True)
# model.save("./save/ppo_ns3ai_gym_env")

model = PPO.load("./save/ppo_ns3ai_gym_env")

# plot_results(["./save"], 100000, results_plotter.X_TIMESTEPS, "PPO NS3AI GYM ENV")
# plt.show()

model.set_env(env)
vec_env = model.get_env()
obs = vec_env.reset()
while True:
    print(obs)
    action, _states = model.predict(obs)
    obs, rewards, dones, info = vec_env.step(action)
    print(rewards)
    print(action)
