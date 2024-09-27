import numpy as np
from stable_baselines3.common.results_plotter import load_results, ts2xy
from stable_baselines3.common.callbacks import BaseCallback

from visdom_utils import VisdomLinePlotter


class VisdomCallback(BaseCallback):
    """
    Callback for visualizing the reward and loss (the check is done every ``check_freq`` steps)

    :param check_freq: (int)
      It must contains the file created by the ``Monitor`` wrapper.
    :param verbose: (int)
    """
    def __init__(self, name:str, check_freq: int, log_dir:str, verbose=1):
        super(VisdomCallback, self).__init__(verbose)
        self.check_freq = check_freq
        self.log_dir = log_dir
        self.plotter = VisdomLinePlotter(env_name=name)

    def _on_step(self) -> bool:
        if self.n_calls % self.check_freq == 0:
            x, y = ts2xy(load_results(self.log_dir), 'timesteps') # Retrieve training reward

            if len(x) > 0:
                print("sending values to visdom ...")
                mean_reward = np.mean(y[-10:]) # Mean training reward over the last 10 episodes
                self.plotter.plot('epochs', 'reward', 'reward', 'reward visual', self.n_calls, mean_reward)

        return True

