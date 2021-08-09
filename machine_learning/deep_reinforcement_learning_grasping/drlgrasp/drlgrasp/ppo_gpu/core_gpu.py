import numpy as np
import scipy.signal
from gym.spaces import Box, Discrete

import torch
import torch.nn as nn
from torch.distributions.normal import Normal
from torch.distributions.categorical import Categorical
import torch.nn.functional as F


def combined_shape(length, shape=None):
    if shape is None:
        return (length,)
    return (length, shape) if np.isscalar(shape) else (length, *shape)


class cnn_model(nn.Module):
    def __init__(self, num_inputs, num_out):
        super(cnn_model, self).__init__()
        self.conv1 = nn.Conv2d(num_inputs, 32, 3, stride=2, padding=1)
        self.conv2 = nn.Conv2d(32, 32, 3, stride=2, padding=1)
        self.conv3 = nn.Conv2d(32, 32, 3, stride=2, padding=1)
        self.conv4 = nn.Conv2d(32, 32, 3, stride=2, padding=1)

        # The output image size is calculated through
        # (I - K + 2*P) / S + 1
        # where: I : image size, the initial image size is 84x84, so I == 84 here.
        #        K : kernel size, here is 3
        #        P : padding size, here is 1
        #        S : stride, here is 2
        # So 84x84 image will become 6x6 through the ConvNet above. And 32 is the filters number.
        self.fc1 = nn.Linear(32 * 6 * 6, 512)
        self.fc_out = nn.Linear(512, num_out)
        self._initialize_weights()

    def _initialize_weights(self):
        for module in self.modules():
            if isinstance(module, nn.Conv2d) or isinstance(module, nn.Linear):
                nn.init.xavier_uniform_(module.weight)
                # nn.init.kaiming_uniform_(module.weight)
                nn.init.constant_(module.bias, 0)
            elif isinstance(module, nn.LSTMCell):
                nn.init.constant_(module.bias_ih, 0)
                nn.init.constant_(module.bias_hh, 0)

    def forward(self, x):
        x = F.relu(self.conv1(x))
        x = F.relu(self.conv2(x))
        x = F.relu(self.conv3(x))
        x = F.relu(self.conv4(x))
        x = x.view(x.size(0), -1)
        x = F.relu(self.fc1(x))
        out = self.fc_out(x)
        return out.squeeze()


def count_vars(module):
    return sum([np.prod(p.shape) for p in module.parameters()])


def discount_cumsum(x, discount):
    """
    magic from rllab for computing discounted cumulative sums of vectors.

    input:
        vector x,
        [x0,
         x1,
         x2]

    output:
        [x0 + discount * x1 + discount^2 * x2,
         x1 + discount * x2,
         x2]
    """
    return scipy.signal.lfilter([1], [1, float(-discount)], x[::-1],
                                axis=0)[::-1]


class CNNGaussianActor(nn.Module):
    def __init__(self, obs_dim, act_dim):
        super().__init__()
        log_std = -0.5 * np.ones(act_dim, dtype=np.float32)
        self.log_std = torch.nn.Parameter(torch.as_tensor(log_std))
        self.mu_net = cnn_model(obs_dim, act_dim)

    def forward(self, obs, act=None):
        mu = self.mu_net(obs)
        std = torch.exp(self.log_std)
        pi = Normal(mu, std)
        logp_a = None
        if act is not None:
            logp_a = pi.log_prob(act).sum(axis=-1)
        return pi, logp_a


class CNNCritic(nn.Module):
    def __init__(self, obs_dim):
        super().__init__()

        self.v_net = cnn_model(obs_dim, 1)
        print(self.v_net)

    def forward(self, obs):
        v = self.v_net(obs)
        return torch.squeeze(v, -1)  # Critical to ensure v has right shape.
