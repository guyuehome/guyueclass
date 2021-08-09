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
        return (length, )
    return (length, shape) if np.isscalar(shape) else (length, *shape)


class cnn_model(nn.Module):
    def __init__(self, num_inputs, num_out, activation=nn.ReLU):
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
    return scipy.signal.lfilter([1], [1, float(-discount)], x[::-1],
                                axis=0)[::-1]


class Actor(nn.Module):
    def _distribution(self, obs):
        raise NotImplementedError

    def _log_prob_from_distribution(self, pi, act):
        raise NotImplementedError

    def forward(self, obs, act=None):
        # Produce action distributions for given observations, and
        # optionally compute the log likelihood of given actions under
        # those distributions.
        pi = self._distribution(obs)
        logp_a = None
        if act is not None:
            logp_a = self._log_prob_from_distribution(pi, act)
        return pi, logp_a


class CNNGaussianActor(Actor):
    def __init__(self, obs_dim, act_dim, activation, pretrain=None):
        super().__init__()
        log_std = -0.5 * np.ones(act_dim, dtype=np.float32)
        self.log_std = torch.nn.Parameter(torch.as_tensor(log_std))
        self.mu_net = cnn_model(obs_dim, act_dim, activation=activation)
        if pretrain != None:
            print('\n\nLoading pretrained from %s.\n\n' % pretrain)
        print(self.mu_net)

    def _distribution(self, obs):
        mu = self.mu_net(obs)
        std = torch.exp(self.log_std)
        return Normal(mu, std)

    def _log_prob_from_distribution(self, pi, act):
        print(f'log_prob(act):{pi.log_prob(act)}')
        return pi.log_prob(act).sum(axis=-1)

    # def forward(self, obs, act=None):
    #     mu = self.mu_net(obs)
    #     std = torch.exp(self.log_std)
    #     pi = Normal(mu, std)
    #     logp_a = None
    #     if act is not None:
    #         logp_a = pi.log_prob(act).sum(axis=-1)
    #     return pi, logp_a


class CNNCritic(nn.Module):
    def __init__(self, obs_dim, activation):
        super().__init__()
        # cnn_net([obs_dim] + list(hidden_sizes) + [1], activation)
        self.v_net = cnn_model(obs_dim, 1, activation=activation)
        print(self.v_net)

    def forward(self, obs):
        v = self.v_net(obs)
        return torch.squeeze(v, -1)  # Critical to ensure v has right shape.


class CNNActorCritic(nn.Module):
    def __init__(self,
                 observation_space,
                 action_space,
                 hidden_sizes=(64, 64),
                 activation=nn.Tanh):
        super().__init__()

        obs_dim = observation_space.shape[0]
        act_dim = action_space.shape[0]

        if isinstance(action_space, Box):
            self.pi = CNNGaussianActor(obs_dim, act_dim, activation)
        elif isinstance(action_space, Discrete):
            #self.pi = MLPCategoricalActor(obs_dim, action_space.n, hidden_sizes, activation)
            raise NotImplementedError

        # build value functionp
        self.v = CNNCritic(obs_dim, activation)

    def step(self, obs):
        with torch.no_grad():
            pi = self.pi._distribution(obs)
            a = pi.sample()
            print(f'a={a}')

            logp_a = self.pi._log_prob_from_distribution(pi, a)
            print(f'logp_a={logp_a}')
            v = self.v(obs)
        return a.numpy(), v.numpy(), logp_a.numpy()

    def act(self, obs):
        return self.step(obs)[0]
