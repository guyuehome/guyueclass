# drlgrasp

Train kuka robot reach a point with deep rl in pybullet.

* **NOTE: The main brach is trained with spinup, and there are some issues with gpu and multi core CPUs at the same time, so this brach will be deprecated in the future. The rllib branch is trained with ray/rllib, and this branch will be mainly used in the future.**
* **The main branch will not update for a while, the rllib brach is the newest**


The train process with mlp|The evaluate process with mlp|train plot
:---------------:|:------------------:|:-------------------------:
![](https://github.com/borninfreedom/kuka-reach-drl/blob/main/pictures/reach_train_with_mlp.gif)|![](https://github.com/borninfreedom/kuka-reach-drl/blob/main/pictures/reach_result_with_mlp.gif)|![](https://github.com/borninfreedom/kuka-reach-drl/blob/main/pictures/reach_mlp_train_process.png)

The train process with cnn|The evaluate process with cnn|train plot
:---------------:|:------------------:|:-------------------------:
![](https://github.com/borninfreedom/kuka-reach-drl/blob/main/pictures/reach_train_with_cnn.gif)|![](https://github.com/borninfreedom/kuka-reach-drl/blob/main/pictures/reach_result_with_cnn.gif)|


# Installation guide (Now only support linux and macos)
**I strongly recommend using Conda to install the env, because you will possible encounter the mpi4py error with pip.**

The spinningup rl library is the necessary lib.
first, you should install miniconda or anaconda.
second, install some dev dependencies.

```bash
sudo apt-get update && sudo apt-get install libopenmpi-dev
sudo apt install libgl1-mesa-glx
```
third, create a conda virtual environment
```bash
conda create -n spinningup python=3.6   #python 3.6 is recommended
```


```bash
#activate the env
conda activate spinningup
```

then, install spiningup,is contains almost dependencies
```bash
# clone my version, I made some changes.
git clone https://github.com/borninfreedom/spinningup.git
cd spinningup
pip install -e .
```

last, install torch and torchvision.

if you have a gpu, please run this (conda will install a correct version of cudatoolkit and cudnn in the virtual env, so don't care which version you have installed in your machine.)
```bash
# CUDA 10.1
conda install pytorch==1.4.0 torchvision==0.5.0 cudatoolkit=10.1 -c pytorch
```

if you only have a cpu, please run this,
```bash
# CPU Only
conda install pytorch==1.4.0 torchvision==0.5.0 cpuonly -c pytorch
```



## view the train results through plot
```bash
python -m spinup.run plot ./logs
```
More detailed information please visit [plotting results](https://spinningup.openai.com/en/latest/user/plotting.html)


# Resources about deep rl reach and grasp.
## Articles
* [spinningup docs](https://spinningup.openai.com/en/latest/user/installation.html)
* [Proximal Policy Optimization Tutorial (Part 1/2: Actor-Critic Method)](https://towardsdatascience.com/proximal-policy-optimization-tutorial-part-1-actor-critic-method-d53f9afffbf6)(do not carefully read now.)
* [some ray/rllib and other rl problems' blogs](https://www.datahubbs.com/)
* [Action Masking with RLlib](https://towardsdatascience.com/action-masking-with-rllib-5e4bec5e7505)
* [This AI designs beautiful Forest Landscapes for Games!](https://medium.com/deepgamingai/this-ai-designs-beautiful-forest-landscapes-for-games-8675e053636e)
* [Chintan Trivedi's homepage](https://medium.com/@chintan.t93), he writes many blogs about AI and games. It's very recommended.
* [Proximal Policy Optimization Tutorial (Part 1/2: Actor-Critic Method)](https://twitter.com/ericwen5986/status/1374361315100172289)
* [Proximal Policy Optimization Tutorial (Part 2/2: GAE and PPO loss)](https://twitter.com/ericwen5986/status/1374361470859767809)
* [Antonin Raffin](https://araffin.github.io/), he is the member of stable baseline3 project.
* [spinningup using in pybullet envs](https://www.etedal.net/2020/04/pybullet-panda_3.html), this is a blog about how to use spinningup to pybullet envs and use the image as the observation.
* [Understanding LSTM Networks](https://colah.github.io/posts/2015-08-Understanding-LSTMs/), this is a good blog introducing lstm.

## Source codes
* [robotics-rl-srl](https://github.com/araffin/robotics-rl-srl), S-RL Toolbox: Reinforcement Learning (RL) and State Representation Learning (SRL) for Robotics. In this project, there are CNN policy and instructions how to connect a real robot using deep rl.
* [zenetio/DeepRL-Robotic](https://github.com/zenetio/DeepRL-Robotic), a deep rl project using gazebo.
* [robotology-playground/pybullet-robot-envs](https://github.com/robotology-playground/pybullet-robot-envs), a deep rl project using pybullet, it is built by a company, there are a lot can study from their codes. But their envs do not introduce images.
* [mahyaret/kuka_rl](https://github.com/mahyaret/kuka_rl), a tutorial tells you how to implement DQN and ppo algorithms to kuka robot grasping.
* [AutodeskRoboticsLab/RLRoboticAssembly](https://github.com/AutodeskRoboticsLab/RLRoboticAssembly), a deep rl robot assembly project build by autodesk, it uses rllib and ray.
* [MorvanZhou/train-robot-arm-from-scratch](https://github.com/MorvanZhou/train-robot-arm-from-scratch), a deep rl robot project build by Morvan.
* **[BarisYazici/deep-rl-grasping](https://github.com/BarisYazici/deep-rl-grasping), a deep rl robot grasping project built by a student in Technical University of Munich. He also released his degree's paper, we can learn a lot from his paper.**
* [mahyaret/gym-panda](https://github.com/mahyaret/gym-panda), this is a pybullet panda environment for deep rl. In the codes, author makes the image as the observation.
* [gaoxiaos/Supermariobros-PPO-pytorch](https://github.com/gaoxiaos/Supermariobros-PPO-pytorch), a tutorial about how to implement deep rl to super mario game, the algorithms are modified from spiningup, and the observation is image. So the code is very suitable for image based deep rl.

* [ShangtongZhang/reinforcement-learning-an-introduction](https://github.com/ShangtongZhang/reinforcement-learning-an-introduction), this is the python version code of the book reinforcement learning an introduction second edition, the full book and other resources can be found here [Reinforcement Learning: An Introduction](http://incompleteideas.net/book/the-book-2nd.html).



