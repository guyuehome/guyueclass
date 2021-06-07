import mujoco_py as mp
from simple_pid import PID
import numpy as np
from collections import defaultdict
from termcolor import colored
import matplotlib.pyplot as plt
import time


class UR5_Controller(object):

    def __init__(self):
        self.model = mp.load_model_from_path('ur5.xml')
        self.sim = mp.MjSim(self.model)
        self.viewer = mp.MjViewer(self.sim)
        self.create_lists()
        self.groups = defaultdict(list)
        self.groups['All'] = [i for i in range(len(self.sim.data.ctrl))]
        self.create_group('Arm', [i for i in range(6)])
        self.actuated_joint_ids = np.array([i[2] for i in self.actuators])
        self.reached_target = False
        self.current_output = np.zeros(len(self.sim.data.ctrl))
        self.image_counter = 0

    def create_lists(self):
        self.controller_list = []

        # sample_time = 0.0001
        sample_time = None

        p_scale = 3
        i_scale = 0.0
        d_scale = 0.1

        self.controller_list.append(PID(7*p_scale, 0.0*i_scale, 1.1*d_scale, setpoint=0, output_limits=(-2, 2), sample_time=sample_time)) # Shoulder Pan Joint
        self.controller_list.append(PID(10*p_scale, 0.0*i_scale, 1.0*d_scale, setpoint=0, output_limits=(-2, 2), sample_time=sample_time)) # Shoulder Lift Joint
        self.controller_list.append(PID(5*p_scale, 0.0*i_scale, 0.5*d_scale, setpoint=0, output_limits=(-2, 2), sample_time=sample_time)) # Elbow Joint
        self.controller_list.append(PID(7*p_scale, 0.0*i_scale, 0.1*d_scale, setpoint=0, output_limits=(-1, 1), sample_time=sample_time)) # Wrist 1 Joint
        self.controller_list.append(PID(2*p_scale, 0.0*i_scale, 0.1*d_scale, setpoint=0, output_limits=(-1, 1), sample_time=sample_time)) # Wrist 2 Joint
        self.controller_list.append(PID(5*p_scale, 0.0*i_scale, 0.1*d_scale, setpoint=0, output_limits=(-1, 1), sample_time=sample_time)) # Wrist 3 Joint

        self.current_target_joint_values = []
        for i in range(len(self.sim.data.ctrl)):
            self.current_target_joint_values.append(self.controller_list[i].setpoint)

        self.current_output = []
        for i in range(len(self.controller_list)):
            self.current_output.append(self.controller_list[i](0))

        self.actuators = []
        for i in range(len(self.sim.data.ctrl)):
            item = []
            item.append(i)
            item.append(self.model.actuator_id2name(i))
            item.append(self.model.actuator_trnid[i][0])
            item.append(self.model.joint_id2name(self.model.actuator_trnid[i][0]))
            item.append(self.controller_list[i])
            self.actuators.append(item)

    def create_group(self, group_name, idx_list):
        try:
            assert len(idx_list) <= len(self.sim.data.ctrl), 'Too many joints specified!'
            assert group_name not in self.groups.keys(), 'A group with name {} already exists!'.format(group_name)
            assert np.max(idx_list) <= len(self.sim.data.ctrl), 'List contains invalid actuator ID (too high)'

            self.groups[group_name] = idx_list
            print('Created new control group \'{}\'.'.format(group_name))

        except Exception as e:
            print(e)
            print('Could not create a new group.')

    def move_group_to_joint_target(self, group='All', target=None, tolerance=0.01, max_steps=10000, plot=False,
                                render=True, quiet=False):
        try:
            assert group in self.groups.keys(), 'No group with name {} exists!'.format(group)
            if target is not None:
                assert len(target) == len(self.groups[group]), 'Mismatching target dimensions for group {}!'.format(group)
            ids = self.groups[group]
            steps = 1
            result = ''
            if plot:
                self.plot_list = defaultdict(list)
            self.reached_target = False
            deltas = np.zeros(len(self.sim.data.ctrl))

            if target is not None:
                for i, v in enumerate(ids):
                    self.current_target_joint_values[v] = target[i]

            for j in range(len(self.sim.data.ctrl)):
                self.actuators[j][4].setpoint = self.current_target_joint_values[j]

            while not self.reached_target:
                current_joint_values = self.sim.data.qpos[self.actuated_joint_ids]
                for j in range(len(self.sim.data.ctrl)):
                    self.current_output[j] = self.actuators[j][4](current_joint_values[j])  # PID core code
                    self.sim.data.ctrl[j] = self.current_output[j]
                for i in ids:
                    deltas[i] = abs(self.current_target_joint_values[i] - current_joint_values[i])

                if steps % 1000 == 0 and target is not None and not quiet:
                    print('Moving group {} to joint target! Max. delta: {}, Joint: {}'. \
                          format(group, max(deltas), self.actuators[np.argmax(deltas)][3]))
                if plot and steps % 2 == 0:
                    self.fill_plot_list(group, steps)

                if max(deltas) < tolerance:
                    if target is not None and not quiet:
                        print(colored(
                            'Joint values for group {} within requested tolerance! ({} steps)'.format(group, steps),
                            color='green', attrs=['bold']))
                    result = 'success'
                    self.reached_target = True
                    # break

                if steps > max_steps:
                    if not quiet:
                        print(colored('Max number of steps reached: {}'.format(max_steps), color='red', attrs=['bold']))
                        print('Deltas: ', deltas)
                    result = 'max. steps reached: {}'.format(max_steps)
                    break

                self.sim.step()
                if render:
                    self.viewer.render()
                steps += 1

            if plot:
                self.create_joint_angle_plot(group=group, tolerance=tolerance)

            return result


        except Exception as e:
            print(e)
            print('Could not move to requested joint target.')

    def fill_plot_list(self, group, step):
        for i in self.groups[group]:
            self.plot_list[self.actuators[i][3]].append(self.sim.data.qpos[self.actuated_joint_ids][i])
        self.plot_list['Steps'].append(step)

    def create_joint_angle_plot(self, group, tolerance):
        self.image_counter += 1
        keys = list(self.plot_list.keys())
        number_subplots = len(self.plot_list) - 1
        columns = 3
        rows = (number_subplots // columns) + (number_subplots % columns)

        position = range(1, number_subplots+1)
        fig = plt.figure(1, figsize=(15,10))
        plt.subplots_adjust(hspace=0.4, left=0.05, right=0.95, top=0.95, bottom=0.05)

        for i in range(number_subplots):
            axis = fig.add_subplot(rows, columns, position[i])
            axis.plot(self.plot_list['Steps'], self.plot_list[keys[i]])
            axis.set_title(keys[i])
            axis.set_xlabel(keys[-1])
            axis.set_ylabel('Joint angle [rad]')
            axis.xaxis.set_label_coords(0.05, -0.1)
            axis.yaxis.set_label_coords(1.05, 0.5)

        filename = 'Joint_values_{}.png'.format(self.image_counter)
        plt.savefig(filename)
        print(colored('Saved trajectory to {}.'.format(filename), color='yellow', on_color='on_grey', attrs=['bold']))
        plt.clf()

    def stay(self, duration, render=True):
        starting_time = time.time()
        elapsed = 0
        while elapsed < duration:
            self.move_group_to_joint_target(max_steps=10, tolerance=0.0000001, plot=False, quiet=True, render=render)
            elapsed = (time.time() - starting_time)*1000

    def move_group_along_trajectory(self, group='All', target=None, tolerance=0.2, max_steps=10000, plot=False,
                                render=True, quiet=False):
        try:
            assert group in self.groups.keys(), 'No group with name {} exists!'.format(group)
            assert target is not None, 'Target is None!'
            assert len(target[0]) == len(self.groups[group]), 'Mismatching target dimensions for group {}!'.format(group)
            ids = self.groups[group]
            steps = 1
            result = 0
            if plot:
                self.plot_list = defaultdict(list)
            self.reached_target = False
            deltas = np.zeros(len(self.sim.data.ctrl))

            while not self.reached_target:
                current_joint_values = self.sim.data.qpos[self.actuated_joint_ids]
                for j in range(len(self.sim.data.ctrl)):
                    self.actuators[j][4].setpoint = target[steps - 1][j]
                    self.current_output[j] = self.actuators[j][4](current_joint_values[j])  # PID core code
                    # self.actuators[j][4].Kp
                    self.sim.data.ctrl[j] = self.current_output[j]
                for i in ids:
                    deltas[i] = abs(target[steps - 1][i] - current_joint_values[i])
                    result = result - (deltas[i] ** 2)
                    #print("deltas[{}] : ".format(i), deltas[i]) # for debug

                if steps % 1000 == 0 and not quiet:
                    print('Moving group {} to joint target! Max. delta: {}, Joint: {}'.format(group, max(deltas),
                                                                                              self.actuators[np.argmax(deltas)][3]))
                if plot and steps % 2 == 0:
                    self.fill_plot_list(group, steps)

                if max(deltas) > tolerance and not quiet:
                    print(colored(
                            'Warning! Tolerance over reached.', color='yellow', attrs=['bold']))

                self.sim.step()
                if render:
                    self.viewer.render()
                steps += 1
                if(steps > len(target)):
                    self.reached_target = True
                    for i, v in enumerate(ids):
                        self.current_target_joint_values[v] = current_joint_values[i]
                    print(colored('Trajectory end reached! ({} steps)'.format(steps - 1), color='green', attrs=['bold']))

            if plot:
                self.create_joint_angle_plot(group=group, tolerance=tolerance)

            return result

        except Exception as e:
            print(e)
            print('Could not move along requested joint trajectory.')






