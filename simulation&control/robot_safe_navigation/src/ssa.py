import numpy as np
import math
import cvxopt
import sys
import collections

class SafeSetAlgorithm():
    def __init__(self, max_speed, dmin = 0.1, k = 1, max_acc = 0.04):
        """
        Args:
            dmin: dmin for phi
            k: k for d_dot in phi
        """
        self.dmin = dmin
        self.k = k
        self.max_speed = max_speed
        self.max_acc = max_acc


    def get_safe_control(self, robot_state, obs_states, f, g, u0):
        """
        Args:
            robot_state <x, y, vx, vy>
            obs_state: np array close obstacle state <x, y, vx, vy, ax, ay>
        """
        u0 = np.array(u0).reshape((2,1))

        L_gs = []
        L_fs = []
        reference_control_laws = []
        is_safe = True
        phis = []
        danger_indexs = []

        for i, obs_state in enumerate(obs_states):
            d = np.array(robot_state - obs_state[:4])
            d_pos = d[:2] # relative pos
            d_vel = d[2:] # relative vel 
            d_abs = np.linalg.norm(d_pos)
            d_dot = self.k * (d_pos @ d_vel.T) / d_abs
            phi = np.power(self.dmin, 2) - np.power(d_abs, 2) - d_dot
            
            # calculate Lie derivative
            # p d to p robot state and p obstacle state
            p_pos_p_robot_state = np.hstack([np.eye(2), np.zeros((2,2))]) # shape (2, 4)
            p_d_pos_p_pos = np.array([d_pos[0], d_pos[1]]).reshape((1,2)) / d_abs # shape (1, 2)
            p_d_pos_p_robot_state = p_d_pos_p_pos @ p_pos_p_robot_state # shape (1, 4)
            p_d_p_robot_state = -2 * d_abs * p_d_pos_p_robot_state

            # p d_dot to p robot state and p obstacle state
            p_vel_p_robot_state = np.hstack([np.zeros((2,2)), np.eye(2)]) # shape (2, 4)
            term1 = (d_pos.reshape((1,2)) / d_abs) @ p_vel_p_robot_state

            term2 = (p_pos_p_robot_state * d_abs - d_pos.reshape((2,1)) @ p_d_pos_p_robot_state) / np.power(d_abs, 2)
            term2 = d_vel.reshape((1,2)) @ term2
            p_d_dot_p_robot_state = term1 + term2  # shape (1, 4)
            p_phi_p_robot_state = p_d_p_robot_state - self.k * p_d_dot_p_robot_state # shape (1, 4)

            L_f = p_phi_p_robot_state @ (f @ robot_state.reshape((-1,1))) # shape (1, 1)
            L_g = p_phi_p_robot_state @ g # shape (1, 2) g contains x information
            L_fs.append(L_f) 

            if (phi > 0):
                L_gs.append(L_g)                                              
                reference_control_laws.append( -0.5*phi - L_f) 
                is_safe = False
                danger_indexs.append(i)


        if (not is_safe):
            # Solve safe optimization problem
            # min_x (1/2 * x^T * Q * x) + (p^T * x)   s.t. Ax <= b
            u0 = u0.reshape(-1,1)
            u, reference_control_laws = self.solve_qp(robot_state, u0, L_gs, reference_control_laws)
            return u, True
        u0 = u0.reshape(1,2)
        u = u0 
        return u[0], False

    def solve_qp(self, robot_state, u0, L_gs, reference_control_laws):
        q = np.eye(2)
        Q = cvxopt.matrix(q)
        u_prime = -u0
        u_prime = q @ u_prime
        p = cvxopt.matrix(u_prime) #-u0
        G = cvxopt.matrix(np.vstack([np.eye(2), -np.eye(2), np.array([[1,0],[-1,0]]), np.array([[0,1],[0,-1]])]))
        S_saturated = cvxopt.matrix(np.array([self.max_acc, self.max_acc, self.max_acc, self.max_acc, \
                                    self.max_speed-robot_state[2], self.max_speed+robot_state[2], \
                                    self.max_speed-robot_state[3], self.max_speed+robot_state[3]]).reshape(-1, 1))
        L_gs = np.array(L_gs).reshape(-1, 2)
        reference_control_laws = np.array(reference_control_laws).reshape(-1,1)
        A = cvxopt.matrix([[cvxopt.matrix(L_gs), G]])
        cvxopt.solvers.options['show_progress'] = False
        cvxopt.solvers.options['maxiters'] = 600
        while True:
            try:
                b = cvxopt.matrix([[cvxopt.matrix(reference_control_laws), S_saturated]])
                sol = cvxopt.solvers.qp(Q, p, A, b)
                u = sol["x"]
                break
            except ValueError:
                # no solution, relax the constraint                  
                for i in range(len(reference_control_laws)):
                    reference_control_laws[i][0] += 0.01
        u = np.array([u[0], u[1]])
        return u, reference_control_laws