from builtins import object
import math
import numpy as np
import random
import copy

SUCCESS = 'success'
FAILURE_TOO_MANY_STEPS = 'too_many_steps'

# Custom failure states for navigation.
NAV_FAILURE_COLLISION = 'collision'
NAV_FAILURE_OUT_OF_BOUNDS = 'out_of_bounds'

def l2( xy0, xy1 ):
    ox = xy1[0]
    oy = xy1[1]
    dx = xy0[0] - xy1[0]
    dy = xy0[1] - xy1[1]
    dist = math.sqrt( (dx * dx) + (dy * dy) )
    if (xy1[0] < -0.9):
        warp_dx = xy0[0] - (1 + (xy1[0] + 1))
        dist1 = math.sqrt( (warp_dx * warp_dx) + (dy * dy) )
        if (dist1 < dist):
            ox = (1 + (xy1[0] + 1))
            dist = dist1
    elif (xy1[0] > 0.9):
        warp_dx = xy0[0] - (-1 + (xy1[0] - 1))
        dist1 = math.sqrt( (warp_dx * warp_dx) + (dy * dy) )
        if (dist1 < dist):
            ox = (-1 + (xy1[0] - 1))
            dist = dist1
    return dist, ox, oy



class Env(object):
	def __init__(self, display, field, robot_state,
                    min_dist,
                    noise_sigma,
                    in_bounds,
                    goal_bounds,
                    nsteps):

		self.init_robot_state = copy.deepcopy(robot_state)
		self.robot_state = copy.deepcopy(self.init_robot_state)

		self.field = field
		self.display = display

		self.min_dist = min_dist
		self.in_bounds = in_bounds
		self.goal_bounds = goal_bounds
		self.nsteps = nsteps
		self.cur_step = 0

		self.max_acc = 0.005
		self.max_steering = np.pi / 8

		self.forecast_steps = 5

	def reset(self):
		self.cur_step = 0
		self.robot_state = copy.deepcopy(self.init_robot_state)
		self.display.setup( self.field.x_bounds, self.field.y_bounds,
							self.in_bounds, self.goal_bounds,
							margin = self.min_dist)
		self.field.random_init() # randomize the init position of obstacles
		cx,cy,_ = self.robot_state.position
		state = [cx, cy, self.robot_state.v_x, self.robot_state.v_y]
		return np.array(state)

	def check_collision(self, cx, cy, unsafe_obstacle_ids = []):
		astlocs = self.field.obstacle_locations(self.cur_step, cx, cy, self.min_dist * 5)
		nearest_obstacle = None
		nearest_obstacle_id = -1
		nearest_obstacle_dist = np.float("inf")    
		collisions = ()
		for i,x,y in astlocs:
			self.display.obstacle_at_loc(i,x,y)
			if (i in unsafe_obstacle_ids):
				self.display.obstacle_set_color(i, 'blue')
			dist, ox, oy = l2( (cx,cy), (x,y) )
			if dist < self.min_dist:
				collisions += (i,)
				return True
		return False

	def display_start(self):
		self.display.begin_time_step(self.cur_step)

	def display_end(self):
		self.display.end_time_step(self.cur_step)

	def step(self, action, is_safe = False, unsafe_obstacle_ids = []):
		'''
		action: [dv_x, dv_y]
		'''
		self.cur_step += 1
		self.robot_state = self.robot_state.steer( action[0], action[1] )
		cx,cy,ch = self.robot_state.position
		self.display.robot_at_loc( cx, cy, ch, is_safe)
		is_collide = self.check_collision(cx, cy, unsafe_obstacle_ids)

		next_robot_state = [cx, cy, self.robot_state.v_x, self.robot_state.v_y]
		next_state = next_robot_state

		# done
		done = False
		if is_collide:
			ret = (NAV_FAILURE_COLLISION, self.cur_step)
			self.display.navigation_done(*ret)
			done = True
			reward = -500
		elif self.goal_bounds.contains( (cx,cy) ):
			ret = (SUCCESS, self.cur_step)
			self.display.navigation_done(*ret)
			done = True
			reward = 2000
		elif self.cur_step > self.nsteps:
			done = True
			reward = 0
		else:
			reward = 0
		return np.array(next_state), reward, done

	def random_action(self):
		dv_x = (2 * random.random() - 1) * self.max_acc
		dv_y = (random.random()) * self.max_acc
		return [dv_x, dv_y]


	def detect_obstacles(self, min_dist):
		cx, cy, _ = self.robot_state.position
		unsafe_obstacles = self.field.unsafe_obstacle_locations(self.cur_step, cx, cy, min_dist)
		unsafe_obstacle_ids = [ele[0] for ele in unsafe_obstacles]
		unsafe_obstacle_info = [np.array(ele[1]) for ele in unsafe_obstacles]
		return unsafe_obstacle_ids, unsafe_obstacle_info
