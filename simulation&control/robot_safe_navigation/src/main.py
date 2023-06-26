from __future__ import print_function
from __future__ import absolute_import

# python modules
import argparse
import numpy as np
import time
# project files
import dynamic_obstacle
import bounds
import robot 
import simu_env
import runner
import param
from turtle_display import TurtleRunnerDisplay
from ssa import SafeSetAlgorithm


def display_for_name( dname ):
    # choose none display or visual display
    if dname == 'turtle':
        return TurtleRunnerDisplay(800,800)
    else:
        return runner.BaseRunnerDisplay()

def parser():
    prsr = argparse.ArgumentParser()
    prsr.add_argument( '--display',
                       choices=('turtle','text','none'),
                       default='turtle' )
    prsr.add_argument( '--ssa',dest='enable_ssa', action='store_true')
    prsr.add_argument( '--no-ssa',dest='enable_ssa', action='store_false')
    return prsr

def run_kwargs( params ):
    in_bounds = bounds.BoundsRectangle( **params['in_bounds'] )
    goal_bounds = bounds.BoundsRectangle( **params['goal_bounds'] )
    min_dist = params['min_dist']
    ret = { 'field': dynamic_obstacle.ObstacleField(),
            'robot_state': robot.DoubleIntegratorRobot( **( params['initial_robot_state'] ) ),
            'in_bounds': in_bounds,
            'goal_bounds': goal_bounds,
            'noise_sigma': params['noise_sigma'],
            'min_dist': min_dist,
            'nsteps': 1000 }
    return ret

def navigate(display_name, enable_ssa):
    try:
        params = param.params
    except Exception as e:
        print(e)
        return
    display = display_for_name(display_name)
    env_params = run_kwargs(params)
    env = simu_env.Env(display, **(env_params))

    # ssa
    safe_controller = SafeSetAlgorithm(max_speed = env.robot_state.max_speed, dmin = env.min_dist * 2)

    # dynamic model parameters
    episode_num = 0
    collision_num = 0
    failure_num = 0
    success_num = 0
    sensing_range = env.min_dist * 6
    fx = np.array([[0,0,1,0],[0,0,0,1],[0,0,0,0],[0,0,0,0]])
    gx = np.array([[1,0],[0,1],[1,0],[0,1]])
    state, done = env.reset(), False

    while(1):
      env.display_start()
      obstacle_ids, obstacles = env.detect_obstacles(sensing_range)
      action = env.random_action()
      # compute safe control
      if (enable_ssa):
        action, is_safe = safe_controller.get_safe_control(state, obstacles, fx, gx, action)
      else:
         is_safe = False
      state, reward, done = env.step(action, is_safe, obstacle_ids) 
      env.display_end()

      if (done and reward == -500):          
        collision_num += 1      
      elif (done and reward == 2000):
        success_num += 1
      elif (done):
        failure_num += 1
      time.sleep(0.05)
      if (done):
        episode_num += 1
        print(f"Train: episode_num {episode_num}, success_num {success_num}, collision_num {collision_num}")
        state, done = env.reset(), False

if __name__ == '__main__':
    args = parser().parse_args()
    navigate(display_name = args.display, enable_ssa = args.enable_ssa)


