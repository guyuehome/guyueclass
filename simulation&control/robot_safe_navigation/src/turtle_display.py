from __future__ import division
from __future__ import absolute_import

import math
import time
import turtle
import runner

class TurtleRunnerDisplay( runner.BaseRunnerDisplay ):

    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.x_bounds = (0.0,1.0)
        self.y_bounds = (0.0,1.0)
        self.obstacle_turtles = {}
        self.estimated_obstacle_turtles = {}
        self.robot_turtle = None

    def setup(self, x_bounds, y_bounds,
              in_bounds, goal_bounds,
              margin):
        self.x_bounds = x_bounds
        self.y_bounds = y_bounds
        self.margin = margin
        xmin,xmax = x_bounds
        ymin,ymax = y_bounds
        dx = xmax - xmin
        dy = ymax - ymin
        margin = 0.1
        turtle.setup(width=self.width, height=self.height)
        turtle.setworldcoordinates(xmin - (dx * margin),
                                   ymin - (dy * margin),
                                   xmax + (dx * margin),
                                   ymax + (dy * margin))
        turtle.tracer(0,1)
        turtle.hideturtle()
        turtle.penup()

        self._draw_goal(goal_bounds)
        self._draw_inbounds(in_bounds)

        self.robot_turtle = turtle.Turtle()
        self.robot_turtle.shape("triangle")
        self.robot_turtle.shapesize(0.3, 0.5)
        self.robot_turtle.penup()

    def _draw_inbounds(self, in_bounds):
        t = turtle.Turtle()
        t.hideturtle()
        t.pencolor("black")
        t.penup()
        t.setposition(in_bounds.x_bounds[0], in_bounds.y_bounds[0])
        t.pendown()
        t.setposition(in_bounds.x_bounds[1], in_bounds.y_bounds[0])
        t.setposition(in_bounds.x_bounds[1], in_bounds.y_bounds[1])
        t.setposition(in_bounds.x_bounds[0], in_bounds.y_bounds[1])
        t.setposition(in_bounds.x_bounds[0], in_bounds.y_bounds[0])

    def _draw_goal(self, goal_bounds):

        t = turtle.Turtle()
        t.hideturtle()
        t.color("green", "#aaffaa")
        t.penup()
        t.setposition(goal_bounds.x_bounds[0], goal_bounds.y_bounds[0])
        t.pendown()
        t.begin_fill()
        t.setposition(goal_bounds.x_bounds[1], goal_bounds.y_bounds[0])
        t.setposition(goal_bounds.x_bounds[1], goal_bounds.y_bounds[1])
        t.setposition(goal_bounds.x_bounds[0], goal_bounds.y_bounds[1])
        t.setposition(goal_bounds.x_bounds[0], goal_bounds.y_bounds[0])
        t.end_fill()
        
    def begin_time_step(self, t):
        for idx,trtl in list(self.obstacle_turtles.items()):
            trtl.clear()
            trtl.hideturtle()
        for idx,trtl in list(self.estimated_obstacle_turtles.items()):
            trtl.clear()
            trtl.hideturtle()
        self.robot_turtle.clear()
        self.robot_turtle.hideturtle()

    def obstacle_at_loc(self, i, x, y, nearest_obstacle = False, close_obstacle = False):
        if i not in self.obstacle_turtles:
            trtl = turtle.Turtle()
            trtl.shape("circle")
            trtl.color("grey")
            trtl.shapesize(self.margin * 20, self.margin * 20)
            trtl.penup()
            self.obstacle_turtles[i] = trtl
        self.obstacle_turtles[i].setposition(x,y)
        self.obstacle_turtles[i].color('grey')
        #Uncomment the following line to display obstacle IDs
        #self.obstacle_turtles[i]._write(str(i), 'center', 'arial')
        self.obstacle_turtles[i].showturtle()

    def obstacle_set_color(self, i, color = 'grey'):
        self.obstacle_turtles[i].color(color)

    def obstacle_estimated_at_loc(self, i, x, y, is_match=False):
        return
        if i not in self.estimated_obstacle_turtles:
            trtl = turtle.Turtle()
            trtl.shape("circle")
            trtl.color("#88ff88" if is_match else "#aa4444")
            trtl.shapesize(0.2,0.2)
            trtl.penup()
            self.estimated_obstacle_turtles[i] = trtl
        self.estimated_obstacle_turtles[i].color("#88ff88" if is_match else "#aa4444")
        self.estimated_obstacle_turtles[i].setposition(x,y)
        self.estimated_obstacle_turtles[i].showturtle()

    def robot_at_loc(self, x, y, h, is_ssa = False):
        self.robot_turtle.setposition(x,y)
        self.robot_turtle.settiltangle(h * 180 / math.pi)
        self.robot_turtle.color("red" if is_ssa else "black")
        self.robot_turtle.showturtle()

    def collision(self):
        self._explode_robot()

    def out_of_bounds(self):
        self._explode_robot()

    def navigation_done(self, retcode, t):
        if retcode in (runner.NAV_FAILURE_COLLISION,
                       runner.NAV_FAILURE_OUT_OF_BOUNDS,
                       runner.FAILURE_TOO_MANY_STEPS):
            self._explode_robot()

    def end_time_step(self, t):
        turtle.update()
        #time.sleep(0.1)

    def teardown(self):
        turtle.done()

    def _explode_robot(self):
        self.robot_turtle.shape("circle")
        self.robot_turtle.shapesize(1.0,1.0)
        self.robot_turtle.color("orange")

