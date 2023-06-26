import math
import numpy as np

class DoubleIntegratorRobot():

    def __init__(self, x, y, vx, vy, max_speed):

        self.x = x
        self.y = y
        self.v_x = vx
        self.v_y = vy
        self.max_speed = max_speed

    @property
    def position(self):
        heading_angle = math.atan2(self.v_y, self.v_x)
        return (self.x, self.y, heading_angle)

    def steer(self, vx_speed_change, vy_speed_change):
        """
        Returns a new Robot object.
        """
        self.v_x += vx_speed_change
        self.v_y += vy_speed_change
        self.v_x = max(min(self.v_x, self.max_speed), -self.max_speed)
        self.v_y = max(min(self.v_y, self.max_speed), -self.max_speed)

        new_x = self.x + self.v_x
        new_y = self.y + self.v_y
        # warp around
        if (new_x > 1):
            new_x = -1 + (new_x - 1) 
        if (new_x < -1):
            new_x = 1 + (new_x + 1) 
        if (new_y < -1):
            new_y = -1
        return DoubleIntegratorRobot(x = new_x, y = new_y, vx = self.v_x, vy = self.v_y, max_speed = self.max_speed)
