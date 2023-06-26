from builtins import object
class BoundsRectangle(object):

    def __init__(self, x_bounds, y_bounds):
        self.x_bounds = x_bounds
        self.y_bounds = y_bounds

    def contains(self, xy):
        return ((self.x_bounds[0] <= xy[0] <= self.x_bounds[1])
                and (self.y_bounds[0] <= xy[1] <= self.y_bounds[1]))

    def __repr__(self):
        return f'(({self.x_bounds[0]}, {self.y_bounds[0]}), ({self.x_bounds[1]}, {self.y_bounds[1]}))'