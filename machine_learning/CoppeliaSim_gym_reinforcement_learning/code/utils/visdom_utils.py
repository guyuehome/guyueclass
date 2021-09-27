import numpy as np
from visdom import Visdom

class VisdomLinePlotter(object):
    """Plots to Visdom"""
    def __init__(self, env_name='main'):
        self.viz = Visdom()
        self.env = env_name
        self.plots = {}
    def plot(self, x_label='x_label', var_name='y_label', split_name='curve_name', title_name='title', x=0, y=0):
        if var_name not in self.plots:
            self.plots[var_name] = self.viz.line(X=np.array([x,x]), Y=np.array([y,y]), env=self.env, opts=dict(
                legend=[split_name],
                title=title_name,
                xlabel=x_label,
                ylabel=var_name
            ))
        else:
            self.viz.line(X=np.array([x]), Y=np.array([y]), env=self.env, win=self.plots[var_name], name=split_name, update = 'append')

if __name__ == "__main__":
    import time

    # run 'visdom' in your terminal to start the visdom server before run this script
    
    plotter = VisdomLinePlotter(env_name='visdom_test')

    t = np.linspace(0, 10, 500)
    x = np.sin(t)
    for i in range(1000):
        plotter.plot('epochs', 'y_label', 'curve_name', 'title', t[i], x[i])
        time.sleep(0.01)
