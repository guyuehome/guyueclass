import mujoco_py as mp


if __name__ == '__main__':
    model = mp.load_model_from_path('ur5.xml')
    sim = mp.MjSim(model)
    viewer = mp.MjViewer(sim)
    while 1:
        viewer.render()