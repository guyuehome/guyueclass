import mujoco_py as mp

model = mp.load_model_from_path('ur5.xml')
sim = mp.MjSim(model)
viewer = mp.MjViewer(sim)

for i in range(3000):
    # sim.data.ctrl[:6] = 1
    sim.step()
    viewer.render()