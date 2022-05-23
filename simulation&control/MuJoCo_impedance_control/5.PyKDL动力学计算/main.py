import mujoco_py as mp
import func_mj
import func_kdl

if __name__ == '__main__':
    model = mp.load_model_from_path('ur5.xml')
    sim = mp.MjSim(model)
    viewer = mp.MjViewer(sim)

    RbtMj = func_mj.RobotMj(sim, 'ee')
    RbtKdl = func_kdl.RobotKdl(sim)

    initial_qpos = {
        'shoulder_pan_joint': -3.1,
        'shoulder_lift_joint': -1.6,
        'elbow_joint': 1.6,
        'wrist_1_joint': -1.6,
        'wrist_2_joint': -1.6,
        'wrist_3_joint': 0,
    }

    for name, value in initial_qpos.items():
        sim.data.set_joint_qpos(name, value)

    sim.forward()

    print('mujoco:', RbtMj.fk())
    print('pykdl:', RbtKdl.fk())

    print('mujoco:', RbtMj.coriolis_gravity())
    print('pykdl:', RbtKdl.coriolis() + RbtKdl.gravity_torque())

    while 1:
        sim.data.ctrl[:] = RbtKdl.coriolis() + RbtKdl.gravity_torque()
        sim.step()
        viewer.render()

