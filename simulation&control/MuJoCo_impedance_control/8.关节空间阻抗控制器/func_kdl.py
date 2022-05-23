import PyKDL as kdl
import utils


class RobotKdl:
    def __init__(self, sim):
        self.sim = sim
        self.chain = kdl.Chain()
        self.chain.addSegment(
            kdl.Segment("link1", kdl.Joint(kdl.Vector(0, 0, 0.089159), kdl.Vector(0, 0, 1), kdl.Joint.JointType(0)),
                        kdl.Frame(kdl.Rotation(1, 0, 0, 0, 1, 0, 0, 0, 1), kdl.Vector(0, 0, 0.089159)),
                        kdl.RigidBodyInertia(3.7, kdl.Vector(0, 0, 0),
                                             kdl.RotationalInertia(0.0102675, 0.0102675, 0.00666, 0, 0, 0))))
        self.chain.addSegment(
            kdl.Segment("link2", kdl.Joint(kdl.Vector(0, 0.13585, 0), kdl.Vector(0, 1, 0), kdl.Joint.JointType(0)),
                        kdl.Frame(kdl.Rotation(4.89658e-12, 0, 1, 0, 1, 0, -1, 0, 4.89658e-12),
                                  kdl.Vector(0, 0.13585, 0)),
                        kdl.RigidBodyInertia(8.393, kdl.Vector(0, 0, 0.28),
                                             kdl.RotationalInertia(0.884902, 0.884902, 0.0151074, 0, 0, 0))))
        self.chain.addSegment(
            kdl.Segment("link3", kdl.Joint(kdl.Vector(0, -0.1197, 0.425), kdl.Vector(0, 1, 0), kdl.Joint.JointType(0)),
                        kdl.Frame(kdl.Rotation(1, 0, 0, 0, 1, 0, 0, 0, 1), kdl.Vector(0, -0.1197, 0.425)),
                        kdl.RigidBodyInertia(2.275, kdl.Vector(0, 0, 0.196125),
                                             kdl.RotationalInertia(0.118725, 0.118725, 0.004095, 0, 0, 0))))
        self.chain.addSegment(
            kdl.Segment("link4", kdl.Joint(kdl.Vector(0, 0, 0.39225), kdl.Vector(0, 1, 0), kdl.Joint.JointType(0)),
                        kdl.Frame(kdl.Rotation(4.89658e-12, 0, 1, 0, 1, 0, -1, 0, 4.89658e-12),
                                  kdl.Vector(0, 0, 0.39225)),
                        kdl.RigidBodyInertia(1.219, kdl.Vector(0, 0.093, 0),
                                             kdl.RotationalInertia(0.013103, 0.0025599, 0.0127373, 0, 0, 0))))
        self.chain.addSegment(
            kdl.Segment("link5", kdl.Joint(kdl.Vector(0, 0.093, 0), kdl.Vector(0, 0, 1), kdl.Joint.JointType(0)),
                        kdl.Frame(kdl.Rotation(1, 0, 0, 0, 1, 0, 0, 0, 1), kdl.Vector(0, 0.093, 0)),
                        kdl.RigidBodyInertia(1.219, kdl.Vector(0, 0, 0.09465),
                                             kdl.RotationalInertia(0.0134805, 0.0134805, 0.0021942, 0, 0, 0))))
        self.chain.addSegment(
            kdl.Segment("link6", kdl.Joint(kdl.Vector(0, 0, 0.09465), kdl.Vector(0, 1, 0), kdl.Joint.JointType(0)),
                        kdl.Frame(kdl.Rotation(1, 0, 0, 0, 1, 0, 0, 0, 1), kdl.Vector(0, 0, 0.09465)),
                        kdl.RigidBodyInertia(0.1879, kdl.Vector(0, 0.06505, 0),
                                             kdl.RotationalInertia(0.000879795, 0.000132117, 0.000879795, 0, 0,
                                                                   -2.32202e-16))))
        self.jac_solver = kdl.ChainJntToJacSolver(self.chain)
        self.jacdot_solver = kdl.ChainJntToJacDotSolver(self.chain)
        self.fk_solver = kdl.ChainFkSolverPos_recursive(self.chain)
        self.ik_solver = kdl.ChainIkSolverPos_LMA(self.chain, maxiter=1500)
        self.gravity = kdl.Vector(0, 0, -9.81)
        self.dyn_params = kdl.ChainDynParam(self.chain, self.gravity)

    def mass_matrix(self):
        input_q = utils.array2jnt_array(self.sim.data.qpos[:])
        output = kdl.JntSpaceInertiaMatrix(6)
        self.dyn_params.JntToMass(input_q, output)
        return utils.matrix2array(output)

    def jacobian(self):
        input_q = utils.array2jnt_array(self.sim.data.qpos[:])
        output = kdl.Jacobian(6)
        self.jac_solver.JntToJac(input_q, output)
        return utils.matrix2array(output)

    def jacobian_dot(self):
        input_q = utils.array2jnt_array(self.sim.data.qpos[:])
        input_qd = utils.array2jnt_array(self.sim.data.qvel[:])
        input_qav = kdl.JntArrayVel(input_q, input_qd)
        output = kdl.Jacobian(6)
        self.jacdot_solver.JntToJacDot(input_qav, output)
        return utils.matrix2array(output)

    def coriolis(self):
        input_q = utils.array2jnt_array(self.sim.data.qpos[:])
        input_qd = utils.array2jnt_array(self.sim.data.qvel[:])
        output = kdl.JntArray(6)
        self.dyn_params.JntToCoriolis(input_q, input_qd, output)
        return utils.jnt_array2array(output)

    def gravity_torque(self):
        input_q = utils.array2jnt_array(self.sim.data.qpos[:])
        output = kdl.JntArray(6)
        self.dyn_params.JntToGravity(input_q, output)
        return utils.jnt_array2array(output)

    def fk(self):
        q = utils.array2jnt_array(self.sim.data.qpos[:])
        frame = kdl.Frame()
        self.fk_solver.JntToCart(q, frame)
        r, p = [0] * 3, [[0] * 3 for _ in range(3)]
        for i in range(3):
            r[i] = frame.__getitem__((i, 3))
        for i in range(3):
            for j in range(3):
                p[i][j] = frame.__getitem__((i, j))
        return r, p

    def ik(self, init, pos, ori):
        pos, ori, ok = utils.check_pos_ori_valid(pos, ori)
        if not ok:
            return []
        q_init = utils.array2jnt_array(init)
        p = kdl.Frame(kdl.Rotation(ori[0][0], ori[0][1], ori[0][2],
                                   ori[1][0], ori[1][1], ori[1][2],
                                   ori[2][0], ori[2][1], ori[2][2]),
                      kdl.Vector(pos[0], pos[1], pos[2]))
        q = kdl.JntArray(6)
        self.ik_solver.CartToJnt(q_init, p, q)
        return utils.jnt_array2array(q)