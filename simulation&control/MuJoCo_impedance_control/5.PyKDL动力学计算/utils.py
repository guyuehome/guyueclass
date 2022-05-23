import numpy as np
import PyKDL as kdl


def check_pos_ori_valid(pos, ori):
    if pos.ndim == 1:
        pos = np.expand_dims(pos, axis=1)
    if pos.shape == (1, 3):
        pos = pos.T
    if pos.shape != (3, 1) or ori.shape != (3, 3):
        return [], [], False
    return pos, ori, True


def jnt_array2array(x):
    return np.array([x.__getitem__(i) for i in range(x.rows())])


def matrix2array(x):
    m, n = x.rows(), x.columns()
    ret = [[0] * n for _ in range(m)]
    for i in range(m):
        for j in range(n):
            ret[i][j] = x.__getitem__((i, j))
    return ret


def array2jnt_array(x):
    ret = kdl.JntArray(len(x))
    for i in range(len(x)):
        ret[i] = x[i]
    return ret
