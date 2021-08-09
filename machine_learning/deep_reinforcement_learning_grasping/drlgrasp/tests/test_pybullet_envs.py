import sys
sys.path.append('..')
from drlgrasp.pybullet_envs.kuka_reach_with_visual import KukaReachVisualEnv,CustomSkipFrame


import numpy as np

def test_kuka_reach_with_visual():
    env=CustomSkipFrame(KukaReachVisualEnv(is_render=True,is_good_view=False))
    init_state=env.reset()
    final_image_shape=env.kFinalImageSize
    assert init_state.shape==(1,4,final_image_shape['width'],final_image_shape['height'])

    action=np.random.random(3)
    state=env.step(action)

    assert state[0].shape==(1,4,final_image_shape['width'],final_image_shape['height'])

if __name__ == '__main__':

    test_kuka_reach_with_visual()