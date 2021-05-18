import contextlib
import os

import numpy as np
import random
import math
import os


@contextlib.contextmanager
def directory(name):
    ret = os.getcwd()
    os.chdir(name)
    yield None
    os.chdir(ret)

@contextlib.contextmanager
def makedirs(name):
    try:
        os.makedirs(name)
    except:
        pass
    yield None

def seed(seed):
    # torch.manual_seed(seed)
    np.random.seed(seed)
    random.seed(seed)

def launch_env(map):
    import gym_duckietown
    from gym_duckietown.envs import DuckietownEnv
    env = DuckietownEnv(
        map_name=map,
        domain_rand=False,
        max_steps=math.inf,
    )
    return env

import cv2

def _mod_mask(mask):
    temp = mask.copy()
    temp[temp == 1] = 50
    temp[temp == 2] = 100
    temp[temp == 3] = 150
    temp[temp == 4] = 200
    temp = temp.astype("uint8")
    mask = cv2.applyColorMap(temp, cv2.COLORMAP_RAINBOW)
    return mask

def display_img_seg_mask(real_img, seg_img):
    all = np.concatenate(
        (cv2.cvtColor(real_img, cv2.COLOR_RGB2BGR), seg_img),
        axis=1
    )

    cv2.imshow("image", all)
    cv2.waitKey(0)