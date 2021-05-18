#!/usr/bin/env python3

import numpy as np

from agent import PurePursuitPolicy
from utils import launch_env, seed
from utils import launch_env, seed, makedirs, display_seg_mask, display_img_seg_mask
import cv2

DATASET_DIR= "../dataset"

npz_index = 0
def save_img(img, boxes, classes):
    global npz_index

    with makedirs(f"{DATASET_DIR}/images"):
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        cv2.imwrite(f"{DATASET_DIR}/images/{npz_index}.jpg", img)
    with makedirs(f"{DATASET_DIR}/labels"):
        with open(f"{DATASET_DIR}/labels/{npz_index}.txt", "w") as f:
            for i in range(len(boxes)):
                f.write(f"{classes[i]} " + " ".join(map(str, boxes[i])) + "\n")
    npz_index += 1

def clean_segmented_image(seg_img):
    # TODO
    # Tip: use the display function found in util.py to ensure that your cleaning produces clean masks
    # (ie masks akin to the ones from PennFudanPed) before extracting the bounding boxes
    pass
    # return boxes, classes

seed(123)
environment = launch_env()  # TODO go see this function in utils: there is a list of maps. You probably should
# TODO change this to a for loop, to iterate over all possible maps, to get a more diverse dataset

policy = PurePursuitPolicy(environment)

MAX_STEPS = 500 # TODO change this, this controls the maximum number of steps your agent takes

while True:
    obs = environment.reset()   # TODO you might want to move launch_env here, and specify a new map at each new loop?
    environment.render(segment=True)
    rewards = []

    nb_of_steps = 0

    while True:
        action = policy.predict(np.array(obs))

        obs, rew, done, misc = environment.step(action) # Gives non-segmented obs as numpy array
        segmented_obs = environment.render_obs(True)  # Gives segmented obs as numpy array

        rewards.append(rew)
        environment.render(segment=int(nb_of_steps / 50) % 2 == 0)

        # TODO boxes, classes = clean_segmented_image(segmented_obs)
        # TODO save_img(obs, boxes, classes)

        nb_of_steps += 1

        if done or nb_of_steps > MAX_STEPS:
            break