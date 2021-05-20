#!/usr/bin/env python3

import os
from functools import reduce

import cv2
import numpy as np
## Important - don't remove these imports even though they seem unneeded
import pyglet
from pyglet.window import key

from agent import PurePursuitPolicy
from utils import launch_env, seed, makedirs, display_img_seg_mask, _mod_mask

from setup import find_all_boxes_and_classes

class SkipException(Exception):
    pass
DATASET_DIR="../dataset"
IMAGE_SIZE=416


all_image_names = []
npz_index = 0

def save_npz(img, boxes, classes):
    global npz_index

    with makedirs(f"{DATASET_DIR}/images"):
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        cv2.imwrite(f"{DATASET_DIR}/images/{npz_index}.jpg", img)
    with makedirs(f"{DATASET_DIR}/labels"):

        #make boxes to xywh format:
        def xminyminxmaxymax2xywfnormalized(box, image_size):
            xmin, ymin, xmax, ymax = np.array(box, dtype=np.float64)
            center_x = (xmin+xmax)/2
            center_y = (ymin+ymax)/2
            width = xmax-xmin
            height = ymax-ymin

            normalized = np.array([center_x, center_y, width, height])/image_size
            return np.round(normalized, 5)

        boxes = np.array([xminyminxmaxymax2xywfnormalized(box, IMAGE_SIZE) for box in boxes])

        with open(f"{DATASET_DIR}/labels/{npz_index}.txt", "w") as f:
            for i in range(len(boxes)):
                f.write(f"{classes[i]} "+" ".join(map(str,boxes[i]))+"\n")

    all_image_names.append(f"{npz_index}")
    npz_index += 1

# some setup
seed(123)
MAX_STEPS = 1000
nb_of_steps = 0

# we interate over several maps to get more diverse data
possible_maps = [
    "loop_pedestrians",
    "udem1",
    "loop_dyn_duckiebots",
    "zigzag_dists"
]
env_id = 0
while True:
    if env_id >= len(possible_maps):
        env_id = env_id % len(possible_maps)
    env = launch_env(possible_maps[env_id])
    policy = PurePursuitPolicy(env)
    obs = env.reset()

    inner_steps = 0
    if nb_of_steps >= MAX_STEPS:
        break

    while True:
        if nb_of_steps >= MAX_STEPS or inner_steps > 100:
            break

        action = policy.predict(np.array(obs))

        obs, rew, done, misc = env.step(action)
        seg = env.render_obs(True)
        env.render(segment=int(nb_of_steps / 50) % 2 == 0)

        obs = cv2.resize(obs, (IMAGE_SIZE, IMAGE_SIZE))
        seg = cv2.resize(seg, (IMAGE_SIZE, IMAGE_SIZE))

        try:
            boxes, classes = find_all_boxes_and_classes(seg)
        except SkipException as e:
            print(e)
            continue

        for box in boxes:
            pt1 = (box[0], box[1])
            pt2 = (box[2], box[3])
            cv2.rectangle(obs, pt1, pt2, (255,0,0), 2)
        #display_img_seg_mask(obs, seg)

        save_npz(obs, boxes, classes)


        nb_of_steps += 1
        inner_steps += 1

        if done or inner_steps % 10 == 0:
            env.reset()
    if nb_of_steps >= MAX_STEPS:
        break

import subprocess

def run(input, exception_on_failure=False):
    try:
        program_output = subprocess.check_output(f"{input}", shell=True, universal_newlines=True, stderr=subprocess.STDOUT)
    except Exception as e:
        if exception_on_failure:
            raise e
        program_output = e.output

    return program_output

train_txt = np.array(all_image_names)
np.random.shuffle(train_txt)
nb_things = len(train_txt)
SPLIT_PERCENTAGE = 0.8
SPLIT_PERCENTAGE = int(SPLIT_PERCENTAGE * nb_things)
train_txt, val_txt = train_txt[:SPLIT_PERCENTAGE], train_txt[SPLIT_PERCENTAGE:]

def mv(img_name, to_train):
    dest = "train" if to_train else "val"

    with makedirs(f"{DATASET_DIR}/{dest}/images"):
        run(f"mv {DATASET_DIR}/images/{img_name}.jpg {DATASET_DIR}/{dest}/images/{img_name}.jpg")
    with makedirs(f"{DATASET_DIR}/{dest}/labels"):
        run(f"mv {DATASET_DIR}/labels/{img_name}.txt {DATASET_DIR}/{dest}/labels/{img_name}.txt")

for img in train_txt:
    mv(img, True)
for img in val_txt:
    mv(img, False)

run(f"rm -rf {DATASET_DIR}/images {DATASET_DIR}/labels")

