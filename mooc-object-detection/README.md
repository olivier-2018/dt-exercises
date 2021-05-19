# Activity and Exercise Instructions

Follow these instructions to run these activities.

The final exercise counts towards grading and must be submitted for evaluation if you are pursuing the MOOC verified track.

## Phase 0: System update

- 💻 Always make sure your Duckietown Shell is updated to the latest version. See [installation instructions](https://github.com/duckietown/duckietown-shell)

- 💻 Update the shell commands: `dts update`

- 💻 Pull latest containers on your laptop: `dts desktop update`

- 🚙 Clean and update your Duckiebot: `dts duckiebot update ROBOTNAME` (where `ROBOTNAME` is the name of your Duckiebot chosen during the initialization procedure.)

- 🚙 Reboot your Duckiebot.


## Phase 1: Notebooks walkthrough

 - Build the workspace with:

  💻$ `dts exercises build`
  
 - Run the notebooks **with the `--vnc` flag**

  💻$ `dts exercises lab --vnc`
  
This will run a [Jupyter Lab][lab] and open a browser window. Enter the password `quackquack`.

[lab]: https://jupyterlab.readthedocs.io/en/stable/

Click through to `01-CNN` and then click on [`cnn.ipynb`](localhost:8888/lab/tree/01-CNN/cnn.ipynb). Once you have completed that notebook, move on to the next. Make sure that you do go through them in order (especially `02`-`04`).

Since we are working with neural networks now, some of these exercises require you to train neural network models. We will use [Google Colaboratory](https://colab.research.google.com/) for this. As a result, having a Google account is a prerequisite. 



## Phase 2: Evaluate and refine your model and solution

You can finetune your solution and see how it behaves in the simulator using:

    💻$ `dts exercises test --sim` 

Similar to the last exercises, you can open up the vnc browser and look at the image stream in `rqt_image_view` to gauge the performance of your model. 

TODO: is there a special image that is published that shows the detections?

You can similarly run your agent on your Duckiebot (if you have a Jetson Nano) using:

    💻$ `dts exercises test -b ![DUCKIEBOT_NAME]`



## Phase 3: Submit the homework

Once you are satisfied with your agent's performance, you can submit it. It's a good idea to evaluate it locally with the exact challenge conditions first. This can be done with:

    💻$ `dts challenges evaluate`
    
Then finally submit with 

    💻$ `dts challenges submit`
