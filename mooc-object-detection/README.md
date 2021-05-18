# Instructions

Follow these instructions to run these activities.

The final exercise counts towards grading and must be submitted for evaluation if you are pursuing the MOOC verified track.

## Make sure your system is up to date

- 💻 Always make sure your Duckietown Shell is updated to the latest version. See [installation instructions](https://github.com/duckietown/duckietown-shell)

- 💻 Update the shell commands: `dts update`

- 💻 Pull latest containers on your laptop: `dts desktop update`

- 🚙 Clean and update your Duckiebot: `dts duckiebot update ROBOTNAME` (where `ROBOTNAME` is the name of your Duckiebot chosen during the initialization procedure.)

- 🚙 Reboot your Duckiebot.

## Execute the activities

- Clone this repository.

- Build the workspace: `dts exercises build`.

- Navigate to the folder and start the documentation with `dts exercises lab`. It will open a page in your browser. The login password is `quackquack`. Make sure you do not have other Jupyter notebooks already open.

- Go to the first folder (`01-CNN`), open the notebook file, and follow through.

- You will have to execute the activities in order, from `/01-CNN` to `/02-Object-Detection`. There will also be some parts where you will have to go and exercute Google Colab notebooks (so that you can have access to a GPU). We will provide these instructions where necessary. 

It is possible to run things on your local machine if you have a GPU (or even if you don't), but we are not providing instructions for this workflow to keep things simple and streamlined. 

## Submit the homework

- The instructions to submit the homework are at the end of the `/02-Object-Detection` notebook. Please follow them to submit your assignment.
