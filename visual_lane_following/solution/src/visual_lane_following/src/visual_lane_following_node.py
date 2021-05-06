#!/usr/bin/env python3
import os

import cv2
import time
import rospy
import numpy as np
import yaml
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import Twist2DStamped
from duckietown.dtros import DTROS, NodeType, TopicType
from duckietown.utils.image.ros import compressed_imgmsg_to_rgb

# TODO: fix this
import SOLUTIONS_visual_lane_following_activity as visual_lane_following_activity
# import visual_lane_following_activity


class LaneFollowingNode(DTROS):
    """
    Performs a form of visual following based on estimates of the image-space lane orientation
    Args:
        node_name (:obj:`str`): a unique, descriptive name for the ROS node
    Configuration:

    Publisher:
        ~wheels_cmd (:obj:`WheelsCmdStamped`): The corresponding resulting wheel commands
    Subscribers:
        ~/image/compressed (:obj:`CompressedImage`):
            compressed image
    """

    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(LaneFollowingNode, self).__init__(node_name=node_name,
                                               node_type=NodeType.LOCALIZATION)
        self.log("Initializing...")
        # get the name of the robot
        self.veh = rospy.get_namespace().strip("/")

        # The following are used for the Braitenberg exercise
        self.v_0 = 0.15  # Forward velocity command
        self.omega_max = 8.3  # Maximum omega used to scale normalized steering command

        # The following are used for the visual control (PID) exercise
        self.theta_ref = 0.0  # Desired lane-relative heading
        self.theta_hat_curr = 0.0
        # Initializing the PID controller parameters
        self.prev_e = 0.0  # previous tracking error, starts at 0
        self.prev_int = 0.0  # previous tracking error integral, starts at 0
        self.time = 0.0

        # self._top_cutoff = np.floor(0.4 * 480).astype(int)
        # self._bottom_cutoff = np.floor(0.08 * 480).astype(int)

        w, h = 640, 480
        self._cutoff = ((int(0.5 * h), int(0.01 * h)), (int(0.1 * w), int(0.1 * w)))

        self.VLF_ACTION = None
        self.VLF_STOPPED = True

        # Used for AIDO evaluation
        self.AIDO_eval = rospy.get_param(f"/{self.veh}/AIDO_eval", False)
        self.log(f"AIDO EVAL VAR: {self.AIDO_eval}")

        # Flags for a joyful learning experience
        # (spins only parts of this code depending on the icons pressed on the VNC desktop)
        self.VLF_EXERCISE = False
        self.VC_EXERCISE = False

        # Active only when submitting and evaluating (PID Exercise)
        if self.AIDO_eval:
            self.VLF_EXERCISE = True
            self.log("Starting evaluation for Visual Lane Following.")

        # Defining subscribers:
        rospy.Subscriber(
            f"/{self.veh}/rectifier_node/image/compressed",
            CompressedImage,
            self.cb_image,
            buff_size=10000000,
            queue_size=1
        )

        # select the current activity
        rospy.Subscriber(f"/{self.veh}/vlf_node/action", String, self.cb_action, queue_size=1)

        # Command publisher
        car_cmd_topic = f"/{self.veh}/joy_mapper_node/car_cmd"
        self.pub_car_cmd = rospy.Publisher(
            car_cmd_topic, Twist2DStamped, queue_size=1, dt_topic_type=TopicType.CONTROL
        )

        self._lt_mask_pub = rospy.Publisher(
            f"/{self.veh}/visual_lane_following/left_mask/image/compressed",
            CompressedImage,
            queue_size=1
        )

        self._rt_mask_pub = rospy.Publisher(
            f"/{self.veh}/visual_lane_following/right_mask/image/compressed",
            CompressedImage,
            queue_size=1
        )

        ext_calib = self.read_params_from_calibration_file()
        self.H = np.reshape(ext_calib['homography'], (3, 3))

        for _ in range(5):
            self.log("Initializing...")
            time.sleep(1)
        self.log("Initialized!")
        self.log("Waiting for the Exercise App \"Visual Lane Following\" to be opened in VNC...")

    def cb_action(self, msg):
        """
        Call the right functions according to desktop icon the parameter.
        """

        if msg.data not in ["init", "go", "stop"]:
            self.log(f"Activity '{msg.data}' not recognized. Exiting...")
            exit(1)

        self.VLF_ACTION = msg.data

        self.loginfo(f"ACTION: {self.VLF_ACTION}")

        if not self.AIDO_eval:
            if self.VLF_ACTION == "init":
                self.log("Put the robot in a lane. Press [Go] when done.")
                return

            if self.VLF_ACTION == "go":
                self.VLF_STOPPED = False
                # NOTE: this is needed to trigger the agent and get another image back
                self.publish_command([0, 0])
                return

            if self.VLF_ACTION == "stop":
                self.publish_command([0, 0])
                self.VLF_STOPPED = True
                return

    def cb_image(self, image_msg):
        """
        Processes the incoming image messages.

        Performs the following steps for each incoming image:

        #. Resizes the image to the ``~img_size`` resolution
        #. Removes the top ``~top_cutoff`` rows in order to remove the part of the
        image that doesn't include the road

        Args:
            image_msg (:obj:`sensor_msgs.msg.CompressedImage`): The receive image message

        """
        image = compressed_imgmsg_to_rgb(image_msg)
        # Resize the image to the desired dimensionsS
        height_original, width_original = image.shape[0:2]
        img_size = image.shape[0:2]
        if img_size[0] != width_original or img_size[1] != height_original:
            image = cv2.resize(image, tuple(reversed(img_size)), interpolation=cv2.INTER_NEAREST)

        # crop image
        (top, bottom), (left, right) = self._cutoff
        image = image[top:-bottom, left:-right, :]

        if self.is_shutdown:
            self.publish_command([0, 0])
            return

        time_now = time.time()
        delta_time = time_now - self.time

        self.time = time_now

        (self.theta_hat_curr, lines_left, lines_right) = visual_lane_following_activity.estimate_lane_relative_heading(self.H, image)

        u, self.prev_e, self.prev_int = visual_lane_following_activity.PIDController(self.v_0, self.theta_ref,
                                                                     self.theta_hat_curr, self.prev_e,
                                                                     self.prev_int, delta_time)

        # Override the forward velocity in case the PIDController changed it
        u[0] = self.v_0

        if self.VLF_ACTION != "go" or self.VLF_STOPPED:
            return

        self.publish_command(u)

        # self.logging to screen for debugging purposes
        self.log("    VISUAL CONTROL    ")
        self.log(f"Command v : {np.round(u[0], 2)},  omega : {np.round(u[1], 2)}")

    def publish_command(self, u):
        """Publishes a car command message.

        Args:
            u (:obj:`tuple(double, double)`): tuple containing [v, w] for the control action.
        """

        car_control_msg = Twist2DStamped()
        car_control_msg.header.stamp = rospy.Time.now()

        car_control_msg.v = u[0]  # v
        car_control_msg.omega = u[1]  # omega

        self.pub_car_cmd.publish(car_control_msg)

    @staticmethod
    def trim(value, low, high):
        """
        Trims a value to be between some bounds.

        Args:
            value: the value to be trimmed
            low: the minimum bound
            high: the maximum bound

        Returns:
            the trimmed value
        """
        return max(min(value, high), low)

    def read_params_from_calibration_file(self):
        """
        Reads the saved parameters from `/data/config/calibrations/kinematics/DUCKIEBOTNAME.yaml`
        or uses the default values if the file doesn't exist. Adjsuts the ROS paramaters for the
        node with the new values.
        """

        def readFile(fname):
            with open(fname, "r") as in_file:
                try:
                    return yaml.load(in_file, Loader=yaml.FullLoader)
                except yaml.YAMLError as exc:
                    self.logfatal("YAML syntax error. File: %s fname. Exc: %s" % (fname, exc))
                    return None

        # Check file existence
        cali_file_folder = "/data/config/calibrations/camera_extrinsic/"
        fname = cali_file_folder + self.veh + ".yaml"
        # Use the default values from the config folder if a robot-specific file does not exist.
        if not os.path.isfile(fname):
            fname = cali_file_folder + "default.yaml"
            self.logwarn("Extrinsics calibration %s not found! Using default instead." % fname)
            return readFile(fname)
        else:
            return readFile(fname)

    def on_shutdown(self):
        self.loginfo("Stopping motors...")
        self.publish_command([0, 0])
        time.sleep(0.5)
        self.loginfo("Motors stopped.")


def rescale(a: float, L: float, U: float):
    if np.allclose(L, U):
        return 0.0
    return (a - L) / (U - L)


if __name__ == "__main__":
    # Initialize the node
    encoder_pose_node = LaneFollowingNode(node_name="visual_lane_following_node")
    # Keep it spinning
    rospy.spin()
