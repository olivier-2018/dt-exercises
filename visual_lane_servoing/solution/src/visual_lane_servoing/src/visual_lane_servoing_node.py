#!/usr/bin/env python3
import math
from typing import Optional

import cv2
import numpy as np
import rospy
import yaml
from duckietown_msgs.msg import EpisodeStart, WheelsCmdStamped
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

# TODO: fix this
import SOLUTIONS_visual_control_activity as visual_control_activity

from duckietown.dtros import DTROS, NodeType, TopicType
from duckietown.utils.image.ros import compressed_imgmsg_to_rgb, rgb_to_compressed_imgmsg


class LaneServoingNode(DTROS):
    """
    Performs a form of visual servoing based on estimates of the image-space lane orientation
    Args:
        node_name (:obj:`str`): a unique, descriptive name for the ROS node
    Configuration:

    Publisher:
        ~wheels_cmd (:obj:`WheelsCmdStamped`): The corresponding resulting wheel commands
    Subscribers:
        ~/image/compressed (:obj:`CompressedImage`):
            compressed image
    """

    right_tick_prev: Optional[int]
    left_tick_prev: Optional[int]
    delta_phi_left: float
    delta_phi_right: float

    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(LaneServoingNode, self).__init__(node_name=node_name,
                                               node_type=NodeType.LOCALIZATION)
        self.log("Initializing...")
        # get the name of the robot
        self.veh = rospy.get_namespace().strip("/")

        # Add the node parameters to the parameters dictionary

        self.theta_left = None
        self.theta_right = None

        self.cmd_left = 0.0
        self.cmd_right = 0.0

        self.y_prev = 0.0
        self.theta_prev = 0.0

        # The following are used for the Braitenberg exercise
        self.gain = 0.5
        self.left_matrix = None
        self.right_matrix = None

        # self._top_cutoff = np.floor(0.4 * 480).astype(int)
        # self._bottom_cutoff = np.floor(0.08 * 480).astype(int)

        w, h = 640, 480
        self._cutoff = ((int(0.5 * h), int(0.08 * h)), (int(0.1 * w), int(0.1 * w)))

        self.VLS_ACTIVITY = False

        # Parameters relevant to motor commands
        self.left_const = 0.1
        self.right_const = 0.1
        self.left_max = -math.inf
        self.right_max = -math.inf
        self.left_min = math.inf
        self.right_min = math.inf

        # Used for AIDO evaluation
        self.AIDO_eval = rospy.get_param(f"/{self.veh}/AIDO_eval", False)
        self.log(f"AIDO EVAL VAR: {self.AIDO_eval}")

        # Flags for a joyful learning experience
        # (spins only parts of this code depending on the icons pressed on the VNC desktop)
        self.BRAITENBERG_EXERCISE = False
        self.VLS_EXERCISE = False

        # Active only when submitting and evaluating (PID Exercise)
        if self.AIDO_eval:
            self.VLS_EXERCISE = True
            self.v_0 = 0.2
            self.log("Starting evaluation for Visual Lane Servoing.")

        # Defining subscribers:
        rospy.Subscriber(
            f"/{self.veh}/rectifier_node/image/compressed",
            CompressedImage,
            self.cb_image,
            buff_size=10000000,
            queue_size=1
        )

        # # select the current activity
        # rospy.Subscriber(f"/{self.veh}/activity_name", String, self.cbActivity, queue_size=1)
        #
        # # # AIDO challenge payload subscriber
        # episode_start_topic = f"/{self.veh}/episode_start"
        # rospy.Subscriber(episode_start_topic, EpisodeStart, self.cbEpisodeStart, queue_size=1)

        # Command publisher
        self.pub_wheels_cmd = rospy.Publisher(
            f"/{self.veh}/wheels_driver_node/wheels_cmd",
            WheelsCmdStamped,
            queue_size=1,
            dt_topic_type=TopicType.CONTROL
        )

        self._lt_mask_pub = rospy.Publisher(
            f"/{self.veh}/visual_control/left_mask/image/compressed",
            CompressedImage,
            queue_size=1
        )

        self._rt_mask_pub = rospy.Publisher(
            f"/{self.veh}/visual_control/right_mask/image/compressed",
            CompressedImage,
            queue_size=1
        )

        self.log("Initialized.")

    def cb_activity(self, msg):
        """
        Call the right functions according to desktop icon the parameter.
        """

        self.VLS_EXERCISE = False

        self.log("")
        self.log(f"Received activity {msg.data}")
        self.log("")

        self.BRAITENBERG_EXERCISE = msg.data == "braitenberg"
        self.VLS_ACTIVITY = msg.data == "vls"

    def cb_episode_start(self, msg: EpisodeStart):
        loaded = yaml.load(msg.other_payload_yaml, Loader=yaml.FullLoader)
        if "initial_pose" in loaded:
            ip = loaded["initial_pose"]
            self.y_prev = float(ip["y"])
            self.theta_prev = float(np.deg2rad(ip["theta_deg"]))
        else:
            self.logwarn("No initial pose received. If you are running this on a real robot "
                         "you can ignore this message.")
            self.y_prev = 0.0
            self.theta_prev = 0.0

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

        # if self.VLS_ACTIVITY:
        #     self.perform_servoing(image)

    # def perform_servoing(self, image):
    #     """
    #     Control the left and right motors based on the estimated orientation of the lane markings
    #     """
    #
    #     theta_left, theta_right, lt_mask, rt_mask = visual_control_activity.LMOrientation(image)
    #     residual_left, residual_right = \
    #         visual_control_activity.getMotorResiduals(theta_left, theta_right)
    #     cmd_left, cmd_right = self.computeCommands(residual_left, residual_right)
    #
    #     self.publish_command(cmd_left, cmd_right)
    #
    #     lt_mask = rgb_to_compressed_imgmsg(cv2.cvtColor(lt_mask, cv2.COLOR_BGR2RGB), "jpeg")
    #     rt_mask = rgb_to_compressed_imgmsg(cv2.cvtColor(rt_mask, cv2.COLOR_BGR2RGB), "jpeg")
    #
    #     self._lt_mask_pub.publish(lt_mask)
    #     self._rt_mask_pub.publish(rt_mask)
    #
    #     # self.logging to screen for debugging purposes
    #     self.log("              VISUAL LANE SERVOING             ")
    #     self.log(f"Orientation (Left) : {np.round(np.rad2deg(theta_left), 1)} deg,"
    #              f"  Orientation (Right) : {np.round(np.rad2deg(theta_right), 1)} deg")
    #     self.log(f"Command (Left) : {cmd_left},  Command (Right) : {cmd_right}")

    def perform_braitenberg(self, image):
        """
            Computes the PWM commands for the left and right motors using a Braitenberg-like controller
            Args:
                self:
                image:  BGR image from forward-facing camera
        """

        shape = (2,)

        if self.left_matrix is None:
            self.left_matrix_left_lm = visual_control_activity.get_motor_left_matrix_left_lane_markings(shape)
            self.left_matrix_right_lm = visual_control_activity.get_motor_left_matrix_right_lane_markings(shape)
            self.right_matrix_left_lm = visual_control_activity.get_motor_right_matrix_left_lane_markings(shape)
            self.right_matrix_right_lm = visual_control_activity.get_motor_right_matrix_right_lane_markings(shape)

        # Call the user-defined function to get the masks for the left
        # and right lane markings
        (lt_mask, rt_mask) = visual_control_activity.detect_lane_markings(image)

        l = float(np.sum(lt_mask * self.left_matrix_left_lm)) + float(np.sum(rt_mask * self.left_matrix_right_lm))
        r = float(np.sum(lt_mask * self.right_matrix_left_lm)) + float(np.sum(rt_mask * self.right_matrix_right_lm))

        # These are big numbers -- we want to normalize them.
        # We normalize them using the history

        # first, we remember the high/low of these raw signals
        self.left_max = max(l, self.left_max)
        self.right_max = max(r, self.right_max)
        self.left_min = min(l, self.left_min)
        self.right_min = min(r, self.right_min)

        # now rescale from 0 to 1
        ls = rescale(l, self.left_min, self.left_max)
        rs = rescale(r, self.right_min, self.right_max)

        pwm_left = self.left_const + self.gain * ls
        pwm_right = self.right_const + self.gain * rs

        self.publish_command(pwm_left, pwm_right)

        # Publish these out for visualization
        lt_mask = rgb_to_compressed_imgmsg(cv2.cvtColor(lt_mask, cv2.COLOR_BGR2RGB), "jpeg")
        rt_mask = rgb_to_compressed_imgmsg(cv2.cvtColor(rt_mask, cv2.COLOR_BGR2RGB), "jpeg")

        self._lt_mask_pub.publish(lt_mask)
        self._rt_mask_pub.publish(rt_mask)

        # self.logging to screen for debugging purposes
        self.log("    VISUAL SERVOING    ")
        self.log(f"Left: (Unnormalized) : {np.round(l, 1)},"
                 f"  Right (Unnormalized) : {np.round(r, 1)} deg")
        self.log(f"Left: (Normalized) : {np.round(ls, 1)},"
                 f"  Right (Normalized) : {np.round(rs, 1)} deg")
        self.log(f"Command (Left) : {pwm_left},  Command (Right) : {pwm_right}")

    def compute_commands(self, residual_left, residual_right):
        """
        Computes the PWM commands for the left and right motors
        Args:
            self:
            residual_left:  Residual for the left motor (double)
            residual_right: Residual for the right motor (double)

        Returns:
            pwm_left:  Command for the left motor (double)
            pwm_right: Command for the right motor (double)

        """

        # These are big numbers -- we want to normalize them.
        # We normalize them using the history

        # first, we remember the high/low of these raw signals
        self.left_max = max(residual_left, self.left_max)
        self.right_max = max(residual_right, self.right_max)
        self.left_min = min(residual_left, self.left_min)
        self.right_min = min(residual_right, self.right_min)

        # now rescale from 0 to 1
        ls = rescale(residual_left, self.left_min, self.left_max)
        rs = rescale(residual_right, self.right_min, self.right_max)

        pwm_left = self.left_const + ls
        pwm_right = self.right_const + rs

        return pwm_left, pwm_right

    def publish_command(self, cmd_left, cmd_right):
        """Publishes a wheel command message.

        Args:
            cmd_left (:obj:`double`): Command for the left wheel.
            cmd_right (:obj:`double`): Command for the right wheel.
        """

        # limiting output to limit, which is 1.0 for the duckiebot
        cmd_right_limited = self.trim(cmd_right, -1.0, 1.0)
        cmd_left_limited = self.trim(cmd_left, -1.0, 1.0)

        # Put the wheel commands in a message and publish
        msg_wheels_cmd = WheelsCmdStamped()
        msg_wheels_cmd.header.stamp = rospy.Time.now()

        msg_wheels_cmd.vel_right = cmd_right_limited
        msg_wheels_cmd.vel_left = cmd_left_limited
        self.pub_wheels_cmd.publish(msg_wheels_cmd)

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

    def onShutdown(self):
        super(LaneServoingNode, self).on_shutdown()

    def angle_clamp(self, theta):
        if theta > 2 * np.pi:
            return theta - 2 * np.pi
        elif theta < -2 * np.pi:
            return theta + 2 * np.pi
        else:
            return theta


def rescale(a: float, L: float, U: float):
    if np.allclose(L, U):
        return 0.0
    return (a - L) / (U - L)


if __name__ == "__main__":
    # Initialize the node
    encoder_pose_node = LaneServoingNode(node_name="visual_lane_servoing_node")
    # Keep it spinning
    rospy.spin()
