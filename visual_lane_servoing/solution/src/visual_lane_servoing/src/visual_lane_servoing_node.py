#!/usr/bin/env python3
import os
import cv2
import math
from typing import Optional

import numpy as np
import rospy
import yaml
from duckietown.dtros import DTROS, NodeType, TopicType
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped, EpisodeStart, WheelsCmdStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage

import visual_lane_servoing_activity


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
        super(LaneServoingNode, self).__init__(node_name=node_name, node_type=NodeType.LOCALIZATION)
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

        # Flags for a joyful learning experience (spins only parts of this code depending on the icons pressed on the VNC desktop)
        self.VLS_EXERCISE = False

        # Active only when submitting and evaluating (PID Exercise)
        if self.AIDO_eval:
            self.VLS_EXERCISE = True
            self.v_0 = 0.2
            self.log("Starting evaluation for PID lateral controller.")

        # Defining subscribers:
        rospy.Subscriber(
            f"/{self.veh}/image/compressed",
            CompressedImage,
            self.cbImage,
            buff_size=10000000,
            queue_size=1
        )

        # select the current activity
        rospy.Subscriber(f"/{self.veh}/activity_name", String, self.cbActivity, queue_size=1)

        # # AIDO challenge payload subscriber
        episode_start_topic = f"/{self.veh}/episode_start"
        rospy.Subscriber(episode_start_topic, EpisodeStart, self.cbEpisodeStart, queue_size=1)

        # Command publisher
        self.pub_wheels_cmd = rospy.Publisher(
            f"/{self.veh}/joy_mapper_node/car_cmd",
            WheelsCmdStamped,
            queue_size=1,
            dt_topic_type=TopicType.CONTROL
        )

        self.log("Initialized.")

    def cbEpisodeStart(self, msg: EpisodeStart):
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


    def cbActivity(self, msg):
        """
        Call the right functions according to desktop icon the parameter.
        """

        self.publishCmd([0, 0])
        self.VLS_ACTIVITY = False

        self.log("")
        self.log(f"Received activity {msg.data}")
        self.log("")

        self.VLS_ACTIVITY = msg.data == "vls"

    def cbImage(self, image_msg):
        """
        Processes the incoming image messages.

        Performs the following steps for each incoming image:

        #. Performs color correction
        #. Resizes the image to the ``~img_size`` resolution
        #. Removes the top ``~top_cutoff`` rows in order to remove the part of the image that doesn't include the road

        Args:
            image_msg (:obj:`sensor_msgs.msg.CompressedImage`): The receive image message

        """

        # Decode from compressed image with OpenCV
        try:
            image = self.bridge.compressed_imgmsg_to_cv2(image_msg)
        except ValueError as e:
            self.logerr('Could not decode image: %s' % e)
            return

        # Resize the image to the desired dimensions
        height_original, width_original = image.shape[0:2]
        img_size = image.shape[0:2]
        if img_size[0] != width_original or img_size[1] != height_original:
            image = cv2.resize(image, img_size, interpolation=cv2.INTER_NEAREST)
        image = image[self._top_cutoff:, :, :]

        self.performServoing(image)

    def performServoing(self, image):
        """
        Control the left and right motors based on the estimated orientation of the lane markings
        Publish:
            ~/encoder_localization (:obj:`PoseStamped`): Duckiebot pose.
        """

        theta_left, theta_right = visual_lane_servoing_activity.LMOrientation(image)
        residual_left, residual_right = visual_lane_servoing_activity.getMotorResiduals(theta_left, theta_right)
        cmd_left, cmd_right = computeCommands(self, residual_left, residual_right)
        self.publishCmd(cmd_left, cmd_right)

        # self.logging to screen for debugging purposes
        self.log("              VISUAL LANE SERVOING             ")
        self.log(f"Orientation (Left) : {np.rad2deg(theta_left)} deg,  Orientation (Right) : {np.rad2deg(theta_right)} deg")
        self.log(f"Command (Left) : {cmd_left},  Command (Right) : {cmd_right} deg")

    def computeCommands(self, residual_left, residual_right):
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

        left_const = self.left_const
        pwm_left = self.left_const + ls
        pwm_right = self.right_const + rs

        return pwm_left, pwm_right

    def publishCmd(self, cmd_left, cmd_right):
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

        car_control_msg = Twist2DStamped()
        car_control_msg.header.stamp = rospy.Time.now()

        car_control_msg.v = u[0]  # v
        car_control_msg.omega = u[1]  # omega
        # save omega in case of STOP
        self.omega = u[1]

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

    def onShutdown(self):
        super(LaneServoingNode, self).on_shutdown()


    def angle_clamp(self, theta):
        if theta > 2 * np.pi:
            return theta - 2 * np.pi
        elif theta < -2 * np.pi:
            return theta + 2 * np.pi
        else:
            return theta


if __name__ == "__main__":
    # Initialize the node
    encoder_pose_node = LaneServoingNode(node_name="visual_lane_servoing_node")
    # Keep it spinning
    rospy.spin()
