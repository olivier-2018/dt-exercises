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

        self.y_prev = 0.0
        self.theta_prev = 0.0

        # The following are used for the Braitenberg exercise
        self.v_0 = 0.1  # Forward velocity command
        self.omega_max = 1.0  # Maximum omega used to scale normalized steering command
        self.steer_matrix_left_lm = None
        self.steer_matrix_right_lm = None

        # The following are used for scaling
        self.steer_max = -math.inf
        self.steer_min = math.inf

        # self._top_cutoff = np.floor(0.4 * 480).astype(int)
        # self._bottom_cutoff = np.floor(0.08 * 480).astype(int)

        w, h = 640, 480
        self._cutoff = ((int(0.5 * h), int(0.01 * h)), (int(0.1 * w), int(0.1 * w)))

        self.VLS_ACTIVITY = False

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

        # Command publisher
        car_cmd_topic = f"/{self.veh}/joy_mapper_node/car_cmd"
        self.pub_car_cmd = rospy.Publisher(
            car_cmd_topic, Twist2DStamped, queue_size=1, dt_topic_type=TopicType.CONTROL
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

        self.perform_braitenberg(image)

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

        shape = image.shape[0:2]

        if self.steer_matrix_left_lm is None:
            self.steer_matrix_left_lm = visual_control_activity.get_steer_matrix_left_lane_markings(shape)
            self.steer_matrix_right_lm = visual_control_activity.get_steer_matrix_right_lane_markings(shape)

        # Call the user-defined function to get the masks for the left
        # and right lane markings
        (lt_mask, rt_mask) = visual_control_activity.detect_lane_markings(image)
        
        steer = float(np.sum(lt_mask * self.steer_matrix_left_lm)) + float(np.sum(rt_mask * self.steer_matrix_right_lm))

        # These are big numbers -- we want to normalize them.
        # We normalize them using the history

        # first, we remember the high/low of these raw signals
        self.steer_max = max(l, self.steer_max)
        self.steer_min = min(l, self.steer_min)

        # now rescale from 0 to 1
        steer_scaled = rescale(steer, self.steer_min, self.steer_max)

        u = [self.v_0, steer_scaled * self.omega_max]
        self.publish_command(u)

        # Publish these out for visualization
        lt_mask = cv2.addWeighted(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), 0.1,
                                  lt_mask.astype(np.uint8), 0.8, 0)
        rt_mask = cv2.addWeighted(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), 0.1,
                                  rt_mask.astype(np.uint8), 0.8, 0)

        lt_mask = rgb_to_compressed_imgmsg(cv2.cvtColor(lt_mask, cv2.COLOR_GRAY2RGB), "jpeg")
        rt_mask = rgb_to_compressed_imgmsg(cv2.cvtColor(rt_mask, cv2.COLOR_GRAY2RGB), "jpeg")

        self._lt_mask_pub.publish(lt_mask)
        self._rt_mask_pub.publish(rt_mask)

        # self.logging to screen for debugging purposes
        self.log("    VISUAL SERVOING    ")
        self.log(f"Steering: (Unnormalized) : {np.round(steer, 1)},"
                 f"  Steering (Normalized) : {np.round(steer_scaled, 1)}")
        self.log(f"Command v : {u[0]},  omega : {u[1]}")

    def publish_command(self, u):
        """Publishes a car command message.

        Args:
            omega (:obj:`double`): omega for the control action.
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
