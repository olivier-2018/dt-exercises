#!/usr/bin/env python3
import math
import time
from typing import Optional

import cv2
import numpy as np
import rospy
import yaml
from duckietown_msgs.msg import EpisodeStart, Twist2DStamped
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
        self.v_0 = 0.15  # Forward velocity command
        self.omega_max = 8.3  # Maximum omega used to scale normalized steering command

        # The following are used for scaling
        self.steer_max = -math.inf

        # self._top_cutoff = np.floor(0.4 * 480).astype(int)
        # self._bottom_cutoff = np.floor(0.08 * 480).astype(int)

        w, h = 640, 480
        self._cutoff = ((int(0.5 * h), int(0.01 * h)), (int(0.1 * w), int(0.1 * w)))

        self.VLS_ACTIVITY = False
        self.VLS_CALIBRATED = False

        # Used for AIDO evaluation
        self.AIDO_eval = rospy.get_param(f"/{self.veh}/AIDO_eval", False)
        self.log(f"AIDO EVAL VAR: {self.AIDO_eval}")

        # Flags for a joyful learning experience
        # (spins only parts of this code depending on the icons pressed on the VNC desktop)
        self.VLS_EXERCISE = False

        # Active only when submitting and evaluating (PID Exercise)
        if self.AIDO_eval:
            self.VLS_EXERCISE = True
            self.log("Starting evaluation for Visual Lane Servoing.")

        # Defining subscribers:
        rospy.Subscriber(
            f"/{self.veh}/rectifier_node/image/compressed",
            CompressedImage,
            self.cb_image,
            buff_size=10000000,
            queue_size=1
        )

        # select the current activity
        rospy.Subscriber(f"/{self.veh}/activity_name", String, self.cb_activity, queue_size=1)

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

        for _ in range(5):
            self.log("Initializing...")
            time.sleep(1)
        self.log("Initialized!")
        self.log("Waiting for the Exercise App \"Visual Lane Servoing\" to be opened in VNC...")


        # TODO: DEBUG ONLY
        self.VLS_ACTIVITY = True
        self.VLS_CALIBRATED = True

    def cb_activity(self, msg):
        """
        Call the right functions according to desktop icon the parameter.
        """
        self.log("")
        self.log(f"Received activity {msg.data}")
        self.log("")
        vls_activity = msg.data == "vls"
        if not vls_activity:
            self.log(f"Activity '{msg.data}' not recognized. Exiting...")
            exit(1)
        if not self.AIDO_eval:
            self.log("Put the robot in a lane. Press [ENTER] when done.")
            # TODO: this is not supported by DTS and Docker Attach the way it is implemented right now
            # _ = input()
            self.log("Using your hands if you are working with a real robot, or the joystick if "
                     "you are working with the simulator. Turn the robot (in place), to the left "
                     "then to the right by about 30deg on each side. Press [ENTER] when done.")
            # TODO: this is not supported by DTS and Docker Attach the way it is implemented right now
            # _ = input()
            self.VLS_CALIBRATED = vls_activity
        self.VLS_ACTIVITY = vls_activity

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

        self.perform_vls(image)

    def perform_vls(self, image):
        """
            Computes the PWM commands for the left and right motors using a Braitenberg-like controller
            Args:
                self:
                image:  BGR image from forward-facing camera
        """
        if self.is_shutdown:
            self.publish_command([0, 0])
            return

        if not self.VLS_ACTIVITY:
            return

        shape = image.shape[0:2]

        steer_matrix_left_lm = visual_control_activity.get_steer_matrix_left_lane_markings(shape)
        steer_matrix_right_lm = visual_control_activity.get_steer_matrix_right_lane_markings(shape)

        # Call the user-defined function to get the masks for the left
        # and right lane markings
        (lt_mask, rt_mask) = visual_control_activity.detect_lane_markings(image)

        # Publish these out for visualization
        lt_mask_viz = cv2.addWeighted(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), 0.1,
                                      lt_mask.astype(np.uint8), 0.8, 0)
        rt_mask_viz = cv2.addWeighted(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), 0.1,
                                      rt_mask.astype(np.uint8), 0.8, 0)

        lt_mask_viz = rgb_to_compressed_imgmsg(cv2.cvtColor(lt_mask_viz, cv2.COLOR_GRAY2RGB), "jpeg")
        rt_mask_viz = rgb_to_compressed_imgmsg(cv2.cvtColor(rt_mask_viz, cv2.COLOR_GRAY2RGB), "jpeg")

        self._lt_mask_pub.publish(lt_mask_viz)
        self._rt_mask_pub.publish(rt_mask_viz)

        if not self.VLS_CALIBRATED:
            return

        steer = float(np.sum(lt_mask * steer_matrix_left_lm)) + \
                float(np.sum(rt_mask * steer_matrix_right_lm))

        self.steer_max = max(self.steer_max, 2 * max(float(np.sum(lt_mask * steer_matrix_left_lm)),
                                                     float(
                                                         np.sum(rt_mask * steer_matrix_right_lm))))

        # now rescale from 0 to 1
        steer_scaled = np.sign(steer) * rescale(np.abs(steer), 0, self.steer_max)

        u = [self.v_0, steer_scaled * self.omega_max]
        self.publish_command(u)

        # self.logging to screen for debugging purposes
        self.log("    VISUAL SERVOING    ")
        self.log(f"Steering: (Unnormalized) : {np.round(steer, 1)},"
                 f"  Steering (Normalized) : {np.round(steer_scaled, 1)}")
        self.log(f"Command v : {u[0]},  omega : {u[1]}")

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

    def angle_clamp(self, theta):
        if theta > 2 * np.pi:
            return theta - 2 * np.pi
        elif theta < -2 * np.pi:
            return theta + 2 * np.pi
        else:
            return theta

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
    encoder_pose_node = LaneServoingNode(node_name="visual_lane_servoing_node")
    # Keep it spinning
    rospy.spin()
