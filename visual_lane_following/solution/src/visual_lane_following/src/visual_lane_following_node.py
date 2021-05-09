#!/usr/bin/env python3
import os

import cv2
import time
import rospy
import numpy as np
import yaml
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from duckietown_msgs.msg import Twist2DStamped
from duckietown.dtros import DTROS, NodeType, TopicType
from duckietown.utils.image.ros import compressed_imgmsg_to_rgb

import visual_lane_following_activity


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

        self.VLF_ACTION = None
        self.VLF_STOPPED = True

        # Used for AIDO evaluation
        self.AIDO_eval = rospy.get_param(f"/{self.veh}/AIDO_eval", False)
        self.log(f"AIDO EVAL VAR: {self.AIDO_eval}")

        # Active only when submitting and evaluating (PID Exercise)
        if self.AIDO_eval:
            self.log("Starting evaluation for Visual Lane Following.")
            self.VLF_ACTION = "go"
            self.VLF_STOPPED = False

        # Defining subscribers:
        rospy.Subscriber(
            f"/{self.veh}/camera_node/image/compressed",
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

        self._lt_lm_pub = rospy.Publisher(
            f"/{self.veh}/visual_lane_following/left_lane_markings",
            Marker,
            queue_size=1,
            dt_topic_type=TopicType.PERCEPTION
        )

        self._rt_lm_pub = rospy.Publisher(
            f"/{self.veh}/visual_lane_following/right_lane_markings",
            Marker,
            queue_size=1,
            dt_topic_type=TopicType.VISUALIZATION
        )

        self._theta_pub = rospy.Publisher(
            f"/{self.veh}/visual_lane_following/lane_orientation",
            Marker,
            queue_size=1,
            dt_topic_type=TopicType.VISUALIZATION
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
        # height_original, width_original = image.shape[0:2]
        # img_size = image.shape[0:2]
        # print(img_size)
        # print('width_original = %d, height_original = %d' % (width_original, height_original))
        # if img_size[0] != width_original or img_size[1] != height_original:
        #     image = cv2.resize(image, tuple(reversed(img_size)), interpolation=cv2.INTER_NEAREST)
        #     print('RESIZING IMAGE')

        if self.is_shutdown:
            self.publish_command([0, 0])
            return

        time_now = time.time()
        delta_time = time_now - self.time

        self.time = time_now

        (self.theta_hat_curr, lm_left_ground, lm_right_ground) = visual_lane_following_activity.estimate_lane_relative_heading(self.H, image)

        u, self.prev_e, self.prev_int = visual_lane_following_activity.PIDController(self.v_0, self.theta_ref,
                                                                                     self.theta_hat_curr, self.prev_e,
                                                                                     self.prev_int, delta_time)

        self.publish_estimated_orientation_as_marker(self.theta_hat_curr)

        self.publish_lines_as_marker(lm_left_ground, lm_right_ground)

        # Override the forward velocity in case the PIDController changed it
        u[0] = self.v_0

        # self.logging to screen for debugging purposes
        self.log("    VISUAL CONTROL    ")
        self.log(f"Estimate theta_hat : {np.round(self.theta_hat_curr * 180 / np.pi, 2)}")

        if self.VLF_ACTION != "go" or self.VLF_STOPPED:
            # self.logging to screen for debugging purposes
            self.log("    VISUAL CONTROL (INACTIVE)    ")
            self.log(f"Estimate theta_hat : {np.round(self.theta_hat_curr * 180 / np.pi, 2)}")
            return

        self.publish_command(u)

        # self.logging to screen for debugging purposes
        self.log("    VISUAL CONTROL    ")
        self.log(f"Estimate theta_hat : {np.round(self.theta_hat_curr*180/np.pi, 2)}")
        self.log(f"Control error : {np.round(self.prev_e, 2)}, integral : {np.round(self.prev_int, 2)}")
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

    def publish_estimated_orientation_as_marker(self, theta_hat):
        theta_marker = Marker()
        theta_marker.header.frame_id = "map"
        theta_marker.header.stamp = rospy.Time.now()
        theta_marker.id = 2
        theta_marker.action = theta_marker.ADD
        theta_marker.lifetime = rospy.Duration.from_sec(0.5)
        theta_marker.type = theta_marker.ARROW

        theta_marker.scale.x = 0.03
        theta_marker.scale.y = 0.05
        theta_marker.scale.z = 0.0

        theta_marker.color.r = 0.0
        theta_marker.color.g = 1.0
        theta_marker.color.b = 1.0
        theta_marker.color.a = 1.0

        theta_marker.pose.position.x = 0.0
        theta_marker.pose.position.y = 0.0
        theta_marker.pose.position.z = 0.0
        theta_marker.pose.orientation.w = 1.0

        theta_marker.points = []
        p1 = Point()
        p2 = Point()
        p1.x = 0.0
        p1.y = 0.0
        p1.z = 0.0
        p2.x = np.cos(-theta_hat) * 0.5
        p2.y = np.sin(-theta_hat) * 0.5
        p2.z = 0.0

        theta_marker.points.append(p1)
        theta_marker.points.append(p2)
        self._theta_pub.publish(theta_marker)

    def publish_lines_as_marker(self, lm_left_ground, lm_right_ground):
        """
            Publishes markers to visualize line segments.

            Args:
                lm_left_ground:  An n x 4 array of candidate lines for the right lane markings (numpy.ndarray)
                                 Each row [x0, y0, x1, y1] specifies the line-segment coordinate in the ground frame
                lm_right_ground: An m x 4 array of candidate lines for the left lane markings (numpy.ndarray)
                                 Each row [x0, y0, x1, y1] specifies the line-segment coordinate in the ground frame

            Returns:
                the trimmed value
        """

        # Right lane markings
        if (lm_right_ground is not None) and lm_right_ground.size != 0:
            marker_right = Marker()
            marker_right.header.frame_id = "map"#self.veh
            marker_right.header.stamp = rospy.Time.now()
            marker_right.id = 0
            marker_right.action = marker_right.ADD
            marker_right.lifetime = rospy.Duration.from_sec(0.5)
            marker_right.type = marker_right.LINE_LIST

            marker_right.scale.x = 0.025
            marker_right.scale.y = 0.2
            marker_right.scale.z = 0.2

            marker_right.color.r = 1.0
            marker_right.color.g = 1.0
            marker_right.color.b = 1.0
            marker_right.color.a = 1.0

            marker_right.pose.position.x = 0.0
            marker_right.pose.position.y = 0.0
            marker_right.pose.position.z = 0.0
            marker_right.pose.orientation.w = 1.0

            marker_right.points = []
            for line in lm_right_ground:
                line = line.flatten()
                p1 = Point()
                p2 = Point()
                p1.x = line[0]
                p1.y = line[1]
                p1.z = 0.0
                p2.x = line[2]
                p2.y = line[3]
                p2.z = 0.0

                marker_right.points.append(p1)
                marker_right.points.append(p2)
            self._rt_lm_pub.publish(marker_right)

        # Left lane markings
        if (lm_left_ground is not None) and lm_left_ground.size != 0:
            marker_left = Marker()
            marker_left.header.frame_id = "map"#self.veh
            marker_left.header.stamp = rospy.Time.now()
            marker_left.id = 1
            marker_left.action = marker_left.ADD
            marker_left.lifetime = rospy.Duration.from_sec(0.5)
            marker_left.type = marker_left.LINE_LIST

            marker_left.scale.x = 0.025
            marker_left.scale.y = 0.2
            marker_left.scale.z = 0.2

            marker_left.color.r = 1.0
            marker_left.color.g = 1.0
            marker_left.color.b = 0.0
            marker_left.color.a = 1.0

            marker_left.pose.position.x = 0.0
            marker_left.pose.position.y = 0.0
            marker_left.pose.position.z = 0.0
            marker_left.pose.orientation.w = 1.0

            marker_left.points = []
            for line in lm_left_ground:
                line = line.flatten()
                p1 = Point()
                p2 = Point()
                p1.x = line[0]
                p1.y = line[1]
                p1.z = 0.0
                p2.x = line[2]
                p2.y = line[3]
                p2.z = 0.0

                marker_left.points.append(p1)
                marker_left.points.append(p2)

            self._lt_lm_pub.publish(marker_left)


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
        Reads the saved parameters from `/data/config/calibrations/camera_extrinsics/DUCKIEBOTNAME.yaml`
        or uses the default values if the file doesn't exist. Adjusts the ROS parameters for the
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
