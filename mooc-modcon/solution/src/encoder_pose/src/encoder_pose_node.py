#!/usr/bin/env python3
import numpy as np
import rospy

from duckietown.dtros import DTROS, NodeType, TopicType
from duckietown_msgs.msg import WheelEncoderStamped, Twist2DStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

import odometry_activity
import PID_controller
import PID_controller_homework


import os
import yaml
import time


class EncoderPoseNode(DTROS):
    """
        Computes an estimate of the Duckiebot pose using the wheel encoders.
        Args:
            node_name (:obj:`str`): a unique, descriptive name for the ROS node
        Configuration:

        Publisher:
            ~encoder_localization (:obj:`PoseStamped`): The computed position
        Subscribers:
            ~/left_wheel_encoder_node/tick (:obj:`WheelEncoderStamped`):
                encoder ticks
            ~/right_wheel_encoder_node/tick (:obj:`WheelEncoderStamped`):
                encoder ticks
    """

    def __init__(self, node_name):
        print("Initializing...")
        # Initialize the DTROS parent class
        super(EncoderPoseNode, self).__init__(
            node_name=node_name,
            node_type=NodeType.LOCALIZATION
        )
        # get the name of the robot
        self.veh = rospy.get_namespace().strip("/")
        # Add the node parameters to the parameters dictionary
        self.delta_phi_left = 0
        self.left_tick_prev = ""

        self.delta_phi_right = 0
        self.right_tick_prev = ""

        self.x_prev = 0
        self.y_prev = 0
        self.theta_prev = 0

        self.x_curr = 0
        self.y_curr = 0
        self.theta_curr = 0

        self.prev_e = 0  # previous tracking error, starts at 0
        self.prev_int = 0  # previous tracking error integral, starts at 0
        self.time = 0

        self.v_0 = 0.5  # fixed robot linear velocity
        self.omega = 0.0
        self.y_ref = -0.10
        self.theta_ref = -90*np.pi/180

        # nominal R and L:
        print("Loading kinematics calibration...")
        self.R = 0.0318
        self.baseline = 0.1
        self.read_params_from_calibration_file()

        # Used for AIDO evaluation
        self.AIDO_eval = rospy.get_param(f'/{self.veh}/AIDO_eval', False)
        print(f"AIDO EVAL VAR: {self.AIDO_eval}")

        # Flags for a joyful learning experience :)
        self.ODOMETRY_ACTIVITY = False
        self.PID_ACTIVITY = False
        self.PID_EXERCISE = False

        if self.AIDO_eval:
            self.PID_EXERCISE = True

        # Defining subscribers:

        # select the current activity
        _ = rospy.Subscriber(
            f'/{self.veh}/activity_name',
            String,
            self.cbActivity,
            queue_size=1
        )

        _ = rospy.Subscriber(
            f'/{self.veh}/PID_parameters',
            String,
            self.cbPIDparam,
            queue_size=1
        )

        # Wheel encoder subscriber:
        left_encoder_topic = f'/{self.veh}/left_wheel_encoder_node/tick'
        _ = rospy.Subscriber(
            left_encoder_topic,
            WheelEncoderStamped,
            self.cbLeftEncoder,
            queue_size=1
        )

        # Wheel encoder subscriber:
        right_encoder_topic = f'/{self.veh}/right_wheel_encoder_node/tick'
        _ = rospy.Subscriber(
            right_encoder_topic,
            WheelEncoderStamped,
            self.cbRightEncoder,
            queue_size=1
        )

        # Odometry publisher
        self.db_estimated_pose = rospy.Publisher(
            f'/{self.veh}/encoder_localization',
            Odometry,
            queue_size=1,
            dt_topic_type=TopicType.LOCALIZATION
        )

        # Command publisher
        car_cmd_topic = f'/{self.veh}/joy_mapper_node/car_cmd'
        self.pub_car_cmd = rospy.Publisher(
            car_cmd_topic,
            Twist2DStamped,
            queue_size=1,
            dt_topic_type=TopicType.CONTROL
        )

        # Wait until the encoders data is received, then start the controller
        self.SIM_STARTED = False
        self.STOP = False

        # rospy.Timer(rospy.Duration(0.1), self.Controller)
        #rospy.Timer(rospy.Duration(0.02), self.posePublisher)

        # For encoders syncronization:
        self.RIGHT_RECEIVED = False
        self.LEFT_RECEIVED = False

        self.log("Initialized!")
        print("Initialized!")

    def cbPIDparam(self, msg):
        PID_parameters = msg.data

        if PID_parameters=="STOP":
            self.publishCmd([0, 0])
            self.STOP = True
            print("STOP")
            return

        PID_parameters=PID_parameters.split(";")
        print(PID_parameters)

        if self.PID_ACTIVITY:
            self.theta_ref = float(PID_parameters[0])*np.pi/180

        elif self.PID_EXERCISE:
            self.y_ref = float(PID_parameters[0])

        self.v_0 = float(PID_parameters[1])

        if self.STOP:
            # if the robot is not moving the wheel encoders
            # will not receive data and so there will be no pose update
            # for this reason we need to set a v_0 and move the robot 
            # in order to restart the controller.
            self.publishCmd([self.v_0, self.omega])

            self.STOP=False
        

    def cbActivity(self, msg):
        """
        change activity accoring to the param.
        """
        # Reset
        self.PID_ACTIVITY = False
        self.publishCmd([0, 0])
        self.ODOMETRY_ACTIVITY = False
        self.PID_EXERCISE = False

        print()
        print(f"Received activity {msg.data}")
        print()

        self.ODOMETRY_ACTIVITY = msg.data == "odometry"
        self.PID_ACTIVITY = msg.data == "pid"
        self.PID_EXERCISE = msg.data == "pid_exercise"

    def cbLeftEncoder(self, encoder_msg):
        """
            Wheel encoder callback, the rotation of the wheel.
            Args:
                encoder_msg (:obj:`WheelEncoderStamped`) encoder ROS message.
        """
        # Do nothing if the activity is not set
        if not (self.ODOMETRY_ACTIVITY or self.PID_ACTIVITY or self.PID_EXERCISE):
            return

        if self.left_tick_prev == "":
            ticks = encoder_msg.data
            self.left_tick_prev = ticks
            return

        delta_phi_left, self.left_tick_prev = odometry_activity.DeltaPhi(
            encoder_msg, self.left_tick_prev)
        self.delta_phi_left += delta_phi_left
        # compute the new pose
        self.LEFT_RECEIVED = True
        self.posePublisher()
        self.SIM_STARTED = True

    def cbRightEncoder(self, encoder_msg):
        """
            Wheel encoder callback, the rotation of the wheel.
            Args:
                encoder_msg (:obj:`WheelEncoderStamped`) encoder ROS message.
        """
        # Do nothing if the activity is not set
        if not (self.ODOMETRY_ACTIVITY or self.PID_ACTIVITY or self.PID_EXERCISE):
            return

        if self.right_tick_prev == "":
            ticks = encoder_msg.data
            self.right_tick_prev = ticks
            return

        delta_phi_right, self.right_tick_prev = odometry_activity.DeltaPhi(
            encoder_msg, self.right_tick_prev)
        self.delta_phi_right += delta_phi_right
        # compute the new pose
        self.RIGHT_RECEIVED = True
        self.posePublisher()
        self.SIM_STARTED = True

    def posePublisher(self):
        """
            Publish the pose of the Duckiebot given by the kinematic model
                using the encoders.
            Publish:
                ~/encoder_localization (:obj:`PoseStamped`): Duckiebot pose.
        """
        if self.STOP or not self.SIM_STARTED or not (self.LEFT_RECEIVED and self.RIGHT_RECEIVED):
            return

        self.LEFT_RECEIVED = self.RIGHT_RECEIVED = False

        self.x_curr, self.y_curr, theta_curr = odometry_activity.poseEstimation(
            self.R, self.baseline,
            self.x_prev, self.y_prev, self.theta_prev,
            self.delta_phi_left, self.delta_phi_right)

        self.theta_curr = self.angle_clamp(theta_curr)

        # Printing to screen for debugging purposes
        print("              ODOMETRY             ")
        print(f"Baseline : {self.baseline}   R: {self.R}")
        print(
            f"Theta : {self.theta_curr*180/np.pi}   x: {self.x_curr}   y: {self.y_curr}")
        print(
            f"Rotation left wheel : {np.rad2deg(self.delta_phi_left)}   Rotation right wheel : {np.rad2deg(self.delta_phi_right)}")
        print(
            f"Prev Ticks left : {self.left_tick_prev}   Prev Ticks right : {self.right_tick_prev}")
        print()

        # duckiebot_is_moving = (abs(self.delta_phi_left)
        #                        > 0 or abs(self.delta_phi_right) > 0)
        # Calculate new odometry only when new data from encoders arrives
        self.delta_phi_left = self.delta_phi_right = 0

        # Current estimate becomes previous estimate at next iteration
        self.x_prev = self.x_curr
        self.y_prev = self.y_curr
        self.theta_prev = self.theta_curr

        # Creating message to plot pose in RVIZ
        odom = Odometry()
        odom.header.frame_id = "map"
        odom.header.stamp = rospy.Time.now()

        odom.pose.pose.position.x = self.x_curr  # x position - estimate
        odom.pose.pose.position.y = self.y_curr  # y position - estimate
        odom.pose.pose.position.z = 0  # z position - no flying allowed in Duckietown

        # these are quaternions - stuff for a different course!
        odom.pose.pose.orientation.x = 0
        # these are quaternions - stuff for a different course!
        odom.pose.pose.orientation.y = 0
        # these are quaternions - stuff for a different course!
        odom.pose.pose.orientation.z = np.sin(self.theta_curr/2)
        # these are quaternions - stuff for a different course!
        odom.pose.pose.orientation.w = np.cos(self.theta_curr/2)

        self.db_estimated_pose.publish(odom)

        if (self.PID_ACTIVITY or self.PID_EXERCISE): # and duckiebot_is_moving:
            self.Controller()

    def Controller(self):
        """
        Calculate theta and perform the control actions given by the PID
        """
        # Do nothing if the PID activity is not set

        if not self.SIM_STARTED:
            return

        time_now = time.time()
        delta_time = time_now-self.time

        self.time = time_now

        if self.PID_ACTIVITY:

            u, self.prev_e, self.prev_int = PID_controller.PIDController(
                self.v_0,
                self.theta_ref,
                self.theta_curr,
                self.prev_e,
                self.prev_int,
                delta_time
            )

        elif self.PID_EXERCISE:
            u, self.prev_e, self.prev_int = PID_controller_homework.PIDController(
                self.v_0,
                self.y_ref,
                self.y_curr,
                self.prev_e,
                self.prev_int,
                delta_time
            )

        if u == []:
            return

        # self.setGain(u[1])
        self.publishCmd(u)
    #
    # Pose estimation is the function that is created by the user.
    #

    def publishCmd(self, u):
        """Publishes a car command message.

        Args:
            omega (:obj:`double`): omega for the control action.
        """

        car_control_msg = Twist2DStamped()
        car_control_msg.header.stamp = rospy.Time.now()

        car_control_msg.v = u[0]  # v
        car_control_msg.omega = u[1]  # omega
        # save omega in case of STOP
        self.omega = u[1]

        self.pub_car_cmd.publish(car_control_msg)

    def onShutdown(self):
        super(EncoderPoseNode, self).onShutdown()

    def read_params_from_calibration_file(self):
        """
        Reads the saved parameters from `/data/config/calibrations/kinematics/DUCKIEBOTNAME.yaml`
        or uses the default values if the file doesn't exist. Adjsuts the ROS paramaters for the
        node with the new values.
        """
        # Check file existence
        cali_file_folder = '/data/config/calibrations/kinematics/'
        fname = cali_file_folder + self.veh + ".yaml"
        # Use the default values from the config folder if a robot-specific file does not exist.
        if not os.path.isfile(fname):
            fname = cali_file_folder + "default.yaml"
            self.readFile(fname)
            self.logwarn(
                "Kinematics calibration %s not found! Using default instead." % fname)
        else:
            self.readFile(fname)

    def readFile(self, fname):
        with open(fname, 'r') as in_file:
            try:
                yaml_dict = yaml.load(in_file)
                print(yaml_dict)
                self.R = yaml_dict['radius']
                self.baseline = yaml_dict['baseline']
            except yaml.YAMLError as exc:
                self.logfatal(
                    "YAML syntax error. File: %s fname. Exc: %s" % (fname, exc))
                rospy.signal_shutdown()
                return

    def angle_clamp(self, theta):
        if theta > 2 * np.pi:
            return theta - 2 * np.pi
        elif theta < -2 * np.pi:
            return theta + 2 * np.pi
        else:
            return theta


if __name__ == "__main__":
    # Initialize the node
    encoder_pose_node = EncoderPoseNode(node_name='encoder_pose_node')
    # Keep it spinning
    rospy.spin()
