#!/usr/bin/env python3
import numpy as np
import rospy

from duckietown.dtros import DTROS, NodeType, TopicType
from duckietown_msgs.msg import WheelEncoderStamped, Twist2DStamped
from geometry_msgs.msg import PoseStamped

from std_msgs.msg import Bool

import PID_controller

import time


class ControllerNode(DTROS):
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

        # Initialize the DTROS parent class
        super(ControllerNode, self).__init__(
            node_name=node_name,
            node_type=NodeType.LOCALIZATION
        )
        # get the name of the robot
        self.veh = rospy.get_namespace().strip("/")

        # Add the node parameters to the parameters dictionary
        self.delta_phi_left = 0
        self.left_tick_prev = 0

        self.delta_phi_right = 0
        self.right_tick_prev = 0

        self.current_e = 0
        self.time = 0
        self.delta_time = 0

        self.ticks_left = 0
        self.ticks_right = 0

        # nominal R and L:
        #self.R = rospy.get_param(f'/{self.veh}/kinematics_node/radius', 100)
        #self.L = rospy.get_param(f'/{self.veh}/kinematics_node/baseline', 100)

        # Wheel encoders subscribers:

        left_encoder_topic = f'/{self.veh}/left_wheel_encoder_node/tick'
        self.enco_left = rospy.Subscriber(
            left_encoder_topic,
            WheelEncoderStamped,
            self.cbLeftEncoder,
            queue_size=1
        )

        right_encoder_topic = f'/{self.veh}/right_wheel_encoder_node/tick'
        self.enco_right = rospy.Subscriber(
            right_encoder_topic,
            WheelEncoderStamped,
            self.cbRightEncoder,
            queue_size=1
        )

        # Param names
        car_cmd_topic = f'/{self.veh}/joy_mapper_node/car_cmd'
        self.pub_car_cmd = rospy.Publisher(
            car_cmd_topic,
            Twist2DStamped,
            queue_size=1,
            dt_topic_type=TopicType.CONTROL
        )

        self.log("Initialized!")

    def publishCmd(self, omega):
        """Publishes a car command message.

        Args:
            omega (:obj:`double`): omega for the control action.
        """
        car_control_msg = Twist2DStamped()
        car_control_msg.header.stamp = rospy.Time.now()

        car_control_msg.v = 0.3
        car_control_msg.omega = omega

        self.pub_car_cmd.publish(car_control_msg)

    def cbLeftEncoder(self, msg_encoder):
        """
            Wheel encoder callback, the rotation of the wheel.
            Args:
                msg_encoder (:obj:`WheelEncoderStamped`) encoder ROS message.
        """
        ticks = msg_encoder.data

        delta_ticks = ticks-self.left_tick_prev
        self.left_tick_prev = ticks

        total_ticks = msg_encoder.resolution

        # Assuming no wheel slipping
        self.delta_phi_left = 2*np.pi*delta_ticks/total_ticks
        if self.time == 0:
            self.time = time.time()
            return

        self.delta_time = time.time()-self.time

        self.time = time.time()

        omega, self.current_e = PID_controller.PIDController(
            self.delta_phi_right,
            self.delta_phi_left,
            self.current_e,
            self.delta_time
        )

        self.publishCmd(omega)

    def cbRightEncoder(self, msg_encoder):
        """
            Wheel encoder callback, the rotation of the wheel.
            Args:
                msg_encoder (:obj:`WheelEncoderStamped`) encoder ROS message.
        """
        ticks = msg_encoder.data

        delta_ticks = ticks-self.right_tick_prev
        self.right_tick_prev = ticks

        total_ticks = msg_encoder.resolution

        # Assuming no wheel slipping
        self.delta_phi_right = 2*np.pi*delta_ticks/total_ticks

        if self.time == 0:
            self.time = time.time()
            return

        self.delta_time = time.time()-self.time

        self.time = time.time()

        omega, self.current_e = PID_controller.PIDController(
            self.delta_phi_right,
            self.delta_phi_left,
            self.current_e,
            self.delta_time
        )

        self.publishCmd(omega)

    def onShutdown(self):
        print("Shutting down, bye, bye!")


if __name__ == "__main__":
    # Initialize the node
    controller_node = ControllerNode(node_name='controller_node')
    # Keep it spinning
    rospy.spin()
    rospy.on_shutdown(controller_node.onShutdown)
