#!/usr/bin/env python3
import numpy as np
import rospy

from duckietown.dtros import DTROS, NodeType, TopicType
from duckietown_msgs.msg import WheelEncoderStamped, Twist2DStamped
from geometry_msgs.msg import PoseStamped

from std_msgs.msg import Bool


class WheelsCalibrationNode(DTROS):
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
        super(WheelsCalibrationNode, self).__init__(
            node_name=node_name,
            node_type=NodeType.LOCALIZATION
        )
        # get the name of the robot
        self.veh = rospy.get_namespace().strip("/")

        # Add the node parameters to the parameters dictionary
        self.last_ticks_left = 0
        self.last_ticks_right = 0

        self.ticks_left = 0
        self.ticks_right = 0

        # nominal R and L:
        self.R = rospy.get_param(f'/{self.veh}/kinematics_node/radius', 100)
        self.L = rospy.get_param(f'/{self.veh}/kinematics_node/baseline', 100)

        # Wheel encoders subscribers:
        keypress_topic = f'/{self.veh}/EnterPressed'
        _ = rospy.Subscriber(
            keypress_topic,
            Bool,
            self.cbEnterPressed,
            queue_size=1
        )

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
        self.START_MOVING = False
        self.TRIM_PARAM = f'/{self.veh}/kinematics_node/trim'
        self.curr_trim = rospy.get_param(self.TRIM_PARAM, 0.0)

        self.log("Initialized!")

    def cbEnterPressed(self, msg):
        if msg.data == True:
            if not self.START_MOVING:
                self.last_ticks_left = self.ticks_left
                self.last_ticks_right = self.ticks_right
                self.START_MOVING = True
            else:
                calib_ticks_left = self.ticks_left-self.last_ticks_left
                calib_ticks_right = self.ticks_right-self.last_ticks_right

                if calib_ticks_right == 0 or calib_ticks_left == 0:
                    print("Please move the Duckiebot using the joystick from the VNC")
                else:
                    if calib_ticks_right > calib_ticks_left:
                        self.curr_trim = self.curr_trim - \
                            (1-calib_ticks_left/calib_ticks_right)
                    else:
                        self.curr_trim = self.curr_trim+1-calib_ticks_right/calib_ticks_left
                    print(f"Suggested trim = {self.curr_trim}")
                    rospy.set_param(self.TRIM_PARAM, self.curr_trim)
                self.START_MOVING = False

    def cbLeftEncoder(self, msg_encoder):
        """
            Wheel encoder callback, the rotation of the wheel.
            Args:
                msg_encoder (:obj:`WheelEncoderStamped`) encoder ROS message.
        """
        self.ticks_left = msg_encoder.data
        if not self.START_MOVING:
            return

        print(f"LEFT ticks : {self.ticks_left-self.last_ticks_left}")

    def cbRightEncoder(self, msg_encoder):
        """
            Wheel encoder callback, the rotation of the wheel.
            Args:
                msg_encoder (:obj:`WheelEncoderStamped`) encoder ROS message.
        """
        self.ticks_right = msg_encoder.data
        if not self.START_MOVING:
            return

        print(f"RIGHT ticks : {self.ticks_right-self.last_ticks_right}")

    def onShutdown(self):
        print("Shutting down, bye, bye!")


if __name__ == "__main__":
    # Initialize the node
    wheels_calibration_node = WheelsCalibrationNode(
        node_name='wheels_calibration_node')
    # Keep it spinning
    rospy.spin()
    rospy.on_shutdown(wheels_calibration_node.onShutdown)
