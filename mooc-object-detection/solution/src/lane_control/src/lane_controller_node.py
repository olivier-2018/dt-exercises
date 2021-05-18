#!/usr/bin/env python3
import numpy as np
import rospy

from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import Twist2DStamped, LanePose, WheelsCmdStamped, BoolStamped, FSMState, StopLineReading

class LaneControllerNode(DTROS):
    """Computes control action.
    This version of lane control just drives straight forward
    Args:
        node_name (:obj:`str`): a unique, descriptive name for the node that ROS will use
    Configuration:
        ~v_bar (:obj:`float`): Nominal velocity in m/s

    Publisher:
        ~car_cmd (:obj:`Twist2DStamped`): The computed control action
    Subscribers:
    """

    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(LaneControllerNode, self).__init__(
            node_name=node_name,
            node_type=NodeType.CONTROL
        )

        # Add the node parameters to the parameters dictionary
        # TODO: MAKE TO WORK WITH NEW DTROS PARAMETERS
        self.params = dict()
        self.params['~v_bar'] = DTParam(
            '~v_bar',
            param_type=ParamType.FLOAT,
            min_value=0.0,
            max_value=5.0
        )

        # Construct publishers
        self.pub_car_cmd = rospy.Publisher("~car_cmd",
                                           Twist2DStamped,
                                           queue_size=1,
                                           dt_topic_type=TopicType.CONTROL)

        # we will just call the controller at 10Hz to publish constant velocity
        rospy.Timer(rospy.Duration(0.1), self.getControlAction)
        self.log("Initialized!")


    def publishCmd(self, car_cmd_msg):
        """Publishes a car command message.

        Args:
            car_cmd_msg (:obj:`Twist2DStamped`): Message containing the requested control action.
        """
        self.pub_car_cmd.publish(car_cmd_msg)


    def getControlAction(self, event):

        # Initialize car control msg, add header from input message
        car_control_msg = Twist2DStamped()
        car_control_msg.header.stamp = event.current_real

        # Add commands to car message
        car_control_msg.v = self.params['~v_bar'].value
        car_control_msg.omega = 0.0

        self.publishCmd(car_control_msg)

    def cbParametersChanged(self):
        """Updates parameters in the controller object."""

        self.controller.update_parameters(self.params)


if __name__ == "__main__":
    # Initialize the node
    lane_controller_node = LaneControllerNode(node_name='lane_controller_node')
    # Keep it spinning
    rospy.spin()
