#!/usr/bin/env python3
import numpy as np
import rospy

from duckietown.dtros import DTROS, NodeType, TopicType
from duckietown_msgs.msg import WheelEncoderStamped
from geometry_msgs.msg import PoseStamped

import odometry_activity

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

        # Initialize the DTROS parent class
        super(EncoderPoseNode, self).__init__(
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

        self.x_prev = 0
        self.y_prev = 0
        self.theta_prev = 0

        self.x_curr = 0
        self.y_curr = 0
        self.theta_curr = 0

        # nominal R and L:
        self.R = rospy.get_param(f'/{self.veh}/kinematics_node/radius', 100)
        self.L = rospy.get_param(f'/{self.veh}/kinematics_node/baseline', 100)

        # Construct publishers
        self.db_estimated_pose = rospy.Publisher(
            f'/{self.veh}/encoder_localization',
            PoseStamped,
            queue_size=1,
            dt_topic_type=TopicType.LOCALIZATION
        )

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

        self.log("Initialized!")

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

        # compute the new pose
        self.posePublisher()

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

        # compute the new pose
        self.posePublisher()

    def posePublisher(self):
        """
            Publish the pose of the Duckiebot given by the kinematic model
                using the encoders.
            Publish:
                ~/encoder_localization (:obj:`PoseStamped`): Duckiebot pose.
        """
        self.x_curr, self.y_curr, self.theta_curr = odometry_activity.poseEstimation(
            self.R, self.L,
            self.x_prev, self.y_prev, self.theta_prev,
            self.delta_phi_left, self.delta_phi_right)

        self.x_prev = self.x_curr
        self.y_prev = self.y_curr
        self.theta_prev = self.theta_curr

        pose = PoseStamped()

        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = 'map'

        pose.pose.position.x = self.x_curr
        pose.pose.position.y = self.y_curr
        pose.pose.position.z = 0

        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 0
        pose.pose.orientation.z = np.sin(self.theta_curr/2)
        pose.pose.orientation.w = np.cos(self.theta_curr/2)

        self.db_estimated_pose.publish(pose)

    #
    # Pose estimation is the function that is created by the user.
    #

    def onShutdown(self):
        super(EncoderPoseNode, self).onShutdown()


if __name__ == "__main__":
    # Initialize the node
    encoder_pose_node = EncoderPoseNode(node_name='encoder_pose_node')
    # Keep it spinning
    rospy.spin()
