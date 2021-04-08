#!/usr/bin/env python3
import numpy as np
import rospy

from duckietown.dtros import DTROS, NodeType, TopicType
from duckietown_msgs.msg import WheelEncoderStamped
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool


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

        keypress_topic = f'/{self.veh}/EnterPressed'
        _ = rospy.Subscriber(
            keypress_topic,
            Bool,
            self.cbEnterPressed,
            queue_size=1
        )

        self.STARTED = False
        self.CIRCLE_STARTED = False
        self.CALC_DELTAS = False
        self.CALC_POSE = False
        self.count = 0
        self.X = np.array([])
        self.partial_x = np.zeros((3, 3))
        self.Y = np.array([])

        self.log("Initialized!")

    def cbEnterPressed(self, msg):
        if msg.data == True:
            if self.CALC_DELTAS:
                self.CALC_DELTAS=False
                self.CALC_POSE = True
            else:
                self.delta_phi_left=0
                self.delta_phi_right=0
                self.CALC_DELTAS=True

            if self.count == 3: # 4 circles
                self.CIRCLE_STARTED=False
                self.count = 0
                self.STARTED = False
                self.LSE(self.X, self.Y)
            # if not self.CIRCLE_STARTED:
            #     self.CIRCLE_STARTED=True
            #     self.STARTED = True
            # else:
            #     if self.STARTED:
            #         self.STARTED = False
            #     else:
            #         self.STARTED = True
                    
            #     if self.count == 3: # 4 circles
            #         self.CIRCLE_STARTED=False
            #         self.count = 0
            #         self.STARTED = False
            #         self.LSE(self.X, self.Y)
                


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
        if self.CALC_DELTAS:
            self.delta_phi_left += 2*np.pi*delta_ticks/total_ticks

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
        if self.CALC_DELTAS:
            self.delta_phi_right += 2*np.pi*delta_ticks/total_ticks

        # compute the new pose
        self.posePublisher()

    def posePublisher(self):
        """
            Publish the pose of the Duckiebot given by the kinematic model
                using the encoders.
            Publish:
                ~/encoder_localization (:obj:`PoseStamped`): Duckiebot pose.
        """
        self.x_curr, self.y_curr, self.theta_curr, x = self.poseEstimation(
            self.R, self.L,
            0, 0, 0,
            self.delta_phi_left, self.delta_phi_right)

        if self.CALC_POSE:
            self.x_curr, self.y_curr, self.theta_curr, x = self.poseEstimation(
                        self.R, self.L,
                        0, 0, 0,
                        self.delta_phi_left, self.delta_phi_right)
            np.concatenate((self.X, x), axis=1)
            self.count+=1
            self.CALC_POSE=False

        # if self.CIRCLE_STARTED:
        #     if self.STARTED and not (self.delta_phi_left==0 or self.delta_phi_right==0):
        #         self.partial_x += x
        #     if not self.STARTED:
        #         np.concatenate((self.X, self.partial_x), axis=1)
        #         self.partial_x = np.zeros((3, 3))

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

    def poseEstimation(self, R,
                    L,
                    x_prev,
                    y_prev,
                    theta_prev,
                    delta_phi_left,
                    delta_phi_right):
        """
            Calculate the current Duckiebot pose using dead reckoning approach,
            based on the kinematic model.

            Returns:
                x_curr, y_curr, theta_curr (:double: values)
        """
        w = [R, R/L, 1]
        x = np.array(
            [
                [
                    (delta_phi_left+delta_phi_right)*np.cos(theta_prev)/2,
                    (delta_phi_left+delta_phi_right)*np.sin(theta_prev)/2,
                    0
                ],
                [
                    0,
                    0,
                    (delta_phi_right-delta_phi_left)/2],
                [
                    x_prev,
                    y_prev,
                    theta_prev
                ]
            ])

        x_curr, y_curr, theta_curr = np.array(w).dot(x)
        
        return x_curr, y_curr, theta_curr


    def LSE(self, X, Y):
        """Compute the Least Square Error
        Args:
            X (:obj:`Array`): Kenematic model input
            y (:obj:`Array`): Pose from the AprilTags
        Return:
            R (:obj:`value`): estimate of R
            L (:obj:`value`): estimate of L
            error (:obj:`value`): error
        """
        sol, error, _, _  = np.linalg.lstsq(X, Y)

        print()
        print("R L 1 solution:")
        print(sol)
        print()

        return sol[0], sol[1], error

    def onShutdown(self):
        print("Bye, bye :)")


if __name__ == "__main__":
    # Initialize the node
    encoder_pose_node = EncoderPoseNode(node_name='encoder_pose_node')
    # Keep it spinning
    rospy.spin()
    rospy.on_shutdown(encoder_pose_node.onShutdown)
