#!/usr/bin/env python3
import os
import rospy
from duckietown.dtros import DTROS, NodeType, TopicType
from duckietown_msgs.msg import Twist2DStamped
from std_msgs.msg import String
import time

if __name__ == "__main__":
    veh = os.getenv('VEHICLE_NAME')


    TRIM_PARAM = f'/{veh}/kinematics_node/trim'
    SAVE_TRIM = f'/{veh}/kinematics_node/save_calibration'

    inter_node = DTROS(
        node_name="PIDstarterNode",
        node_type=NodeType.DIAGNOSTICS
    )

    # Construct publishers
    car_cmd_topic = f'/{veh}/joy_mapper_node/car_cmd'
    pub_car_cmd = rospy.Publisher(
        car_cmd_topic,
        Twist2DStamped,
        queue_size=1,
        dt_topic_type=TopicType.CONTROL
    )
    activity_name = rospy.Publisher(
        f'/activity_name',
        String,
        queue_size=1,
        dt_topic_type=TopicType.LOCALIZATION
    )

    msg = String()
    msg.data="pid"
    activity_name.publish(msg)


    car_control_msg = Twist2DStamped()
    car_control_msg.header.stamp = rospy.Time.now()

    car_control_msg.v = 0  # v
    car_control_msg.omega = 0  # omega

    pub_car_cmd.publish(car_control_msg)

    time.sleep(3)
