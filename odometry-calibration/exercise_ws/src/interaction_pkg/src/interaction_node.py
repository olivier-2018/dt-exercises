#!/usr/bin/env python3
import numpy as np
import rospy
import os
from duckietown.dtros import DTROS, NodeType, TopicType
from duckietown_msgs.msg import WheelEncoderStamped, Twist2DStamped
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

import threading
from std_srvs.srv import Empty
import time

EnterPressed = False


def thread_function(name):
    msg = Bool()
    global EnterPressed

    rate = rospy.Rate(10)

    while(not rospy.is_shutdown()):
        msg.data = EnterPressed
        keypressed.publish(msg)
        if EnterPressed:
            EnterPressed = False
        rate.sleep()


if __name__ == "__main__":
    veh = os.getenv('VEHICLE_NAME')

    SAVE_PARAMS = f'/{veh}/kinematics_node/save_calibration'

    inter_node = DTROS(
        node_name="InteractionNode",
        node_type=NodeType.DIAGNOSTICS
    )

    # Construct publishers
    keypressed = rospy.Publisher(
        f'/{veh}/EnterPressed',
        Bool,
        queue_size=1,
        dt_topic_type=TopicType.LOCALIZATION
    )

    x = threading.Thread(target=thread_function, args=(1,))
    x.start()

    while(not rospy.is_shutdown()):
        input("Put you Duckiebot on a lane and then press ENTER...")
        EnterPressed = True
        for i in range(4):
            dire = "clockwise" if i % 2 else "cunterclockwise"
            input(
                f"Move your Duckiebot {dire} along a circle then press ENTER...")
            EnterPressed = True
            time.sleep(5)

        R = rospy.get_param(f'/{veh}/kinematics_node/radius', 100)
        L = rospy.get_param(f'/{veh}/kinematics_node/baseline', 100)

        print(f"The R and L values found are : {R} cm , {L} cm respectively")
        if(input("Do you want to save them? (y/n): ") == "y"):
            rospy.set_param(f'/{veh}/kinematics_node/radius', R)
            rospy.set_param(f'/{veh}/kinematics_node/baseline', L)
            rospy.ServiceProxy(SAVE_PARAMS, Empty)
            break
        else:
            print(
                "Trim value not saved, you can replicate the procedure or close this program with CRTL+C")
