#!/usr/bin/env python3
import numpy as np
import rospy
import os
from duckietown.dtros import DTROS, NodeType, TopicType
from duckietown_msgs.msg import WheelEncoderStamped, Twist2DStamped
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

import threading
from std_srvs.srv import  Empty
import time

EnterPressed=False

def thread_function(name):
    msg = Bool()
    global EnterPressed
    
    rate = rospy.Rate(10)

    while(not rospy.is_shutdown()):
        msg.data = EnterPressed
        keypressed.publish(msg)
        if EnterPressed:
            EnterPressed=False
        rate.sleep()

if __name__ == "__main__":
    veh = os.getenv('VEHICLE_NAME')


    TRIM_PARAM = f'/{veh}/kinematics_node/trim'
    SAVE_TRIM = f'/{veh}/kinematics_node/save_calibration'

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
        input("Put you Duckiebot on a lane and press ENTER...")
        print("Enter pressed")
        EnterPressed=True
        input("Move your Duckiebot along a straight line for 2 m then press ENTER...")
        EnterPressed=True
        time.sleep(5)
        trim = rospy.get_param(TRIM_PARAM)
        print(f"The trim value found using the wheel encoders id {trim}")
        if(input("Do you want to save it? (y/n): ")=="y"):
            rospy.ServiceProxy(SAVE_TRIM, Empty)
            break
        else:
            print("Trim value not saved, you can replicate the procedure or close this program with CRTL+C")