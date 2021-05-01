import numpy as np
import cv2
from matplotlib import pyplot as plt



class UnitTestMessage:
    # Test the WheelEncoderStamped messages
    def __init__(self, callback):
        from duckietown_msgs.msg import WheelEncoderStamped
        from std_msgs.msg import Header

        # creating a dummy wheel encoder message to allow testing how to read fields

        header = Header()
        header.seq = 372
        # rospy.Time.now() is the correct stamp, anyway this works only when a node is initialized
        header.stamp.secs = 1618436796
        header.stamp.nsecs = 55785179
        header.frame_id = f"agent/left_wheel_axis"

        encoder_msg = WheelEncoderStamped(
            header=header, data=4, resolution=135, type=WheelEncoderStamped.ENCODER_TYPE_INCREMENTAL
        )

        callback(encoder_msg)


class UnitTestLMO:
    # Test the detection and estimation of lane marking orientations
    def __init__(self, LMOrientation):
        imgbgr = cv2.imread('../images/visual_control/pic2_rect.png')

        theta_left, theta_right = LMOrientation(imgbgr)
        print()
        print('Function returned the following orientations for the given image:')
        print('    Left Edge:   %.2f radians (%.2f degrees)' % (theta_left, theta_left*180/np.pi))
        print('    Right Edge:  %.2f radians (%.2f degrees)' % (theta_right, theta_right*180/np.pi))

        fig = plt.figure()
        ax1 = fig.add_subplot(1, 1, 1)
        # OpenCV uses BGR by default, whereas matplotlib uses RGB, so we generate an RGB version for the sake of visualization
        ax1.imshow(cv2.cvtColor(imgbgr, cv2.COLOR_BGR2RGB))
        ax1.set_title('Do these orientations look right?'), ax1.set_xticks([]), ax1.set_yticks([])