#!/usr/bin/env python3
import numpy as np
import rospy
import rospkg

from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from sensor_msgs.msg import CompressedImage, Image
from duckietown_msgs.msg import Twist2DStamped, LanePose, WheelsCmdStamped, BoolStamped, FSMState, StopLineReading, AntiInstagramThresholds
from image_processing.anti_instagram import AntiInstagram
import cv2
from object_detection.model import Wrapper
from cv_bridge import CvBridge
from integration import NUMBER_FRAMES_SKIPPED, filter_by_classes, filter_by_bboxes, filter_by_scores

class ObjectDetectionNode(DTROS):

    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(ObjectDetectionNode, self).__init__(
            node_name=node_name,
            node_type=NodeType.PERCEPTION
        )
        self.initialized = False



        # Construct publishers
        self.pub_obj_dets = rospy.Publisher(
            "~duckie_detected",
            BoolStamped,
            queue_size=1,
            dt_topic_type=TopicType.PERCEPTION
        )

        # Construct subscribers
        self.sub_image = rospy.Subscriber(
            "~image/compressed",
            CompressedImage,
            self.image_cb,
            buff_size=10000000,
            queue_size=1
        )
        
        self.sub_thresholds = rospy.Subscriber(
            "~thresholds",
            AntiInstagramThresholds,
            self.thresholds_cb,
            queue_size=1
        )
        
        self.ai_thresholds_received = False
        self.anti_instagram_thresholds=dict()
        self.ai = AntiInstagram()
        self.bridge = CvBridge()

        model_file = rospy.get_param('~model_file','.')
        rospack = rospkg.RosPack()
        model_file_absolute = rospack.get_path('object_detection') + model_file
        self.model_wrapper = Wrapper(model_file_absolute)
        self.frame_id = 0
        self.initialized = True
        self.log("Initialized!")
    
    def thresholds_cb(self, thresh_msg):
        self.anti_instagram_thresholds["lower"] = thresh_msg.low
        self.anti_instagram_thresholds["higher"] = thresh_msg.high
        self.ai_thresholds_received = True

    def image_cb(self, image_msg):
        if not self.initialized:
            return

        if self.frame_id != 0:
            return
        self.frame_id += 1
        self.frame_id = self.frame_id % (1 + NUMBER_FRAMES_SKIPPED())

        # Decode from compressed image with OpenCV
        try:
            image = self.bridge.compressed_imgmsg_to_cv2(image_msg)
        except ValueError as e:
            self.logerr('Could not decode image: %s' % e)
            return
        
        # Perform color correction
        if self.ai_thresholds_received:
            image = self.ai.apply_color_balance(
                self.anti_instagram_thresholds["lower"],
                self.anti_instagram_thresholds["higher"],
                image
            )
        
        image = cv2.resize(image, (416,416))
        bboxes, classes, scores = self.model_wrapper.predict(image)
        
        msg = BoolStamped()
        msg.header = image_msg.header
        msg.data = self.det2bool(bboxes, classes, scores)
        
        self.pub_obj_dets.publish(msg)
    
    def det2bool(self, bboxes, classes, scores):
        print(f"Before filtering: {len(bboxes)} detections")

        box_ids = np.array(list(map(filter_by_bboxes, bboxes))).nonzero()
        cla_ids = np.array(list(map(filter_by_classes, classes))).nonzero()
        sco_ids = np.array(list(map(filter_by_scores, scores))).nonzero()

        print(box_ids)
        print(cla_ids)
        print(sco_ids)

        box_cla_ids = np.intersect1d(box_ids, cla_ids)
        box_cla_sco_ids = np.intersect1d(box_cla_ids, sco_ids)

        print(f"After filtering: {len(box_cla_sco_ids)} detections")

        if len(box_cla_sco_ids) > 0:
            return True
        else:
            return False

if __name__ == "__main__":
    # Initialize the node
    object_detection_node = ObjectDetectionNode(node_name='object_detection_node')
    # Keep it spinning
    rospy.spin()
