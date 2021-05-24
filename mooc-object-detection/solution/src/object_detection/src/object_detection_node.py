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
        self.log("Initializing!")


        # Construct publishers
        self.pub_obj_dets = rospy.Publisher(
            "~duckie_detected",
            BoolStamped,
            queue_size=1,
            dt_topic_type=TopicType.PERCEPTION
        )

        self. pub_detections_image = rospy.Publisher(
            "~object_detections_img", Image, queue_size=1, dt_topic_type=TopicType.DEBUG
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
        self.veh = rospy.get_namespace().strip("/")
        aido_eval = rospy.get_param(f"/{self.veh}/AIDO_eval", False)
        self.log(f"AIDO EVAL VAR: {aido_eval}")
        self.log("Starting model loading!")
        self._debug = rospy.get_param("~debug", False)
        self.model_wrapper = Wrapper(aido_eval)
        self.log("Finished model loading!")
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

        if self._debug:
            colors = {0: (0, 255, 255), 1: (0, 165, 255), 2: (0, 250, 0), 3: (0, 0, 255)}
            names = {0: "duckie", 1: "cone", 2: "truck", 3: "bus"}
            font = cv2.FONT_HERSHEY_SIMPLEX
            for clas, box in zip(classes, bboxes):
                pt1 = np.array([int(box[0]), int(box[1])])
                pt2 = np.array([int(box[2]), int(box[3])])
                pt1 = tuple(pt1)
                pt2 = tuple(pt2)
                color = colors[clas.item()]
                name = names[clas.item()]
                image = cv2.rectangle(image, pt1, pt2, color, 2)
                text_location = (pt1[0], min(416, pt1[1]+20))
                image = cv2.putText(image, name, text_location, font, 1, color, thickness=3)
            obj_det_img = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
            self.pub_detections_image.publish(obj_det_img)


    def det2bool(self, bboxes, classes, scores):

        box_ids = np.array(list(map(filter_by_bboxes, bboxes))).nonzero()[0]
        cla_ids = np.array(list(map(filter_by_classes, classes))).nonzero()[0]
        sco_ids = np.array(list(map(filter_by_scores, scores))).nonzero()[0]


        box_cla_ids = set(list(box_ids)).intersection(set(list(cla_ids)))
        box_cla_sco_ids = set(list(sco_ids)).intersection(set(list(box_cla_ids)))


        if len(box_cla_sco_ids) > 0:
            return True
        else:
            return False

if __name__ == "__main__":
    # Initialize the node
    object_detection_node = ObjectDetectionNode(node_name='object_detection_node')
    # Keep it spinning
    rospy.spin()
