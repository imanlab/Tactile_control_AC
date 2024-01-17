#!/usr/bin/env python
import sys
import threading
import time
import os
import cv2 as cv
import numpy as np
import rospy
from detectron2.config import get_cfg
from detectron2 import model_zoo
from detectron2.data import MetadataCatalog
from cv_bridge import CvBridge, CvBridgeError
# import some common detectron2 utilities
from detectron2.engine import DefaultPredictor
from detectron2.utils.logger import setup_logger
from detectron2.utils.visualizer import Visualizer, ColorMode
#from BerryDetection.claras.inference_strawberry_segmentation.msg import Result
from sensor_msgs.msg import Image, RegionOfInterest
from std_msgs.msg import Float64MultiArray

working_dir = os.path.abspath(os.path.dirname(sys.argv[0]))
# Create a named window
cv.namedWindow('inference')

# Resize the window
cv.resizeWindow('inference', 1500, 1200)
centroid = []

class Detectron2node(object):
    def __init__(self):
        rospy.logwarn("Initializing")
        setup_logger()

        self._bridge = CvBridge()
        self._last_msg = None
        self._msg_lock = threading.Lock()
        self._image_counter = 0

        self.cfg = get_cfg()
        self.cfg.merge_from_file(model_zoo.get_config_file("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml"))
        self.cfg.DATASETS.TRAIN = ("strawberry_train",)
        self.cfg.DATASETS.TEST = ()
        self.cfg.DATALOADER.NUM_WORKERS = 2
        self.cfg.SOLVER.IMS_PER_BATCH = 2
        self.cfg.SOLVER.BASE_LR = 0.00025  # pick a good LR
        self.cfg.SOLVER.MAX_ITER = 1100
        self.cfg.SOLVER.STEPS = []         # do not decay learning rate
        self.cfg.MODEL.ROI_HEADS.BATCH_SIZE_PER_IMAGE = 128   # faster, and good enough for this toy dataset (default: 512)
        self.cfg.MODEL.ROI_HEADS.NUM_CLASSES = 1  # only has one class (strawberry).
        self.cfg.MODEL.WEIGHTS = os.path.join(working_dir, "model_final.pth")  # path to the model we just trained
        self.cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.7   # set a custom testing threshold
        self.predictor = DefaultPredictor(self.cfg)

        self._visualization = True
        self.topic_name = "/centroid"
        self.message_type = Float64MultiArray

        self._cent_pub = rospy.Publisher(self.topic_name,self.message_type,queue_size = 10)
        self._vis_pub = rospy.Publisher('/visualization', Image, queue_size=1)
        self._sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback_image, queue_size=1)
        self.start_time = time.time()
        rospy.logwarn("Initialized")


    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self._msg_lock.acquire(False):
                img_msg = self._last_msg
                self._last_msg = None
                self._msg_lock.release()
            else:
                rate.sleep()
                continue

            if img_msg is not None:
                self._image_counter = self._image_counter + 1
                if (self._image_counter % 11) == 10:
                    rospy.loginfo("Images detected per second=%.2f",
                                  float(self._image_counter) / (time.time() - self.start_time))

                
                np_image = self.convert_to_cv_image(img_msg)
                outputs = self.predictor(np_image)
                outputs = outputs["instances"].to("cpu")
                #result_msg = self.getResult(result)

                #self._result_pub.publish(result_msg)

                # Visualize results
                if self._visualization:
                    v = Visualizer(np_image[:, :, ::-1], 
                                    MetadataCatalog.get(self.cfg.DATASETS.TRAIN[0]), 
                                    scale=1,
                                    instance_mode=ColorMode.IMAGE)
                    v = v.draw_instance_predictions(outputs)
                    img = v.get_image()[:, :, ::-1]

                  #extract top left-corner coordinates and display
                    centroid = self.extract_centroid(outputs)

                    # if centroid is not None:
                    #         cv.putText(img, f"Centroid: {centroid}", (centroid[0], centroid[1] - 10),
                    #                     cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2) #display
         
                    cv.imshow('inference',img)
                    key = cv.waitKey(1)
                    if key == ord('q'):
                        break

                    image_msg = self._bridge.cv2_to_imgmsg(img)

                    self._vis_pub.publish(image_msg)

                    self.message =Float64MultiArray(data=centroid)
                    self._cent_pub.publish(self.message)
                    

            rate.sleep()

    def extract_centroid(self, outputs):
        pred_boxes = outputs.pred_boxes.tensor.numpy() if outputs.has("pred_boxes") else None
        if pred_boxes is not None:
            for box in pred_boxes: #check because this should just be one element
                box = [int(coord) for coord in box]
                top_left = (box[0], box[1])
                bottom_right = (box[2], box[3])
                centroid = ((top_left[0] + bottom_right[0]) / 2, (top_left[1] + bottom_right[1]) / 2)
                return centroid
        return None


    def convert_to_cv_image(self, image_msg):

        if image_msg is None:
            return None

        self._width = image_msg.width
        self._height = image_msg.height
        channels = int(len(image_msg.data) / (self._width * self._height))

        encoding = None

        if image_msg.encoding.lower() in ['rgb8', 'bgr8']:
            encoding = np.uint8
        elif image_msg.encoding.lower() == 'mono8':
            encoding = np.uint8
    
        cv_img = np.ndarray(shape=(image_msg.height, image_msg.width, channels),
                            dtype=encoding, buffer=image_msg.data)
        
        if image_msg.encoding.lower() == 'mono8':
            cv_img = cv.cvtColor(cv_img, cv.COLOR_RGB2GRAY)

        else:
            cv_img = cv.cvtColor(cv_img, cv.COLOR_RGB2BGR)

        return cv_img


    def callback_image(self, msg):
        rospy.logdebug("Get an image")
        if self._msg_lock.acquire(False):
            self._last_msg = msg
            self._header = msg.header
            self._msg_lock.release()



def main(argv):
    rospy.init_node('detectron2_ros')
    node = Detectron2node()
    node.run()

if __name__ == '__main__':
    main(sys.argv)