# import some common detectron2 utilities
import json
import os
import sys
import pathlib
import re
from tqdm import tqdm
import torch

import cv2
from detectron2 import model_zoo
from detectron2.engine import DefaultPredictor
from detectron2.config import get_cfg
from detectron2.utils.visualizer import Visualizer
from detectron2.utils.visualizer import ColorMode
from detectron2.data import MetadataCatalog, DatasetCatalog

working_dir = os.path.abspath(os.path.dirname(sys.argv[0]))
print(working_dir)


def read_json(file_path):
    with open(file_path, 'r') as read_file:
        data = json.load(read_file)
    read_file.close()
    return data


for d in ["train"]:
    DatasetCatalog.register("strawberry_" + d, lambda d=d: read_json(working_dir + d + '.json'))
    MetadataCatalog.get("strawberry_" + d).set(thing_classes=["strawberry"])
    strawberry_metadata = MetadataCatalog.get("strawberry_train")


cfg = get_cfg()
cfg.merge_from_file(model_zoo.get_config_file("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml"))
cfg.DATASETS.TRAIN = ("strawberry_train",)
cfg.DATASETS.TEST = ()
cfg.DATALOADER.NUM_WORKERS = 2
cfg.SOLVER.IMS_PER_BATCH = 2
cfg.SOLVER.BASE_LR = 0.00025  # pick a good LR
cfg.SOLVER.MAX_ITER = 1100
cfg.SOLVER.STEPS = []        # do not decay learning rate
cfg.MODEL.ROI_HEADS.BATCH_SIZE_PER_IMAGE = 128   # faster, and good enough for this toy dataset (default: 512)
cfg.MODEL.ROI_HEADS.NUM_CLASSES = 1  # only has one class (strawberry).
cfg.MODEL.WEIGHTS = os.path.join(working_dir, "model_final.pth")  # path to the model we just trained
cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.7   # set a custom testing threshold
predictor = DefaultPredictor(cfg)

dataset_dicts = []

# Create a named window
cv2.namedWindow('inference')

# Resize the window
cv2.resizeWindow('inference', 1500, 1200)

cap = cv2.VideoCapture(6)  # Use 0 for webcam, or replace with the path to a video file

while True:
    ret, frame = cap.read()
    if not ret:
        break

    outputs = predictor(frame)
    outputs = outputs['instances'].to('cpu')

    v = Visualizer(frame[:, :, ::-1],
                   metadata=strawberry_metadata,
                   scale=1,
                   instance_mode=ColorMode.IMAGE
                   )

    out = v.draw_instance_predictions(outputs)
    cv2.imshow('inference', out.get_image()[:, :, ::-1])
    key = cv2.waitKey(1)
    if key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()




