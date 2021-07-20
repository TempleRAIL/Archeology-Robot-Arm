import torch, torchvision
print('Torch: ', torch.__version__, torch.cuda.is_available())

import detectron2
from detectron2.utils.logger import setup_logger
setup_logger()

# import some common libraries
import numpy as np
import os, json, cv2, random
import matplotlib.pyplot as plt

# import some common detectron2 utilities
from detectron2 import model_zoo
from detectron2.engine import DefaultPredictor
from detectron2.config import get_cfg
from detectron2.utils.visualizer import Visualizer
from detectron2.data import MetadataCatalog, DatasetCatalog
from detectron2.data.datasets import register_coco_instances
print('\nImported relevant libraries\n')

#register_coco_instances("sherd_dataset", {}, "/home/brandon-lutz/catkin_ws/src/Archeology-Robot-Arm/cnn/sherd_datasets/validation/val_annotations_coco.json", "/home/brandon-lutz/catkin_ws/src/Archeology-Robot-Arm/cnn/sherd_datasets/validation")
#print('\nRegistered \'sherd_dataset\'\n')

im = plt.imread('/home/brandon-lutz/catkin_ws/src/Archeology-Robot-Arm/cnn/sherd_datasets/validation/Plaquemine1.jpg')
#im = plt.imread('/home/brandon-lutz/Downloads/PennFudanPed/PNGImages/PennPed00014.png')
#im = plt.imread("./test_images/924187027_bc01449011_z.jpg")
plt.imshow(im)
plt.show()

cfg = get_cfg()
cfg.merge_from_file(model_zoo.get_config_file("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml"))
cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.7 # set threshold for this model
# Find a model from detectron2's model zoo. You can use the https://dl.fbaipublicfiles... url as well
cfg.MODEL.WEIGHTS = './weights/sherd_model.pth'
#cfg.DATASETS.TEST = ("sherd_dataset",)
#cfg.DATALOADER.NUM_WORKERS = 2
#cfg.MODEL.WEIGHTS = model_zoo.get_checkpoint_url("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml")
cfg.MODEL.DEVICE = 'cpu'
cfg.MODEL.ROI_HEADS.NUM_CLASSES = 1
#cfg.MODEL.ROI_HEADS.BATCH_SIZE_PER_IMAGE = 512

print('\nConfiguration is set. Making Predictions.\n')
predictor = DefaultPredictor(cfg)
outputs = predictor(im)

# look at the outputs. See https://detectron2.readthedocs.io/tutorials/models.html#model-output-format for specification
print('\nPrinting Prediction Classes:\n')
print(outputs["instances"].pred_classes)
print('\nPrinting Prediction Boxes:\n')
print(outputs["instances"].pred_boxes)

'''
# We can use `Visualizer` to draw the predictions on the image.
v = Visualizer(im[:, :, ::-1], MetadataCatalog.get(cfg.DATASETS.TEST[0]), scale=1.2)
out = v.draw_instance_predictions(outputs["instances"].to("cpu"))
plt.imshow(out.get_image()[:, :, ::-1])
plt.show()
'''