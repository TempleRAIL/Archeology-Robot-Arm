import json
from PIL import Image
import os

json_file = open('./sherd_datasets/training/train_annotations_coco.json', 'r')
#json_file = open('./instances_train2017.json', 'r')
data = json.load(json_file)

#print('Info: \n', data['info'])
#print('Images: \n', data['images'][0])
#print('Annotations: \n', type(data['annotations'][6]['segmentation'][0]))
print('Annotations: \n', data['images'][6]['height'])
#print('Licenses: \n', data['licenses'][0])
#print('Categories: \n', data['categories'][0])
#tmp = data['annotations'][6]['segmentation'].copy()
#data['annotations'][6]['segmentation'].clear()
#data['annotations'][6]['segmentation'].append(tmp)
#print('Annotations: \n', data['annotations'][6]['segmentation'])