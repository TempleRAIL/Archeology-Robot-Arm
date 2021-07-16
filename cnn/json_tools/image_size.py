import os
from PIL import Image
from pathlib import Path
import json

img_dir = './sherd_datasets/validation/'

# opening json file in read mode
json_file = open('./sherd_datasets/validation/val_annotations_coco.json', 'r')

# converting json file to python dictionary
data = json.load(json_file)

# closing json file
json_file.close()
print('json_file closed\n')

# this loop iterates the jpg files
pathlist = Path(img_dir).glob('*.jpg')
for path in pathlist:
    path_str = str(path)
    img = Image.open(path)
    for i in data['images']:
        if i['file_name'] == path_str[24:]:
            i['width'] = img.size[0]
            i['height'] = img.size[1]

# this loop iterates the png files
pathlist = Path(img_dir).glob('*.png')
for path in pathlist:
    path_str = str(path)
    img = Image.open(path)
    for i in data['images']:
        if i['file_name'] == path_str[24:]:
            i['width'] = img.size[0]
            i['height'] = img.size[1]

# opening json file in write mode and overwriting data
json_file = open('./sherd_datasets/validation/val_annotations_coco.json', 'w')
print('\nvia.json file opened\n')
json.dump(data, json_file)
print('via.json file overwritten\n')
json_file.close()
print('via.json file closed\n')