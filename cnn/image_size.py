import os
from PIL import Image
from pathlib import Path
import json

img_dir = './sherd_datasets/training/'

json_file = open('./sherd_datasets/training/train_annotations_coco.json', 'r')

data = json.load(json_file)
json_file.close()
print('json_file closed\n')

# need on loop for png and one for jpg
pathlist = Path(img_dir).glob('*.png')
for path in pathlist:
    path_str = str(path)
    print(path)
    img = Image.open(path)
    print('\n\nImage Width: ', img.size[0], 'Image Height: ', img.size[1])
    for i in data['images']:
        print(i)
        print(i['file_name'])
        print(path_str[24:])
        if i['file_name'] == path_str[24:]:
            i['width'] = img.size[0]
            i['height'] = img.size[1]
            print('\n', i, '\n')

# need on loop for png and one for jpg
'''
pathlist = Path(img_dir).glob('*.png')
for path in pathlist:
    print(path)
    img = Image.open('./sherd_datasets/training/histarchTypeGallery_21_front.jpg')
'''