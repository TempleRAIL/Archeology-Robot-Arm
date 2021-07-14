import json

json_file = open('./sherd_datasets/via_validation_coco.json', 'r')

data = json.load(json_file)

print('Info: \n', data['info'])
print('Images: \n', data['images'][0])
print('Annotations: \n', data['annotations'][5])
print('Licenses: \n', data['licenses'][0])
print('Categories: \n', data['categories'][0])