import json

# open json file in read mode
json_file = open('./sherd_datasets/training/train_annotations_coco.json', 'r')

data = json.load(json_file)
json_file.close()
print('json_file closed\n')

# update the image_id to an int
for i in data['annotations']:
    i['image_id'] = int(i['image_id'])

# open existing file and overwrite data
json_file = open('./sherd_datasets/training/train_annotations_coco.json', 'w')
print('\nvia.json file opened\n')
json.dump(data, json_file)
print('via.json file overwritten\n')
json_file.close()
print('via.json file closed\n')