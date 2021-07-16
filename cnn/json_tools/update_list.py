import json

# open json file in read mode
json_file = open('./sherd_datasets/validation/val_annotations_coco.json', 'r')

# convert json file to python dictionary
data = json.load(json_file)

# close json_file
json_file.close()
print('json_file closed\n')

# append the segmentation list within itself to create a nested list
for i in data['annotations']:
    print('Annotations: \n', i['segmentation'])
    tmp = i['segmentation'].copy()
    i['segmentation'].clear()
    i['segmentation'].append(tmp)
    print('Annotations: \n', i['segmentation'])

# open existing file and overwrite data
json_file = open('./sherd_datasets/validation/val_annotations_coco.json', 'w')
print('\nvia.json file opened\n')
json.dump(data, json_file)
print('via.json file overwritten\n')
json_file.close()
print('via.json file closed\n')