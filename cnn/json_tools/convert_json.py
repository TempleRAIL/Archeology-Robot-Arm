import json

# open json file in read mode
json_file = open('./sherd_datasets/via_validation_coco.json', 'r')
print('\nvia.json file opened\n')

# convert json_file to python dictionary
data = json.load(json_file)
print(type(data))
json_file.close()
print('\nvia.json file closed\n')

# append a 'category_id' of '1' to each annotation
for idx in data['annotations']:
    idx.update({"category_id": 1})

# overwrite the existing file with the new appended information
json_file = open('./sherd_datasets/via_validation_coco.json', 'w')
print('\nvia.json file opened\n')
json.dump(data, json_file)
print('via.json file overwritten\n')
json_file.close()
print('via.json file closed\n')