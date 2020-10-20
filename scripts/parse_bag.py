#!/usr/bin/env python

import rospy
import rosbag
import os
import yaml

# This script parses the data for processed sherds recorded in bagfiles

if __name__ == "__main__":

    # load sherd data bagfile
    current_dir = os.getcwd()
    package_dir = os.path.abspath(os.path.join(current_dir, os.pardir))
    bagfile = 'sherd_data_2020-10-17-19-30-47.bag'  # edit name of bagfile as necessary
    filename = os.path.join(package_dir,'bagfiles',bagfile)
    bag = rosbag.Bag(filename)

    # convert statuses listed in YAML file to Python dictionary with rospy.Duration items
    yaml_file = 'statuses.yaml'
    filename = os.path.join(package_dir,'config',yaml_file)
    statuses = file(filename, 'r')
    timing = yaml.load(statuses)
    current_status = ''
    start_time = None
    num_sherds = 0

    # calculate time logged for each status
    try:
        for topic, msg, t in bag.read_messages():
            if topic == '/clock': # simulated time from Gazebo
                time = msg.clock
            elif topic == '/status':
                if not start_time is None:
                    timing[current_status] += time - start_time
                current_status = msg.data
                start_time = time
            elif topic == '/sherd_data' and not msg.incomplete:
                num_sherds += 1
    finally:
        bag.close()

    # print time logged for each status
    for key in timing:
        print('{} took {} secs'.format(key, timing[key].to_sec()))
