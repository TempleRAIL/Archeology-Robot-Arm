#!/usr/bin/env python

import rospy
import rosbag
import os
import yaml
from matplotlib import pyplot as plt
import numpy as np

# This script parses the data for processed sherds recorded in bagfiles

if __name__ == "__main__":

    # load sherd data bagfiles
    current_dir = os.getcwd()
    package_dir = os.path.abspath(os.path.join(current_dir, os.pardir))

    # bagfile names and simulation end times determined from Gazebo logfile playbacks
    bagfile_end_times = {'benchmark_1_2020-10-22-21-45-54.bag': rospy.Time.from_sec(402.154), 'benchmark_2_2020-10-26-15-59-59.bag': rospy.Time.from_sec(515)} 

    # convert statuses listed in YAML file to Python dictionary with rospy.Duration items
    yaml_file = 'statuses.yaml'
    filename = os.path.join(package_dir,'config',yaml_file)
    statuses = file(filename, 'r')
    timing = yaml.load(statuses)

    # calculate time logged for each status
    total_time = 0
    num_sherds = 0
    for bagfile, end_time in bagfile_end_times.items():
        start_time = None
        current_status = ''           
        filename = os.path.join(package_dir,'bagfiles',bagfile)
        bag = rosbag.Bag(filename) 
        try:           
            for topic, msg, t in bag.read_messages():
                if topic == '/clock': # simulated time from Gazebo
                    time = msg.clock
                    if time <= end_time:
                        pass
                    else:
                        break
                elif topic == '/status':
                    if not start_time is None:
                        timing[current_status] += time - start_time
                    current_status = msg.data
                    start_time = time
                elif topic == '/sherd_data' and not msg.incomplete:
                    num_sherds += 1
        finally:
            bag.close()

    # print number of sherds processed
    print('{} sherds processed.'.format(num_sherds, total_time))

    # show pie chart of time logged for each status
    sizes = []
    labels = []
    explode = (0.05, 0.05, 0.05, 0.05)  # explode all slices

    for key in timing:
        sizes.append( float('{}'.format(timing[key].to_sec())) )
        labels.append('{}\n{} secs'.format( key, round(timing[key].to_sec(), 3) ))
        print('{} took {} secs.'.format(key, timing[key].to_sec()))
        total_time += timing[key].to_sec()   
    print('Total time: {} secs.'.format(total_time))

    """
    def func(pct, allvals):
        absolute = float(pct/100.*np.sum(allvals))
        rounded = round(absolute, 3)
        return "{:.1f}%\n({} secs)".format(pct, rounded)
    """

    fig, ax = plt.subplots()
    ax.pie(sizes, explode=explode, labels=labels, autopct='%.0f%%', textprops={'fontsize': 20}, shadow=False, startangle=-60)
    ax.axis('equal')
    plt.title('Simulated Time by Subroutine\nper 10 Sherds Processed', fontdict = {'fontsize' : 50})
    plt.show()
