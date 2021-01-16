#!/usr/bin/env python

import rospy
import rosbag
import rospkg
import os
import yaml
from matplotlib import pyplot as plt
import numpy as np

##############################################################
# get_metrics()
# This function initiates a ROS node that parses data processed sherds recorded in bagfiles. Bagfile(s) to be parsed and names of subroutines to expect in the bagfile(s) are loaded from a YAML configuration file.
# inputs: None
# outputs: 1) pie chart of processing time per sherd, broken down by subroutine, 2) printout of times for all sherds, 3) printout of number of grasp failures (drops), 4) printout of number of sherds processed.

# This script parses the data for processed sherds recorded in bagfiles
def get_metrics():
    rospy.init_node('get_metrics')
    rospack = rospkg.RosPack()

    # load bagfiles ( {filename: end_time, ... } ) and status timings ( {status_name: duration} ) from YAML file
    package_path = rospack.get_path('robot_arm')

    # convert dictionaries in YAML file to Python dictionaries with rospy.Time and rospy.Duration objects
    # YAML file contains dictionary of bagfiles {bagfile_name: simulation end_time} and dictionary of statuses {status: duration}
    yaml_file = 'metrics.yaml'
    filename = os.path.join(package_path,'config',yaml_file)
    stream = file(filename, 'r')
    metrics = yaml.load(stream)
    bagfiles = metrics['bagfiles']
    timing = metrics['statuses']

    # calculate time logged for each status, summed across all bagfiles listed in YAML file
    total_time = 0
    num_sherds = 0
    num_drops = 0
    for bagfile, end_time in bagfiles.items(): 
        start_time = None
        current_status = ''           
        filename = os.path.join(package_path,'bagfiles/benchmarks',bagfile)
        bag = rosbag.Bag(filename) # create bag object
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
                        timing[current_status] += time - start_time # update duration of current status {status: duration}
                    current_status = msg.data # update current status
                    start_time = time # update start time
                elif topic == '/sherd_data' and not msg.incomplete: # count sherds processed and grasp failures
                    num_sherds += 1
                    try:
                        num_drops += msg.drops
                        print('{} drops at {} sherds'.format(num_drops, num_sherds))
                    except:
                        pass
        finally:
            bag.close()

    # print number of sherds processed and number of drop events
    print('{} sherds processed with {} grasp failures. For a bagfile with no grasp failure count, grasp failures must be counted manually by reviewing gazebo log files.'.format(num_sherds, num_drops))

    # show pie chart of time logged per sherd for each status
    sizes = []
    labels = []
    explode = (0.05, 0.05, 0.05)  # explode all slices

    # build wedges (sizes) and labels for pie chart
    for key in timing:
        if key != 'Initialization': # Initialization cannot be calculated per sherd
            sizes.append( float('{}'.format(timing[key].to_sec()))/num_sherds )
            labels.append('{}\n{} s/sherd'.format( key, round(timing[key].to_sec()/num_sherds, 1) ))
        print('{} took {} secs.'.format(key, timing[key].to_sec()))
        total_time += timing[key].to_sec()
    print('Total time: {} secs.'.format(total_time))

    fig, ax = plt.subplots()
    
    wedges, labels, autopcts = ax.pie(sizes, explode=explode, labels=labels, labeldistance=1.15, autopct='%.0f%%', shadow=True, 					startangle=-60, textprops={'fontsize':18})
    plt.setp(labels, fontsize=24) # status labels
    plt.setp(autopcts, fontsize=24, weight="bold", color="w") # percentage labels
    #autopcts[1].set_color('black') # if 'Initialization' included
    ax.axis('equal')
    #plt.title('Simulated Time by Subroutine\nper Sherd Processed', fontdict = {'fontsize' : 26})
    plt.show()

##############################################################
# main function

if __name__ == '__main__':
    get_metrics()
