#!/usr/bin/env python

#Import Python libraries
import numpy as np
import random

# Import ROS libraries and message types
import rospy


class MatConfiguration():
    
    def __init__(self):
       
        # Pyrobot parameters
        self.use_numerical_ik = rospy.get_param('~use_numerical_ik', False)  # default boolean for using numerical method when solving IK
        
        # Initialize standard operation parameters (when not picking/placing)
        working_pose = rospy.get_param('~working_pose')
        self.working_z, self.working_r, self.working_p = working_pose['z'], working_pose['roll'], working_pose['pitch']
        self.table_z = rospy.get_param('~table_z')

        # Initialize calibration information
        mat_color_location = rospy.get_param('~mat_color_location')
        mat_color_position = np.array([mat_color_location['x'], mat_color_location['y'], self.working_z])
        self.mat_color_pose = {'position': mat_color_position, 'pitch': self.working_p, 'roll': self.working_r, 'numerical': self.use_numerical_ik}
	scale_color_location = rospy.get_param('~scale_color_location')
        scale_color_position = np.array([scale_color_location['x'], scale_color_location['y'], self.working_z])
        self.scale_color_pose = {'position': scale_color_position, 'pitch': self.working_p, 'roll': self.working_r, 'numerical': self.use_numerical_ik}

        
        # Initialize pickup area
        pickup_positions = []
        pickup_area = rospy.get_param('~pickup_area')
        self.survey_z = rospy.get_param('~survey_z')
        num_steps_x = np.ceil(pickup_area['length_x'] / pickup_area['step_size'])
        num_steps_y = np.ceil(pickup_area['length_y'] / pickup_area['step_size'])
        for i in range(0, int(num_steps_x)):
            x = pickup_area['offset_x'] + (i+0.5) * pickup_area['length_x'] / num_steps_x
            y_vals = range(0, int(num_steps_y))
            for j in (y_vals if i % 2 == 0 else reversed(y_vals)):
                y = pickup_area['offset_y'] + (j+0.5) * pickup_area['length_y'] / num_steps_y
                pickup_positions.append([x, y, self.survey_z])
        self.pickup_positions = np.array(pickup_positions)
        self.num_pickup_positions = np.size(self.pickup_positions, 0)
        
        # Initialize scale location
        scale_location = rospy.get_param('~scale_location')
        self.scale_position = np.array([scale_location['x'], scale_location['y'], self.working_z])
        self.scale_z = scale_location['z']
       
        # Initialize camera location (for placing sherd for archival photo)
        cam_location = rospy.get_param('~cam_location')
        self.camera_position = np.array([cam_location['x'], cam_location['y'], self.working_z])
        self.camera_z = self.table_z #cam_location['z']

        # Initialize standby location and pose (out of archival camera frame)
        standby_location = rospy.get_param('~standby_location')
        standby_position = np.array([standby_location['x'], standby_location['y'], self.working_z])
        self.standby_pose = {"position": standby_position, "pitch": self.working_p, "roll": self.working_r, "numerical": self.use_numerical_ik}

        # Initialize scale survey location and pose (if first regrasp attempt fails)
        scale_survey_location = rospy.get_param('~scale_survey_location')
        self.scale_survey_position = np.array([scale_survey_location['x'], scale_survey_location['y'], self.survey_z])

        # Initialize camera survey location (for reacquiring sherd after archival photo)
        cam_survey_location = rospy.get_param('~cam_survey_location')
        self.camera_survey_position = np.array([cam_survey_location['x'], cam_survey_location['y'], self.survey_z])
      
        # Initialize discard_area
        discard_area = rospy.get_param('~discard_area')
        self.discard_z = self.table_z #discard_area['z']
        self.discard_offset_x, self.discard_offset_y = discard_area['offset_x'], discard_area['offset_y']
        self.discard_length_x, self.discard_length_y = discard_area['length_x'], discard_area['length_y'] # dimensions of rectangular pickup area

        # Stations
        self.stations = {
            'pickup': 0,
            'scale_place': 1,
            'scale_pick': 2,
            'scale_search': 3,
            'camera_place': 4,
            'camera_pick': 5,
            'dropoff': 6
        }


    # Print a string of the current station
    def print_station(self, station):
        station_string = [key for (key, value) in self.stations.items() if value == station]
        rospy.loginfo('Moving to station: {}'.format(*station_string))
    

    # Get pose for the station
    def get_goal_pose(self, station):
        if station == self.stations['pickup']:
            pos = self.pickup_positions[0]
        elif station == self.stations['scale_place'] or station == self.stations['scale_pick']:
            pos = self.scale_position 
        elif station == self.stations['scale_search']: # if first 'blind' reacquisition at scale fails
            pos = self.scale_survey_position
        elif station == self.stations['camera_place']:
            pos = self.camera_position
        elif station == self.stations['camera_pick']: 
            pos = self.camera_survey_position # always survey before trying to reacquire from camera area
        elif station == self.stations['dropoff']:
            pos = self.dropoff_position()
        return {"position": pos, "pitch": self.working_p, "roll": self.working_r, "numerical": self.use_numerical_ik}


    # Get z height for a goal position depending on the station
    def select_goal_z(self, pose, station):
        if station == self.stations['pickup']:
            pose['position'][2] = self.table_z 
        elif station == self.stations['scale_place'] or station == self.stations['scale_pick'] or station == self.stations['scale_search']:
            pose['position'][2] = self.scale_z
        elif station == self.stations['camera_place'] or station == self.stations['camera_pick']:
            pose['position'][2] = self.camera_z
        elif station == self.stations['dropoff']:
            pose['position'][2] = self.table_z
        return pose


    # Get pickup pose based on index
    def pickup_pose(self, index):
        search_pose = {
            'position': self.pickup_positions[index], 
            'pitch': self.working_p, 
            'roll': self.working_r, 
            'numerical': self.use_numerical_ik
        }
        return search_pose

    
    # Randomly draw dropoff location from box
    def dropoff_position(self):
        rospy.logdebug('AutoCore: Discard triggered.')
        # Generate random location in dropoff area
        discard_x = self.discard_offset_x + random.random() * self.discard_length_x
        discard_y = self.discard_offset_y + random.random() * self.discard_length_y
        dropoff_pos = np.array([discard_x, discard_y, self.working_z])
        rospy.logdebug('Dropff position: {}'.format(dropoff_pos))
        return dropoff_pos
